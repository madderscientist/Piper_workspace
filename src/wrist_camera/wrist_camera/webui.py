"""标定面板的纯 Python HTTP 骨架（标准库 ThreadingHTTPServer，不依赖 Flask 或 rclpy）。"""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from pathlib import Path
import threading
import traceback
from urllib.parse import urlparse

_STATIC = Path(__file__).with_name('static')
_MAX_BODY = 1 << 20
_CTYPE = {'.html': 'text/html; charset=utf-8',
          '.js': 'application/javascript; charset=utf-8',
          '.css': 'text/css; charset=utf-8'}
_LOG = logging.getLogger('wrist_camera.webui')


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 128


def make_handler(actions: dict, route):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, format: str, *args) -> None:  # noqa: A002
            pass

        def send_bytes(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def send_json(self, payload, code: int = 200) -> None:
            self.send_bytes(code, json.dumps(payload, ensure_ascii=False).encode('utf-8'),
                            'application/json; charset=utf-8')

        def send_static(self, name: str) -> None:
            path = (_STATIC / name).resolve()
            if _STATIC.resolve() not in path.parents or not path.is_file():
                return self.send_json({'error': 'not found'}, 404)
            self.send_bytes(200, path.read_bytes(),
                            _CTYPE.get(path.suffix, 'application/octet-stream'))

        def body(self) -> dict:
            n = int(self.headers.get('Content-Length') or 0)
            if n <= 0:
                return {}
            if n > _MAX_BODY:
                raise ValueError('请求体过大')
            return json.loads(self.rfile.read(n) or b'{}')

        def _fail(self, exc: Exception) -> None:
            if isinstance(exc, (RuntimeError, ValueError, KeyError)):
                return self.send_json({'ok': False, 'error': str(exc)}, 400)
            _LOG.error('%s 失败: %s', self.path, traceback.format_exc())
            return self.send_json(
                {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}, 500)

        def do_GET(self) -> None:                  # noqa: N802
            try:
                return route(self, urlparse(self.path))
            except Exception as exc:               # noqa: BLE001
                return self._fail(exc)

        def do_POST(self) -> None:                 # noqa: N802
            fn = actions.get(urlparse(self.path).path)
            if fn is None:
                return self.send_json({'ok': False, 'error': 'not found'}, 404)
            try:
                return self.send_json({'ok': True, 'result': fn(self.body())})
            except Exception as exc:               # noqa: BLE001
                return self._fail(exc)

    return Handler


class Panel:
    """后台线程 HTTP 标定面板。"""

    def __init__(self, handler, port: int, name: str,
                 host: str = '0.0.0.0') -> None:
        self.handler = handler
        self.port, self.host, self.name = port, host, name
        self._srv: _Server | None = None

    def start(self) -> None:
        self._srv = _Server((self.host, self.port), self.handler)
        self.port = int(self._srv.server_port)
        threading.Thread(target=self._srv.serve_forever, daemon=True, name='calib-http').start()
        print(f'{self.name} -> http://127.0.0.1:{self.port}', flush=True)

    def stop(self) -> None:
        if self._srv is not None:
            self._srv.shutdown()
            self._srv.server_close()
            self._srv = None
