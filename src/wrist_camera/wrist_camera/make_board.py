"""按 config/board.yaml 生成一张可打印的 ChArUco 标定板 PNG。"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import yaml

from wrist_camera.board import Board
from wrist_camera.rtsp import default_cameras_config_path

MM_PER_INCH = 25.4


def main(argv: list[str] | None = None) -> None:
    cfg_dir = default_cameras_config_path().parent
    parser = argparse.ArgumentParser(description='生成 ChArUco 标定板图')
    parser.add_argument('--config', default=str(cfg_dir / 'board.yaml'))
    parser.add_argument('--out', default='/tmp/charuco_board.png')
    parser.add_argument('--dpi', type=float, default=300.0)
    parser.add_argument('--margin-squares', type=float, default=0.5)
    args = parser.parse_args(argv)

    config = yaml.safe_load(Path(args.config).expanduser().read_text(encoding='utf-8'))
    board = Board.from_config(config)
    image = board.draw(args.dpi / MM_PER_INCH * 1000.0, args.margin_squares)
    cv2.imwrite(args.out, image)

    described = board.describe()
    print(f'{args.out}  {image.shape[1]}x{image.shape[0]} px @ {args.dpi:g} dpi')
    print(f"  {described['squares_x']}x{described['squares_y']} 格 · "
          f"方格 {described['square_size'] * 1000:g} mm · "
          f"marker {described['marker_size'] * 1000:g} mm · {described['dictionary']}")
    print(f"  板面 {described['width_m'] * 1000:g} x {described['height_m'] * 1000:g} mm · "
          f"{described['corner_count']} 个内角点 · "
          f"{described['squares_x'] * described['squares_y'] // 2} 个 marker")
    print('  打印时关掉缩放，打完量一格核对边长')


if __name__ == '__main__':
    main()
