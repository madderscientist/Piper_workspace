"""Opt-in Chromium layout/control checks; all SDK construction is forbidden."""

import json
import os
from pathlib import Path
import re
import shutil
import threading
import time

from ament_index_python.packages import get_package_share_directory
from conftest import MockDriverNode as DriverNode
from piper_x_driver.configuration import default_config_path, FleetConfig
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from state_dashboard.dashboard import DashboardNode, DashboardServer
import yaml


@pytest.mark.skipif(
    os.environ.get('PIPER_BROWSER_TESTS') != '1',
    reason='Set PIPER_BROWSER_TESTS=1 after installing Playwright Chromium')
@pytest.mark.usefixtures('fake_arm')
def test_browser_mit_sliders_configuration_and_explicit_close(tmp_path, monkeypatch):
    from playwright.sync_api import sync_playwright, expect

    path = tmp_path / 'arms.yaml'
    shutil.copyfile(default_config_path(), path)
    data = yaml.safe_load(path.read_text())
    for arm in data['arms']:
        arm['namespace'] = f'/browser_test_{os.getpid()}/' + arm['id']
    path.write_text(yaml.safe_dump(data, allow_unicode=True))
    fleet = FleetConfig(path)
    rclpy.init()
    executor = SingleThreadedExecutor()
    drivers = [
        DriverNode(namespace=arm.namespace, parameter_overrides=[
            Parameter('config_file', value=str(path))]) for arm in fleet.arms
    ]
    for driver in drivers:
        driver.control._clear_power_state()
        driver.arm._powered = False
        driver.arm._mode = 'standby'
        driver.arm._write_mode('standby')
        driver.arm.enabled = (False,) * 7
        driver.arm.calls.clear()
    drivers[0].arm.status.update(ctrl_mode=2, teach_status=2)
    dashboard = DashboardNode([arm.namespace for arm in fleet.arms])
    for node in drivers + [dashboard]:
        executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    assets = Path(get_package_share_directory('state_dashboard')) / 'dashboard'
    try:
        with DashboardServer(
            ('127.0.0.1', 0), dashboard.store, assets,
            dashboard.enable_controller, dashboard.arm_controller,
        ) as server:
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            try:
                url = f'http://127.0.0.1:{server.server_port}'
                with sync_playwright() as playwright:
                    browser = playwright.chromium.launch(headless=True)
                    page = browser.new_page(viewport={'width': 1440, 'height': 1100})
                    errors = []
                    page.on('pageerror', lambda error: errors.append(str(error)))

                    def until(predicate, timeout=5):
                        deadline = time.monotonic() + timeout
                        snapshot = {'arms': []}
                        while time.monotonic() < deadline:
                            snapshot = page.request.get(url + '/api/status').json()
                            if predicate(snapshot):
                                return snapshot
                            page.wait_for_timeout(50)
                        raise AssertionError(json.dumps([
                            {'namespace': arm['namespace'], 'power': arm['status']['power_state'],
                             'mode': arm['control']['mode']}
                            for arm in snapshot['arms']]))

                    page.goto(url)
                    cards = page.locator('.arm-row')
                    expect(cards).to_have_count(4)
                    first = cards.nth(0)
                    expect(first.locator('.namespace')).to_contain_text('can0')
                    expect(first.locator('.namespace')).to_contain_text('示教手柄')
                    expect(cards.nth(2).locator('.namespace')).to_contain_text('夹爪')
                    assert page.locator('.arm-card').count() == 0
                    assert page.evaluate(
                        "getComputedStyle(document.documentElement).colorScheme") == 'light'
                    assert page.evaluate(
                        "getComputedStyle(document.body).backgroundColor") == 'rgb(255, 255, 255)'
                    bounds = cards.evaluate_all("""rows => rows.map(row => {
                        const rect = row.getBoundingClientRect();
                        return {x: rect.x, y: rect.y, right: rect.right, bottom: rect.bottom};
                    })""")
                    assert all(abs(row['x']) < 1 and abs(row['right'] - 1440) < 1
                               for row in bounds), bounds
                    assert all(bounds[i]['y'] < bounds[i + 1]['y'] for i in range(3))
                    expect(first.locator('.wrench-section .wrench-value')).to_have_count(6)
                    assert first.locator('.mit-sliders').count() == 0
                    second = cards.nth(1)
                    drivers[1].arm.position = drivers[1].arm.position[:6] + (-0.0108,)
                    expect(second.locator('.gripper')).to_have_text('-10.8')
                    second.locator('.expand-arm-btn').click()
                    studio = page.locator('#arm-studio')
                    expect(studio).to_be_visible()
                    studio_box = studio.bounding_box()
                    assert studio_box is not None
                    assert (abs(studio_box['width'] - 1440) <= 2
                            and abs(studio_box['height'] - 1100) <= 2)
                    studio.locator('.studio-tab-btn[data-studio-tab="diag"]').click()
                    cal_btn = studio.locator('#studio-calibrate-btn')
                    expect(cal_btn).to_be_disabled()
                    studio.locator('#studio-cal-confirm-check').check()
                    expect(cal_btn).to_be_enabled()
                    cal_btn.click()
                    until(lambda s: s['arms'][1]['joints']['gripper_m'] == 0)
                    assert drivers[1].arm.calls == [('calibrate_effector',)]
                    assert all(
                        not driver.arm.calls
                        for index, driver in enumerate(drivers) if index != 1)

                    studio.locator('.studio-arm-tab').nth(0).click()
                    studio.locator('#studio-power-btn').click()
                    expect(first.locator('.power-state')).to_have_attribute(
                        'data-state', 'enabled')
                    expect(cal_btn).to_be_disabled()
                    studio.locator('.studio-tab-btn[data-studio-tab="mit"]').click()
                    studio.locator('#studio-mit-track-btn').click()
                    expect(studio.locator('#studio-mit-track-btn')).to_have_class(
                        re.compile(r'active'))
                    slider = studio.locator('.mit-row-slider').nth(0)
                    slider.evaluate("""node => {
                        node.value = '5';
                        node.dispatchEvent(new Event('input', {bubbles: true}));
                    }""")
                    until(lambda s: s['arms'][0]['joints']['position_deg'][0] > 4.5)
                    studio.locator('.studio-tab-btn[data-studio-tab="diag"]').click()
                    studio.locator('#studio-gravity-scale').evaluate("""node => {
                        node.value = '0.5';
                        node.dispatchEvent(new Event('input', {bubbles: true}));
                        node.dispatchEvent(new Event('change', {bubbles: true}));
                    }""")
                    state = until(lambda s: (
                        s['arms'][0]['control']['configuration']['gravity_scale'] == 0.5
                        and s['arms'][0]['control']['gravity_scale_applied'] == 0.5))
                    assert yaml.safe_load(path.read_text())['arms'][0]['gravity_scale'] == 0.5
                    assert all(a['status']['power_state'] == 'disabled' for a in state['arms'][1:])
                    studio.locator('.studio-tab-btn[data-studio-tab="mit"]').click()
                    studio.locator('#studio-mit-free-btn').click()
                    expect(studio.locator('#studio-mit-free-btn')).to_have_class(
                        re.compile(r'active'))
                    studio.locator('#studio-power-btn').click()
                    expect(first.locator('.power-state')).to_have_attribute(
                        'data-state', 'disabled', timeout=8000)
                    studio.locator('.studio-tab-btn[data-studio-tab="diag"]').click()
                    studio.locator('#studio-effector-select').select_option('gripper')
                    studio.locator('#studio-save-effector-btn').click()
                    expect(first.locator('.namespace')).to_contain_text('标准夹爪')
                    expect(studio.locator('#studio-mesh-canvas')).to_be_visible()
                    studio.locator('.studio-tab-btn[data-studio-tab="wrench"]').click()
                    expect(
                        studio.locator('#studio-wrench-cards .studio-wrench-box')
                    ).to_have_count(6)
                    expect(studio.locator('#studio-wrench-joint-tbody tr')).to_have_count(6)
                    studio.locator('.studio-tab-btn[data-studio-tab="diag"]').click()
                    expect(studio.locator('#studio-can-ages .can-age-item')).to_have_count(11)
                    studio.locator('#studio-close-btn').click()
                    expect(studio).not_to_be_visible()
                    page.set_viewport_size({'width': 390, 'height': 844})
                    assert not page.evaluate(
                        'document.documentElement.scrollWidth > window.innerWidth')
                    assert not errors, errors
                    browser.close()
            finally:
                server.shutdown()
                worker.join(timeout=3)
    finally:
        executor.shutdown()
        ros_thread.join(timeout=3)
        for node in drivers + [dashboard]:
            node.destroy_node()
        rclpy.try_shutdown()
