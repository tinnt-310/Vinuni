#!/usr/bin/env python3
"""Launch one simulation profile with Gazebo/RViz and record GUI/ROS evidence."""
from __future__ import annotations

import argparse
import ctypes
import json
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener


WORKSPACE = Path(__file__).resolve().parents[4]


class GuiProbe(Node):
    def __init__(self):
        super().__init__('rb_v3_gui_profile_probe')
        self.clock_trace = []
        self.joints = []
        self.scans = 0
        self.odom = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self._joints, 10)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom, 10)

    def _on_clock(self, message):
        self.clock_trace.append(message.clock.sec + message.clock.nanosec * 1e-9)

    def _joints(self, message):
        self.joints.append(list(message.name))

    def _scan(self, _message):
        self.scans += 1

    def _odom(self, _message):
        self.odom += 1

    def controllers(self):
        client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        if not client.wait_for_service(timeout_sec=5):
            return []
        future = client.call_async(ListControllers.Request())
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            return []
        return [{'name': item.name, 'type': item.type, 'state': item.state}
                for item in future.result().controller]

    def tf_exists(self, parent, child):
        try:
            self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return True
        except Exception:
            return False


def process_alive(pid):
    if not pid:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def parse_window_inventory(text):
    rows = []
    for line in text.splitlines():
        match = re.search(
            r'\s*(0x[0-9a-f]+)\s+"([^"]*)".*?'
            r'\("([^"]*)"\s+"([^"]*)"\)\s+(\d+)x(\d+)[+-]',
            line, re.I)
        if match:
            rows.append((int(match.group(1), 16), match.group(2),
                         match.group(3), match.group(4),
                         int(match.group(5)), int(match.group(6))))
    return rows


def select_largest_window(windows, wm_class, excluded_ids=frozenset()):
    candidates = [row for row in windows
                  if row[3].lower() == wm_class.lower()
                  and row[0] not in excluded_ids]
    return max(candidates, key=lambda row: row[4] * row[5]) if candidates else None


def window_inventory(path):
    result = subprocess.run(['xwininfo', '-root', '-tree'], text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            check=False)
    path.write_text(result.stdout)
    return parse_window_inventory(result.stdout)


def top_level_window_id(window_id, root_id, parent_lookup):
    current = window_id
    visited = set()
    while current not in visited:
        visited.add(current)
        parent = parent_lookup(current)
        if parent in (0, root_id):
            return current
        current = parent
    return window_id


def raise_window(window_id):
    x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XQueryTree.argtypes = [
        ctypes.c_void_p, ctypes.c_ulong,
        ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
        ctypes.POINTER(ctypes.c_uint),
    ]
    x11.XQueryTree.restype = ctypes.c_int
    x11.XFree.argtypes = [ctypes.c_void_p]
    x11.XMapRaised.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x11.XSetInputFocus.argtypes = [
        ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = x11.XOpenDisplay(None)
    if not display:
        return False
    display_ptr = ctypes.c_void_p(display)
    try:
        root_id = x11.XDefaultRootWindow(display_ptr)

        def parent_lookup(candidate):
            root = ctypes.c_ulong()
            parent = ctypes.c_ulong()
            children = ctypes.POINTER(ctypes.c_ulong)()
            child_count = ctypes.c_uint()
            status = x11.XQueryTree(
                display_ptr, ctypes.c_ulong(candidate), ctypes.byref(root),
                ctypes.byref(parent), ctypes.byref(children),
                ctypes.byref(child_count))
            if children:
                x11.XFree(ctypes.cast(children, ctypes.c_void_p))
            return parent.value if status else 0

        top_level = top_level_window_id(window_id, root_id, parent_lookup)
        x11.XMapRaised(display_ptr, ctypes.c_ulong(top_level))
        x11.XRaiseWindow(display_ptr, ctypes.c_ulong(top_level))
        x11.XSetInputFocus(
            display_ptr, ctypes.c_ulong(window_id), 2, ctypes.c_ulong(0))
        x11.XFlush(display_ptr)
        time.sleep(1)
        return True
    finally:
        x11.XCloseDisplay(display_ptr)


def screenshot(path):
    from PIL import ImageGrab
    image = ImageGrab.grab()
    image.save(path)
    return {'path': str(path), 'width': image.width, 'height': image.height,
            'bytes': path.stat().st_size}


def decode_xwd_image(raw):
    if len(raw) < 100:
        raise ValueError('XWD data is shorter than its fixed header')
    header = struct.unpack('>25I', raw[:100])
    header_size = header[0]
    file_version = header[1]
    pixmap_format = header[2]
    width = header[4]
    height = header[5]
    byte_order = header[7]
    bits_per_pixel = header[11]
    bytes_per_line = header[12]
    red_mask, green_mask, blue_mask = header[14:17]
    color_count = header[19]
    if file_version != 7 or pixmap_format != 2:
        raise ValueError(
            f'unsupported XWD format: version={file_version}, '
            f'pixmap_format={pixmap_format}')
    bytes_per_pixel = bits_per_pixel // 8
    if (bits_per_pixel not in (24, 32)
            or bytes_per_line < width * bytes_per_pixel):
        raise ValueError(
            f'unsupported XWD pixel layout: bpp={bits_per_pixel}, '
            f'bytes_per_line={bytes_per_line}')
    pixel_offset = header_size + color_count * 12
    pixel_size = height * bytes_per_line
    if width <= 0 or height <= 0 or len(raw) < pixel_offset + pixel_size:
        raise ValueError('invalid or truncated XWD image')

    import numpy as np
    from PIL import Image

    if bits_per_pixel == 32:
        if bytes_per_line % 4:
            raise ValueError('32-bit XWD rows are not word-aligned')
        dtype = '<u4' if byte_order == 0 else '>u4'
        pixels = np.frombuffer(
            raw, dtype=dtype, count=pixel_size // 4, offset=pixel_offset)
        pixels = pixels.reshape(height, bytes_per_line // 4)[:, :width]
    else:
        rows = np.frombuffer(
            raw, dtype=np.uint8, count=pixel_size, offset=pixel_offset)
        packed = rows.reshape(height, bytes_per_line)
        packed = packed[:, :width * 3].reshape(height, width, 3)
        packed = packed.astype(np.uint32)
        if byte_order == 0:
            pixels = packed[:, :, 0] | (packed[:, :, 1] << 8)
            pixels |= packed[:, :, 2] << 16
        else:
            pixels = (packed[:, :, 0] << 16) | (packed[:, :, 1] << 8)
            pixels |= packed[:, :, 2]

    def channel(mask):
        if not mask:
            raise ValueError('XWD TrueColor channel mask is zero')
        shift = (mask & -mask).bit_length() - 1
        maximum = mask >> shift
        values = ((pixels & mask) >> shift).astype(np.uint32)
        return ((values * 255 + maximum // 2) // maximum).astype(np.uint8)

    rgb = np.stack(
        (channel(red_mask), channel(green_mask), channel(blue_mask)), axis=2)
    return Image.fromarray(rgb, mode='RGB')


def screenshot_window(window_id, path):
    xwd_path = path.with_name(f'.{path.name}.xwd')
    try:
        subprocess.run(
            ['xwd', '-silent', '-id', hex(window_id), '-out', str(xwd_path)],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, check=True)
        image = decode_xwd_image(xwd_path.read_bytes())
        image.save(path)
    finally:
        xwd_path.unlink(missing_ok=True)
    return {'path': str(path), 'width': image.width, 'height': image.height,
            'bytes': path.stat().st_size, 'window_id': hex(window_id),
            'capture_mode': 'direct_xwd'}


def run(output, profile, domain, port):
    output.mkdir(parents=True, exist_ok=False)
    vehicle = profile in ('vehicle', 'combined')
    arm = profile in ('arm', 'combined')
    expected = {'joint_state_broadcaster'}
    if vehicle:
        expected.add('swerve_controller')
    if arm:
        expected.add('fairino10_controller')
    env = os.environ.copy()
    # Desktop shells started from Snap applications can leak core20 GTK/glibc
    # modules into native Qt applications.  ROS/Gazebo/RViz must use the host
    # Jammy runtime and desktop data directories.
    host_xdg_data = env.get(
        'XDG_DATA_DIRS_VSCODE_SNAP_ORIG', '/usr/local/share:/usr/share')
    host_xdg_config = env.get('XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG', '/etc/xdg')
    for key in list(env):
        if key.startswith('SNAP'):
            env.pop(key, None)
    for key in ('PATH', 'LD_LIBRARY_PATH'):
        env[key] = ':'.join(item for item in env.get(key, '').split(':')
                            if item and not item.startswith('/snap/'))
    for key in ('LD_PRELOAD', 'GTK_MODULES', 'GTK_PATH', 'GTK_EXE_PREFIX',
                'GDK_PIXBUF_MODULEDIR', 'GDK_PIXBUF_MODULE_FILE',
                'GIO_MODULE_DIR', 'GSETTINGS_SCHEMA_DIR', 'LOCPATH'):
        env.pop(key, None)
    env['QT_ACCESSIBILITY'] = '0'
    env['XDG_DATA_DIRS'] = host_xdg_data
    env['XDG_CONFIG_DIRS'] = host_xdg_config
    env.update(ROS_DOMAIN_ID=str(domain), RMW_LOCALHOST_ONLY='1',
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}',
               GAZEBO_MODEL_DATABASE_URI='', ROS_LOG_DIR=str(output / 'ros_log'),
               LIBGL_ALWAYS_SOFTWARE='1')
    command = [
        'ros2', 'launch', 'vehicle_bringup', 'bringup.launch.py',
        f'use_vehicle:={str(vehicle).lower()}',
        f'use_arm:={str(arm).lower()}',
        'use_gazebo:=true', 'gui:=true', 'rviz:=true',
        f'gazebo_port:={port}',
    ]
    summary = {'profile': profile, 'domain': domain, 'port': port,
               'command': command, 'expected_controllers': sorted(expected)}
    runtime_log = output / 'runtime.log'
    preexisting_window_ids = {row[0] for row in window_inventory(
        output / 'x11_windows_before.txt')}
    with runtime_log.open('w') as log:
        launch = subprocess.Popen(command, cwd=WORKSPACE, env=env, stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        summary['launch_pid'] = launch.pid
        previous_domain = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(domain)
        rclpy.init()
        node = GuiProbe()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        import threading
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            deadline = time.monotonic() + 75
            ready_since = None
            controllers = []
            while time.monotonic() < deadline and launch.poll() is None:
                if len(node.clock_trace) >= 20 and node.joints:
                    controllers = node.controllers()
                    active = {row['name'] for row in controllers
                              if row['state'] == 'active'}
                    if expected <= active:
                        ready_since = ready_since or time.monotonic()
                        if time.monotonic() - ready_since >= 10:
                            break
                time.sleep(0.25)
            summary['controllers'] = controllers or node.controllers()
            summary['clock_samples'] = len(node.clock_trace)
            summary['clock_sim_elapsed_s'] = (
                node.clock_trace[-1] - node.clock_trace[0]
                if len(node.clock_trace) >= 2 else 0.0)
            summary['joint_state_samples'] = len(node.joints)
            summary['joint_names'] = sorted(set(node.joints[-1])) if node.joints else []
            summary['scan_samples'] = node.scans
            summary['odom_samples'] = node.odom
            if vehicle and arm:
                summary['tf_check'] = ['odom', 'charging_tcp']
            elif vehicle:
                summary['tf_check'] = ['odom', 'laser_frame']
            else:
                summary['tf_check'] = ['fr10_base_link', 'charging_tcp']
            summary['tf_connected'] = node.tf_exists(*summary['tf_check'])
            summary['launch_alive_at_capture'] = launch.poll() is None
            windows = window_inventory(output / 'x11_windows.txt')
            summary['windows'] = [
                {'id': hex(wid), 'title': title, 'instance': instance,
                 'class': wm_class, 'width': width, 'height': height}
                for wid, title, instance, wm_class, width, height in windows]
            gui_window = select_largest_window(
                windows, 'gazebo', preexisting_window_ids)
            rviz_window = select_largest_window(
                windows, 'rviz2', preexisting_window_ids)
            summary['gazebo_window'] = gui_window[1] if gui_window else None
            summary['rviz_window'] = rviz_window[1] if rviz_window else None
            if gui_window:
                summary['gazebo_screenshot'] = screenshot_window(
                    gui_window[0], output / 'gazebo.png')
            if rviz_window:
                summary['rviz_screenshot'] = screenshot_window(
                    rviz_window[0], output / 'rviz.png')
            summary['desktop_screenshot'] = screenshot(output / 'desktop.png')
        except Exception as error:
            summary['exception'] = repr(error)
        finally:
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            executor.remove_node(node)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if previous_domain is None:
                os.environ.pop('ROS_DOMAIN_ID', None)
            else:
                os.environ['ROS_DOMAIN_ID'] = previous_domain
            if launch.poll() is None:
                os.killpg(launch.pid, signal.SIGINT)
            try:
                summary['launch_exit'] = launch.wait(timeout=18)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                try:
                    summary['launch_exit'] = launch.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    summary['launch_exit'] = launch.wait(timeout=5)
                summary['forced_shutdown'] = True
    content = runtime_log.read_text(errors='replace')
    pids = {}
    for process in ('gzserver', 'gzclient', 'rviz2'):
        match = re.search(rf'\[{process}-\d+\]: process started with pid \[(\d+)\]',
                          content)
        pids[process] = int(match.group(1)) if match else None
    summary['process_pids'] = pids
    summary['rviz_error_lines'] = [line for line in content.splitlines()
                                   if re.search(r'\[rviz2-\d+\].*\[ERROR\]', line)]
    runtime_before_shutdown = content.split("sending signal 'SIGINT'", 1)[0]
    summary['gazebo_crash'] = bool(re.search(
        r'\[ERROR\] \[(?:gzserver|gzclient)-\d+\]: process has died',
        runtime_before_shutdown))
    active = {row['name'] for row in summary.get('controllers', [])
              if row['state'] == 'active'}
    wheel_names = {f'Wheel{i}_to_Revolute{i}' for i in range(1, 5)}
    steer_names = {f'Revolute{i}_to_base_link' for i in range(1, 5)}
    arm_names = {f'j{i}' for i in range(1, 7)}
    names = set(summary.get('joint_names', []))
    joint_contract = ((not vehicle or wheel_names | steer_names <= names) and
                      (not arm or arm_names <= names))
    summary['joint_contract_valid'] = joint_contract
    summary['pass'] = bool(
        summary.get('launch_alive_at_capture')
        and expected <= active
        and summary.get('clock_samples', 0) >= 20
        and summary.get('joint_state_samples', 0) >= 20
        and summary.get('tf_connected')
        and joint_contract
        and (not vehicle or summary.get('scan_samples', 0) >= 5)
        and (not vehicle or summary.get('odom_samples', 0) >= 5)
        and summary.get('gazebo_window') and summary.get('rviz_window')
        and summary.get('gazebo_screenshot') and summary.get('rviz_screenshot')
        and not summary.get('rviz_error_lines')
        and not summary.get('gazebo_crash'))
    (output / 'summary.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n')
    print(json.dumps({key: summary.get(key) for key in (
        'pass', 'profile', 'controllers', 'clock_sim_elapsed_s',
        'joint_state_samples', 'scan_samples', 'odom_samples', 'tf_check',
        'tf_connected', 'joint_contract_valid', 'gazebo_window', 'rviz_window',
        'rviz_error_lines', 'gazebo_crash', 'exception')}, indent=2))
    return 0 if summary['pass'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--profile', choices=('vehicle', 'arm', 'combined'), required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--port', type=int, required=True)
    args = parser.parse_args()
    raise SystemExit(run(args.output, args.profile, args.domain, args.port))
