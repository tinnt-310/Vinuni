#!/usr/bin/env python3
"""Lease admission/replay/expiry/release checks on the dynamic mounted vehicle.

This test never requests contact or FAIRINO hardware. Daily startup remains
the public ros2 launch; this harness only records regression evidence.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from run_mounted_pose_gazebo_smoke import WORKSPACE, ProbeContext, list_controllers


def run(output, domain, port):
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), RMW_LOCALHOST_ONLY='1',
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}', GAZEBO_MODEL_DATABASE_URI='',
               ROS_LOG_DIR=str(output/'ros_log'))
    os.environ['RMW_LOCALHOST_ONLY'] = '1'
    command = ['ros2', 'launch', 'vehicle_bringup', 'bringup.launch.py',
               'use_vehicle:=true', 'use_arm:=true', 'use_gazebo:=true',
               'gui:=false', 'rviz:=false', f'gazebo_port:={port}']
    result = {'command': command, 'real_contact': False, 'gate': 'vehicle lease, not insertion'}
    trace = []
    with (output/'runtime.log').open('w') as log:
        launch = subprocess.Popen(command, cwd=WORKSPACE, env=env, stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        try:
            with ProbeContext(domain) as probe:
                from fr10_motion_interfaces.msg import VehicleParkingLease, VehicleParkingState
                from geometry_msgs.msg import Twist
                from nav_msgs.msg import Odometry
                statuses, odometry = [], []
                probe.create_subscription(VehicleParkingState, '/vehicle/parking_state',
                    lambda message: statuses.append((time.monotonic(), message)), 10)
                def on_odom(message):
                    row = {'wall': time.monotonic(), 'stamp': message.header.stamp.sec +
                        message.header.stamp.nanosec*1e-9, 'x': message.pose.pose.position.x,
                        'y': message.pose.pose.position.y, 'vx': message.twist.twist.linear.x,
                        'vy': message.twist.twist.linear.y}
                    odometry.append(row); trace.append(row)
                probe.create_subscription(Odometry, '/odom', on_odom, 10)
                drive = probe.create_publisher(Twist, '/cmd_vel', 10)
                lease_pub = probe.create_publisher(VehicleParkingLease, '/vehicle/parking_lease', 10)
                result['ready'] = probe.wait_ready(timeout=90)
                if not result['ready']: raise RuntimeError('mounted runtime not ready')
                result['controllers'] = list_controllers(probe)
                time.sleep(1)
                lease = VehicleParkingLease(); lease.owner_id = 731; lease.sequence = 1
                lease.lease_duration.nanosec = 300000000
                moving = Twist(); moving.linear.x = .08
                for _ in range(20):
                    drive.publish(moving); time.sleep(.05)
                lease_pub.publish(lease)
                deadline = time.monotonic()+4
                while time.monotonic()<deadline and (not statuses or statuses[-1][1].state != 2):
                    lease.sequence += 1; lease_pub.publish(lease); drive.publish(moving); time.sleep(.05)
                result['stationary_ack'] = bool(statuses and statuses[-1][1].owner_id == 731 and statuses[-1][1].state == 2)
                parked_start = time.monotonic()
                # Replayed heartbeat cannot renew expiry. Competing owner cannot
                # release or commandeer the parked actuators.
                intruder = VehicleParkingLease(); intruder.owner_id = 732; intruder.sequence = 100
                intruder.release = True
                lease_pub.publish(intruder)
                for _ in range(24):
                    drive.publish(moving); lease_pub.publish(lease); time.sleep(.05)
                result['expired_latched'] = bool(statuses and statuses[-1][1].owner_id == 731 and statuses[-1][1].state == 3)
                parked_rows = [row for row in odometry if row['wall'] > parked_start+.3]
                result['parked_peak_velocity_m_s'] = max((abs(row['vx'])+abs(row['vy']) for row in parked_rows), default=1.)
                result['rejected_drive_commands'] = statuses[-1][1].rejected_drive_commands if statuses else 0
                drive.publish(Twist()); time.sleep(.1)
                lease.sequence += 1; lease.release = True; lease_pub.publish(lease)
                time.sleep(.2)
                result['release_ack'] = bool(statuses and statuses[-1][1].owner_id == 0)
                release_start = time.monotonic()
                for _ in range(20): drive.publish(moving); time.sleep(.05)
                released_rows = [row for row in odometry if row['wall'] > release_start+.3]
                result['fresh_drive_after_release_m_s'] = max((abs(row['vx']) for row in released_rows), default=0.)
                drive.publish(Twist()); time.sleep(.5)
        except Exception as error:
            result['error'] = repr(error)
        finally:
            if launch.poll() is None:
                os.killpg(launch.pid, signal.SIGINT)
                try: launch.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGTERM); launch.wait(timeout=10)
    runtime = (output/'runtime.log').read_text(errors='replace')
    result['ode_lcp_errors'] = runtime.count('LCP internal error')
    result['pass'] = all(result.get(key, False) for key in ('ready', 'stationary_ack',
        'expired_latched', 'release_ack')) and result.get('parked_peak_velocity_m_s', 1) <= 1e-4 and \
        result.get('fresh_drive_after_release_m_s', 0) > .03 and result.get('rejected_drive_commands', 0) > 0 and \
        result['ode_lcp_errors'] == 0 and 'error' not in result
    (output/'odom_trace.json').write_text(json.dumps(trace, indent=2)+'\n')
    (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['pass'] else 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('output', type=Path)
    parser.add_argument('--domain', type=int, default=229); parser.add_argument('--port', type=int, default=11979)
    args = parser.parse_args(); raise SystemExit(run(args.output.resolve(), args.domain, args.port))
