#!/usr/bin/env python3
"""Read-only, Docker-scoped native SDK compatibility preflight; never actuates."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import subprocess
import xmlrpc.client


class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.0
        return connection


def collect_report(container_name, probe, soak_seconds=0.):
    if not 0. <= soak_seconds <= 60.:
        raise ValueError('Read-only state soak must be between0 and60 seconds')
    container = json.loads(subprocess.check_output(
        ['docker', 'inspect', container_name], text=True, timeout=5))[0]
    if not container['State']['Running']:
        raise RuntimeError('SimMachine container is not running')
    ip = container['NetworkSettings']['Networks']['fairino-net']['IPAddress']
    if not ip or ip in ('0.0.0.0', '127.0.0.1'):
        raise RuntimeError('Docker-owned simulator endpoint could not be resolved')
    report = {'captured_utc': datetime.now(timezone.utc).isoformat(),
              'container': container_name, 'container_id': container['Id'],
              'image_digest': container['Image'], 'docker_verified_ip': ip,
              'read_only': True, 'fr5_substitution_authorized': True,
              'real_contact_test': False, 'ports': {}}
    for port in (8080, 20003, 20004, 20005):
        try:
            with socket.create_connection((ip, port), timeout=2): pass
            report['ports'][str(port)] = 'open'
        except OSError as error:
            report['ports'][str(port)] = str(error)
    try:
        report['software_version'] = xmlrpc.client.ServerProxy(
            f'http://{ip}:20003', transport=BoundedTransport()).GetSoftwareVersion()
    except Exception as error:
        report['software_version_error'] = str(error)
    command = [str(probe.resolve()), ip]
    if soak_seconds:
        command += ['--soak-seconds', str(soak_seconds)]
    native = subprocess.run(command,
                            text=True, capture_output=True, timeout=20+soak_seconds)
    report.update(native_probe_exit=native.returncode,
                  native_probe_stdout=native.stdout,
                  native_probe_stderr=native.stderr)
    report['native_transport_pass'] = native.returncode == 0
    report['status'] = 'READ_ONLY_TRANSPORT_PASS' if native.returncode == 0 else 'BLOCKED_NATIVE_TRANSPORT'
    report['motion_authorized_by_preflight'] = False
    report['servo_rate_or_soak_pass'] = False
    if soak_seconds:
        state = json.loads(native.stdout.strip())
        report['state_read_soak_pass'] = state.get('state_read_soak_pass', False)
        report['state_read_soak_is_actuation_evidence'] = False
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container', default='fairino-container')
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--soak-seconds', type=float, default=0.,
                        help='Optional125 Hz read-only SDK state observation, never ServoJ')
    args = parser.parse_args()
    report = collect_report(args.container, args.probe, args.soak_seconds)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['native_transport_pass'] else 40


if __name__ == '__main__':
    raise SystemExit(main())
