"""Read-only proof gate for the commissioned Docker image, never cabinet Mode."""
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess

IMAGE = 'sha256:7f4124d7fcfa95a604802bff20c1ae0f65a18b5d2e782cb88f49921dc63daab2'
ENDPOINT = '192.168.58.2'


def verify_simulator_proof(path, inspect=None, now=None):
    report = json.loads(Path(path).read_text())
    current = now or datetime.now(timezone.utc)
    age = (current-datetime.fromisoformat(report['captured_utc'])).total_seconds()
    if age < 0 or age > 30 or not report.get('read_only'):
        raise RuntimeError('Native simulator proof is stale or not read-only')
    if not report.get('native_transport_pass'):
        raise RuntimeError('Native simulator transport preflight failed')
    native = json.loads(report['native_probe_stdout'])
    if native.get('mode') != 0:
        raise RuntimeError('SimMachine is Manual: operator must select Auto; no automatic Mode call')
    if (native.get('enabled') != 1 or native.get('frame_head') != 0x5A5A or
            any(native.get(field) != 0 for field in ('rpc_rc','version_rc','state_rc',
                'main_error','sub_error','emergency','safety0','safety1')) or
            native.get('web_version') != 'v3.9.3' or
            native.get('sdk') != 'SDK V2.3.3.0-robot v3.9.3'):
        raise RuntimeError('Native simulator health/version proof failed')
    if inspect is None:
        inspect = lambda name: json.loads(subprocess.check_output(
            ['docker','inspect',name],text=True,timeout=5))[0]
    container = inspect(report['container'])
    if (container['Id'] != report['container_id'] or container['Image'] != IMAGE or
            report['image_digest'] != IMAGE or not container['State']['Running'] or
            container['NetworkSettings']['Networks']['fairino-net']['IPAddress'] != ENDPOINT or
            report['docker_verified_ip'] != ENDPOINT):
        raise RuntimeError('Native simulator Docker identity/endpoint changed')
    return {'force.native_simulator.verified': True,
            'force.native_simulator.image_digest': IMAGE,
            'force.native_simulator.endpoint': ENDPOINT}
