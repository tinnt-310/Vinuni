"""Identity/health gate tests do not contact Docker or send robot commands."""
from datetime import datetime, timedelta, timezone
import importlib.util
import json
from pathlib import Path

import pytest

SOURCE = Path(__file__).resolve().parents[4]/'src/fr10_arm_control/fr10_pose_controller/launch/native_simulator_gate.py'
SPEC = importlib.util.spec_from_file_location('native_simulator_gate', SOURCE)
GATE = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(GATE)
NOW = datetime(2026,10,2,tzinfo=timezone.utc)


def fixture(tmp_path):
    native = dict(mode=0,enabled=1,frame_head=0x5A5A,web_version='v3.9.3',
                  sdk='SDK V2.3.3.0-robot v3.9.3')
    native.update({k:0 for k in ('rpc_rc','version_rc','state_rc','main_error',
                                'sub_error','emergency','safety0','safety1')})
    report = dict(captured_utc=NOW.isoformat(),read_only=True,native_transport_pass=True,
                  native_probe_stdout=json.dumps(native),container='fairino-container',container_id='test-id',
                  image_digest=GATE.IMAGE,docker_verified_ip=GATE.ENDPOINT)
    container = dict(Id='test-id',Image=GATE.IMAGE,State={'Running':True},
        NetworkSettings={'Networks':{'fairino-net':{'IPAddress':GATE.ENDPOINT}}})
    path = tmp_path/'proof.json'; path.write_text(json.dumps(report))
    return path,report,container,native


def test_pinned_docker_auto_health_proof(tmp_path):
    path,_,container,_ = fixture(tmp_path)
    result = GATE.verify_simulator_proof(path,lambda _:container,NOW)
    assert result['force.native_simulator.verified']
    assert result['force.native_simulator.endpoint'] == GATE.ENDPOINT


@pytest.mark.parametrize('field,value',[('mode',1),('enabled',0),('frame_head',0),
    ('main_error',1),('safety0',1),('emergency',1),('state_rc',-1),('web_version','v3.7.8')])
def test_manual_or_faulted_sdk_never_authorizes(tmp_path,field,value):
    path,report,container,native = fixture(tmp_path)
    native[field] = value; report['native_probe_stdout'] = json.dumps(native)
    path.write_text(json.dumps(report))
    with pytest.raises(RuntimeError): GATE.verify_simulator_proof(path,lambda _:container,NOW)


def test_stale_or_changed_docker_identity_never_authorizes(tmp_path):
    path,report,container,_ = fixture(tmp_path)
    with pytest.raises(RuntimeError,match='stale'):
        GATE.verify_simulator_proof(path,lambda _:container,NOW+timedelta(seconds=31))
    for key,value in [('Image','different-image'),('Id','replacement-container')]:
        changed=dict(container);changed[key]=value
        with pytest.raises(RuntimeError,match='identity'):
            GATE.verify_simulator_proof(path,lambda _:changed,NOW)
    container['NetworkSettings']['Networks']['fairino-net']['IPAddress']='192.168.58.3'
    with pytest.raises(RuntimeError,match='identity'):
        GATE.verify_simulator_proof(path,lambda _:container,NOW)
