#!/usr/bin/env python3
"""Mechanical source/config/binary inventory for recorded force test artifacts."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
from datetime import datetime, timezone
import yaml

ROOT = Path(__file__).resolve().parents[4]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('artifact', type=Path)
    args = parser.parse_args()
    out = args.artifact.resolve()
    out.relative_to(ROOT / 'test_results')
    config = ROOT / 'src/fr10_arm_control/fr10_pose_controller/config'
    files = []
    for name in ('fr10_force_control', 'fr10_insertion_manager', 'fr10_motion_core',
                 'fr10_motion_planning', 'fr10_motion_interfaces', 'fr10_pose_controller'):
        files.extend(p for p in (ROOT/'src/fr10_arm_control'/name).rglob('*')
                     if p.is_file() and p.suffix in ('.hpp','.cpp','.yaml','.py','.xml','.action','.msg','.srv','.txt','.md'))
    files.extend(p for p in (ROOT/'src/vehicle_bringup/test/integration').glob('*force*.py'))
    driver = ROOT/'src/hardware_interface/fr10_driver/fairino_hardware_v3_9_3'
    files.extend(p for folder in ('src','include','libfairino/include')
                 for p in (driver/folder).rglob('*')
                 if p.is_file() and p.suffix in ('.cpp','.hpp','.h'))
    files.extend((driver/'CMakeLists.txt', driver/'package.xml'))
    files.extend([ROOT/'src/vehicle_bringup/test/integration/fr10_mujoco_bridge.py', ROOT/'src/vehicle_bringup/test/integration/fr10_gazebo_force_evidence.py',
        ROOT/'src/vehicle_bringup/test/integration/fr10_fake_wrench_bridge.py', ROOT/'src/vehicle_bringup/test/integration/fr10_wrench_fault_adapter.py',
        ROOT/'src/vehicle_bringup/test/integration/fr10_simmachine_readonly_probe.cpp',
        ROOT/'src/vehicle_bringup/test/integration/fr10_simmachine_preflight.py',
        ROOT/'src/fr10_arm_control/fairino10_v6_moveit2_config/worlds/fr10_force_home_test.world',
        ROOT/'src/vehicle_description/urdf/fr10_wrist.xacro',
        ROOT/'src/vehicle_description/models/cong_sac/model.sdf',
        ROOT/'src/vehicle_description/models/cong_sac/meshes/cong_sac.stl'])
    binaries = list((ROOT/'.colcon/rb_v3_humble/install').glob(
        'fr10_*/lib/**/*.so'))
    binaries.extend((ROOT/'.colcon/rb_v3_humble/install/fairino_hardware_v3_9_3/lib').rglob('*.so'))
    binaries.append(ROOT/'.colcon/rb_v3_humble/install/fr10_pose_controller/lib/fr10_pose_controller/fr10_pose_controller_node')
    binaries.append(ROOT/'src/hardware_interface/fr10_driver/fairino_hardware_v3_9_3/libfairino/lib/libfairino.so.2.3.3')
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = {'captured_utc': datetime.now(timezone.utc).isoformat(),
        'git_head': subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'dirty_worktree': subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True).splitlines(),
        'ros_distro':'humble', 'mujoco_version':importlib.metadata.version('mujoco'),
        'real_contact_test':False, 'force_default_enabled':False,
        'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in sorted(set(files))},
        'binary_sha256':{str(p.relative_to(ROOT)):sha(p) for p in binaries if p.is_file()},
        'simmachine_image_digest':'sha256:d5a3dedd8b839d2ab32a0fa7a65a5628f1092546ace2859715a44f0db1c19fb3',
        'simmachine_readonly_software_version':[0,'FR5-V1-001(V5.0)','v3.7.8','V3.7.68'],
        'simmachine_version_provenance':'XMLRPC GetSoftwareVersion on Docker-verified 192.168.58.2:20003; see simmachine/STATUS.md',
        'run_summaries':{str(p.relative_to(out)):sha(p) for p in sorted(out.glob('*/**/summary.json'))}}
    chosen = []
    for backend in ('fake','gazebo','mujoco'):
        chosen.extend((out/backend).glob('handoff_*/summary.json'))
        chosen.append(out/backend/'hold_ack_normal_01/summary.json')
    manifest['expected_simulation_handoff_runs'] = 50
    manifest['simulation_handoff_results'] = {
        str(p.relative_to(out)): {key: value for key,value in json.loads(p.read_text()).items()
            if key in ('pass','success','code','error','controlled_stop_completed',
                       'conditioned_frame_contract','post_result_hold_verified','pause_hold_verified')}
        for p in sorted(chosen) if p.is_file()}
    results = manifest['simulation_handoff_results'].values()
    manifest['simulation_handoff_gate_pass'] = (len(manifest['simulation_handoff_results']) == 50 and
                                                all(r.get('pass') is True for r in results))
    preflight = out/'simmachine/fr5_preflight_handoff.json'
    if not preflight.is_file(): preflight = out/'simmachine/fr5_preflight.json'
    if preflight.is_file():
        observed = json.loads(preflight.read_text())
        manifest['simmachine_image_digest'] = observed['image_digest']
        manifest['simmachine_readonly_software_version'] = observed.get('software_version')
        manifest['simmachine_version_provenance'] = str(preflight.relative_to(out))
        manifest['simmachine_preflight_sha256'] = sha(preflight)
        manifest['native_simmachine_transport_pass'] = observed['native_transport_pass']
    manifest['documentation_sha256'] = {
        str(p.relative_to(ROOT)):sha(p) for p in (
            out/'REPORT.md', out/'BASELINE.md', ROOT/'PROGRESS_LEDGER.md',
            ROOT/'docs/fr10_force_sensor_handoff.md') if p.is_file()}
    effective = {name:yaml.safe_load((config/name).read_text())
                 for name in ('controller.yaml','fake_force_test.yaml','gazebo_force_test.yaml','mujoco_force_test.yaml','gazebo_force_clock.yaml')}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (out/'effective_config.yaml').write_text(yaml.safe_dump(effective,sort_keys=False))
    print(f"Recorded {len(manifest['source_sha256'])} source hashes and {len(manifest['run_summaries'])} run summaries")


if __name__ == '__main__': main()
