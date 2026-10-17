#!/usr/bin/env python3
"""Capture reproducible source/Git/environment evidence for matched insertion."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import zipfile
from datetime import datetime, timezone


def runtime_binary_inventory(prefixes):
    selected = {name: str(Path(prefix).resolve()) for name, prefix in prefixes.items()
                if name.startswith('fr10_') or name == 'fairino_hardware_v3_9_3'}
    if 'fr10_pose_controller' not in selected:
        raise ValueError('ROS workspace is not loaded: fr10_pose_controller was not found. '
                         'Run: source /opt/ros/humble/setup.bash && source install/setup.bash '
                         'from /home/tinnt/Vinuni, then retry in the same terminal.')
    binaries = set()
    for prefix in selected.values():
        binaries.update(p for p in (Path(prefix)/'lib').rglob('*.so*') if p.is_file())
    node = Path(selected['fr10_pose_controller'])/'lib/fr10_pose_controller/fr10_pose_controller_node'
    if not node.is_file():
        raise ValueError('Active ament runtime has no motion-node executable')
    binaries.add(node)
    return selected, sorted(binaries)


def capture(root, output, external_binaries=()):
    output.mkdir(parents=True, exist_ok=True)
    sources = {}
    for folder in ('src/fr10_arm_control', 'src/vehicle_bringup',
                   'src/vehicle_description', 'src/swerve_vehicle_control',
                   'src/hardware_interface/fr10_simulation_hardware',
                   'src/hardware_interface/fr10_driver/fairino_hardware_v3_9_3', 'src/vehicle_bringup/test/integration'):
        for path in sorted((root / folder).rglob('*')):
            if path.is_file() and not any(p in ('__pycache__', '.git') for p in path.parts):
                sources[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    # Save the actual text sources/configuration that produced this run.
    # Large CAD and binary assets remain addressed by their recorded hashes.
    archive=output/'source_snapshot.zip'
    text_suffixes={'.py','.cpp','.hpp','.h','.yaml','.yml','.json','.xacro','.xml','.srdf','.md','.sh','.txt','.cmake'}
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED) as snapshot:
        for name in sources:
            path=root/name
            if path.suffix in text_suffixes or path.name=='CMakeLists.txt': snapshot.write(path,name)
    git = lambda *args: subprocess.check_output(['git', *args], cwd=root, text=True)
    versions = {}
    for package in ('numpy', 'scipy', 'mujoco'):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    from ament_index_python.packages import get_packages_with_prefixes
    prefixes, binaries = runtime_binary_inventory(get_packages_with_prefixes())
    result = {'captured_utc': datetime.now(timezone.utc).isoformat(),
              'source_snapshot_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),
              'root': str(root), 'commit': git('rev-parse', 'HEAD').strip(),
              'git_status': git('status', '--porcelain=v1'),
              'git_diff_stat': git('diff', '--stat'), 'source_sha256': sources,
              'python_packages': versions,
              'runtime_package_prefix': prefixes,
              'binary_sha256': {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in binaries if p.is_file()},
              'external_binary_sha256': {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                                        for p in external_binaries},
              'contact_acceptance': 'pending; inspect run summary and independent contact evidence'}
    (output / 'manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    print(f'Captured {len(sources)} source/assets in {output / "manifest.json"}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    capture(Path(__file__).resolve().parents[4], args.output)
