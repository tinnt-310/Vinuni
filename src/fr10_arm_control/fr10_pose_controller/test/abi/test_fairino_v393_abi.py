#!/usr/bin/env python3
"""Verify that the v3.9.3 plugin resolves exactly its colocated SDK 2.3.3."""

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys


def run(command, env=None):
    return subprocess.run(
        command, check=False, capture_output=True, text=True, env=env)


def dynamic(path):
    result = run(["readelf", "-d", str(path)])
    return result.stdout + result.stderr


def ldd(path, env):
    result = run(["ldd", "-r", str(path)], env=env)
    return result.stdout + result.stderr, result.returncode


def resolution(output):
    match = re.search(r"^\s*libfairino\.so\.2\s+=>\s+(\S+)", output, re.MULTILINE)
    return match.group(1) if match else ""


def linker_error(output):
    lowered = output.lower()
    return "not found" in lowered or "undefined symbol" in lowered


def add(checks, check_id, description, passed, evidence):
    checks.append({
        "id": check_id,
        "description": description,
        "result": "PASS" if passed else "FAIL",
        "evidence": evidence,
    })


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--plugin", type=Path, required=True)
    parser.add_argument("--backend", type=Path, required=True)
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    plugin = args.plugin.resolve()
    backend = args.backend.resolve()
    sdk = plugin.parent / "libfairino.so.2.3.3"
    expected_sdk = str(sdk.resolve())

    inventory = []
    for root_name in ("src", "build", "install", "build_grouped", "install_grouped"):
        root = workspace / root_name
        if root.exists():
            for candidate in sorted(root.rglob("libfairino.so*")):
                inventory.append({
                    "path": str(candidate),
                    "target": str(candidate.resolve(strict=False)),
                    "kind": "symlink" if candidate.is_symlink() else "file",
                })

    checks = []
    add(checks, "ABI-01", "FAIRINO SDK inventory is non-empty",
        bool(inventory), inventory)

    pose_prefixes = tuple(f"{os.sep}{root}{os.sep}fr10_pose_controller{os.sep}"
                          for root in ("install", "install_grouped"))
    pose_providers = [item["path"] for item in inventory
                      if any(prefix in item["path"] for prefix in pose_prefixes)]
    add(checks, "ABI-02", "fr10_pose_controller installs no FAIRINO SDK provider",
        not pose_providers, pose_providers)

    full_output, full_rc = ldd(plugin, os.environ.copy())
    full_resolved = resolution(full_output)
    full_canonical = str(Path(full_resolved).resolve()) if full_resolved else ""
    add(checks, "ABI-03", "full overlay resolves the adjacent v3.9.3 SDK",
        full_rc == 0 and full_canonical == expected_sdk and not linker_error(full_output),
        {"expected": expected_sdk, "resolved": full_resolved,
         "canonical": full_canonical, "ldd": full_output})

    backend_dynamic = dynamic(backend)
    add(checks, "ABI-04", "controller backend has no direct libfairino dependency",
        "Shared library: [libfairino.so" not in backend_dynamic, backend_dynamic)

    plugin_dynamic = dynamic(plugin)
    add(checks, "ABI-05", "plugin needs libfairino.so.2 and pins $ORIGIN search",
        "Shared library: [libfairino.so.2]" in plugin_dynamic
        and "$ORIGIN" in plugin_dynamic, plugin_dynamic)

    sdk_dynamic = dynamic(sdk)
    add(checks, "ABI-06", "SDK SONAME is libfairino.so.2",
        "Library soname: [libfairino.so.2]" in sdk_dynamic, sdk_dynamic)

    clean_env = {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "LANG": os.environ.get("LANG", "C.UTF-8"),
        "LD_LIBRARY_PATH": ":".join((
            "/opt/ros/humble/lib/x86_64-linux-gnu", "/opt/ros/humble/lib")),
    }
    clean_output, clean_rc = ldd(plugin, clean_env)
    clean_resolved = resolution(clean_output)
    clean_canonical = str(Path(clean_resolved).resolve()) if clean_resolved else ""
    add(checks, "ABI-07", "clean environment resolves the same adjacent SDK",
        clean_rc == 0 and clean_canonical == expected_sdk
        and not linker_error(clean_output),
        {"expected": expected_sdk, "resolved": clean_resolved,
         "canonical": clean_canonical, "ldd": clean_output})

    loaded = run([
        "/usr/bin/python3", "-c",
        "import ctypes,sys; ctypes.CDLL(sys.argv[1], mode=ctypes.RTLD_GLOBAL)",
        str(plugin),
    ], env=clean_env)
    add(checks, "ABI-08", "plugin dlopen succeeds in a clean environment",
        loaded.returncode == 0,
        {"returncode": loaded.returncode, "stdout": loaded.stdout,
         "stderr": loaded.stderr})

    report = {
        "workspace": str(workspace),
        "plugin": str(plugin),
        "expected_sdk": expected_sdk,
        "checks": checks,
        "passed": sum(item["result"] == "PASS" for item in checks),
        "failed": sum(item["result"] == "FAIL" for item in checks),
    }
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(report, indent=2) + "\n")

    for item in checks:
        print(f"{item['id']} {item['result']}: {item['description']}")
        if item["result"] == "FAIL":
            print(json.dumps(item["evidence"], indent=2))
    print(f"ABI checks: {report['passed']} passed, {report['failed']} failed")
    return 0 if report["failed"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
