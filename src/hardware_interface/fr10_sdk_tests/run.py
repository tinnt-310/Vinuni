#!/usr/bin/env python3
"""Standalone FAIRINO SDK bench. Dry-run unless --execute is explicit."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import cnde

ROOT = Path(__file__).resolve().parent
CASES = {
    "state": ("read-only", "SDK version and real-time state"),
    "ft_read": ("read-only", "FT_GetConfig, FT_GetForceTorqueOrigin/RCS"),
    "servoj": ("servo", "ServoMoveStart, ServoJ, ServoMoveEnd"),
    "servocart_absolute": ("servo", "ServoCart mode=0; exaxis is explicit"),
    "servocart_base": ("servo", "ServoCart mode=1; incremental base"),
    "servocart_tool": ("servo", "ServoCart mode=2; incremental tool"),
    "servojt": ("torque", "ServoJTStart, ServoJT(torque,interval), ServoJTEnd"),
    "servojt_limits": ("torque", "ServoJT with checkFlag=3, power/velocity limits"),
    "ft_control_basic": ("force", "FT_Control basic overload"),
    "ft_control_mb": ("force", "FT_Control overload with M/B"),
    "ft_control_extended": ("force", "FT_Control with M/B, threshold, adjustCoeff"),
    "ft_movel": ("force", "FT_Control basic + short tangential MoveL with force measurement"),
    "ft_compliance": ("force", "FT_ComplianceStart/Stop"),
    "ft_spiral": ("force", "FT_Control + FT_SpiralSearch"),
    "ft_linear": ("force", "FT_Control + FT_LinInsertion"),
    "ft_rotational": ("force", "FT_Control + FT_RotInsertion"),
    "ft_surface": ("force", "FT_Control + FT_FindSurface"),
    "ft_zero": ("calibration", "FT_SetZero; changes controller sensor zero"),
    "ft_payload_record": ("calibration", "FT_PdIdenRecord at manually prepared pose"),
    "ft_payload_compute": ("calibration", "FT_PdIdenCompute; record must already exist"),
    "cnde": ("cnde", "UDP output configuration, start, measured period, stop"),
}
GOOD = {"OBSERVED", "MOTION_OBSERVED", "API_ACCEPTED_UNVERIFIED"}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def config_issue(binary, config_path, name):
    if not binary.is_file():
        return "Build sdk_probe first: bash build.sh"
    if name == "cnde":
        try:
            cnde.validate(json.loads(config_path.read_text()))
            return None
        except (KeyError, TypeError, ValueError) as error:
            return str(error)
    checked = subprocess.run([str(binary), "--config", str(config_path), "--case", name,
                              "--output", "/unused", "--validate-only"],
                             capture_output=True, text=True, timeout=5)
    return None if checked.returncode == 0 else checked.stderr.strip()


def worker(binary, config_path, name, output, timeout, exclusive):
    output.mkdir(parents=True, exist_ok=False)
    command = [str(binary), "--config", str(config_path), "--case", name,
               "--output", str(output), "--execute"]
    if exclusive:
        command.append("--exclusive-control")
    with (output / "stdout.log").open("w") as stdout, (output / "stderr.log").open("w") as stderr:
        process = subprocess.Popen(command, stdout=stdout, stderr=stderr, start_new_session=True)
        timed_out = False
        try:
            process.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            timed_out = True
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        file = output / "result.json"
        if file.exists():
            result = json.loads(file.read_text())
        else:
            result = {"case": name, "status": "WORKER_FAILED", "cleanup_confirmed": False,
                      "error": "No final result; inspect events.jsonl and stderr.log"}
        if timed_out:
            result["status"] = "TIMEOUT_CLEANUP_UNCONFIRMED" if not result.get("cleanup_confirmed") else "TIMEOUT"
        result["worker_exit_code"] = process.returncode
        file.write_text(json.dumps(result, indent=2) + "\n")
        return result


@contextmanager
def endpoint_lock(ip):
    key = hashlib.sha256(ip.encode()).hexdigest()[:16]
    with open(f"/tmp/fr10_sdk_bench_{key}.lock", "w") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("Another SDK bench already owns this endpoint") from error
        yield


def report(path, results, metadata):
    data = {**metadata, "results": results,
            "note": "API_ACCEPTED_UNVERIFIED is not proof of force behavior or successful insertion."}
    (path / "summary.json").write_text(json.dumps(data, indent=2) + "\n")
    lines = ["# FAIRINO SDK bench", "", f"Endpoint: `{metadata['ip']}`; target label: `{metadata['target']}`.", "",
             "| Case | Result | Detail |", "|---|---|---|"]
    for r in results:
        detail = str(r.get("error", "")).replace("|", "/").replace("\n", " ")
        if r.get("identity_mismatches"):
            detail += " Identity mismatch: " + ", ".join(r["identity_mismatches"])
        lines.append(f"| {r['case']} | {r['status']} | {detail} |")
        if r.get("force_tracking"):
            lines.append(f"\nMeasured tangent displacement: {r.get('measured_tangential_mm')} mm. "
                         f"Force statistics (middle 10–90% of stroke): `{json.dumps(r['force_tracking'])}`\n")
    lines += ["", data["note"], "", "Each case stores parameters, calls/return codes and fresh state samples."]
    (path / "summary.md").write_text("\n".join(lines) + "\n")


def compare(left, right):
    def load(path):
        p = Path(path)
        data = json.loads((p / "summary.json" if p.is_dir() else p).read_text())
        return {r["case"]: r for r in data["results"]}
    a, b = load(left), load(right)
    print("| Case | A | B |")
    print("|---|---|---|")
    for name in CASES:
        if name in a or name in b:
            print(f"| {name} | {a.get(name, {}).get('status', 'NOT_RUN')} | {b.get(name, {}).get('status', 'NOT_RUN')} |")
    for label, entries in (("A", a), ("B", b)):
        identity = next((r.get("versions") for r in entries.values() if r.get("versions")), None)
        print(f"\n{label} versions: {json.dumps(identity, ensure_ascii=False)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config.example.json")
    parser.add_argument("--binary", type=Path, default=ROOT / "build/sdk_probe")
    parser.add_argument("--list", action="store_true")
    select = parser.add_mutually_exclusive_group()
    select.add_argument("--case", choices=CASES)
    select.add_argument("--suite", choices=["all", "read-only", "servo", "torque", "force", "calibration", "cnde"], default="read-only")
    parser.add_argument("--ip")
    parser.add_argument("--target", choices=["real", "sim"])
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--exclusive-control", action="store_true", help="Operator has stopped ROS and all competing command clients")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--compare", nargs=2, metavar=("RUN_A", "RUN_B"))
    args = parser.parse_args()
    if args.compare:
        compare(*args.compare)
        return 0
    if args.list:
        for name, (suite, description) in CASES.items():
            print(f"{name:24s} {suite:12s} {description}")
        return 0
    config = json.loads(args.config.read_text())
    if args.ip:
        config["ip"] = args.ip
    if args.target:
        config["target"] = args.target
    if config["target"] not in {"real", "sim"}:
        parser.error("config target must be real or sim")
    timeout = config.get("timeout_s")
    if type(timeout) not in (int, float) or not 5 <= timeout <= 120:
        parser.error("timeout_s must be in [5,120]")
    selected = [args.case] if args.case else [n for n, (s, _) in CASES.items() if args.suite == "all" or s == args.suite]
    if args.execute and any(CASES[n][0] != "read-only" for n in selected):
        if not args.case:
            parser.error("Run each mutating case explicitly with --case; --suite all is an offline inventory")
        if not args.exclusive_control:
            parser.error("This case requires --exclusive-control; stop ROS/other command clients first")
    if args.execute:
        def interrupted(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupted)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = (args.output or ROOT / "results" / stamp).resolve()
    output.mkdir(parents=True, exist_ok=False)
    config_path = output / "config.json"
    config_path.write_text(json.dumps(config, indent=2) + "\n")
    binary = args.binary.resolve()
    metadata = {"captured_utc": datetime.now(timezone.utc).isoformat(), "ip": config["ip"],
                "target": config["target"], "execute": args.execute, "config_source": str(args.config.resolve()),
                "binary": str(binary), "binary_sha256": sha256(binary) if binary.is_file() else None,
                "python": sys.version, "config_sha256": sha256(config_path)}
    results = []
    with endpoint_lock(config["ip"]) if args.execute else null_context():
        for name in selected:
            issue = config_issue(binary, config_path, name)
            if issue:
                result = {"case": name, "status": "CONFIG_REQUIRED", "error": issue}
            elif not args.execute:
                result = {"case": name, "status": "PLANNED", "description": CASES[name][1]}
            elif name == "cnde":
                preflight = worker(binary, config_path, "state", output / "cnde_preflight", timeout, False)
                if preflight["status"] != "OBSERVED" or preflight.get("identity_mismatches"):
                    result = {"case": name, "status": "BLOCKED_IDENTITY_OR_TRANSPORT", "preflight": preflight}
                else:
                    result = cnde.run(config, output / name)
                    result["versions"] = preflight.get("versions")
                    (output / name / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            else:
                result = worker(binary, config_path, name, output / name, timeout, args.exclusive_control)
            results.append(result)
            print(f"{name}: {result['status']}", flush=True)
            report(output, results, metadata)
            if args.execute and result["status"] not in GOOD:
                break
    print(f"Report: {output / 'summary.md'}")
    return int(args.execute and (len(results) != len(selected) or any(r["status"] not in GOOD for r in results)))


@contextmanager
def null_context():
    yield


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
