#!/usr/bin/env python3
"""Collect one uninterrupted >=50,000-cycle SimMachine telemetry record."""

from __future__ import annotations

import argparse
import pathlib
import time

from common import (ManagedLaunch, ProbeContext, analyze_log, check_preflight,
                    make_results_dir, write_json)


def fields(status: str):
    result = {}
    for item in status.split():
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        try:
            result[key] = float(value)
        except ValueError:
            result[key] = value
    return result


def run(robot_ip: str, domain_id: int, results_dir: pathlib.Path,
        minimum_cycles: int) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    preflight = check_preflight(robot_ip)
    write_json(results_dir / "preflight.json", preflight)
    if not preflight["all_ok"]:
        return 2

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    context = ProbeContext(domain_id)
    probe = None
    result = {"minimum_cycles": minimum_cycles}
    try:
        probe = context.__enter__()
        if not probe.wait_ready(timeout=30.0):
            result.update(result="FAIL", reason="runtime not ready")
            return 1
        begun = time.monotonic()
        last_report = -1
        while time.monotonic() - begun < minimum_cycles / 100.0 + 120.0:
            status = probe.status()
            parsed = fields(status)
            samples = int(parsed.get("loop_samples", 0))
            report_bucket = samples // 6250
            if report_bucket > last_report:
                print(f"SOAK_PROGRESS loop_samples={samples}", flush=True)
                last_report = report_bucket
            if samples >= minimum_cycles:
                sdk = analyze_log(launch.log_path)
                passed = (parsed.get("sdk_errors") == 0.0 and
                          123.5 <= parsed.get("loop_hz", 0.0) <= 126.5 and
                          sum(sdk.values()) == 0)
                result.update(
                    result="PASS" if passed else "FAIL",
                    elapsed_s=time.monotonic() - begun,
                    final_status=status,
                    telemetry=parsed,
                    sdk_audit=sdk,
                )
                return 0 if passed else 1
            time.sleep(0.2)
        result.update(result="FAIL", reason="cycle timeout",
                      final_status=probe.status())
        return 1
    finally:
        try:
            context.__exit__(None, None, None)
        finally:
            shutdown = launch.stop(timeout=20.0)
        result["shutdown"] = shutdown
        write_json(results_dir / "loop_soak_summary.json", result)
        (results_dir / "loop_soak_report.md").write_text(
            "# 50k-cycle SimMachine soak\n\n" +
            "\n".join(f"- {key}: {value}" for key, value in result.items()) +
            "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--minimum-cycles", type=int, default=50000)
    parser.add_argument("--results-dir")
    args = parser.parse_args()
    results_dir = (pathlib.Path(args.results_dir) if args.results_dir
                   else make_results_dir())
    return run(args.robot_ip, args.domain_id, results_dir,
               args.minimum_cycles)


if __name__ == "__main__":
    raise SystemExit(main())
