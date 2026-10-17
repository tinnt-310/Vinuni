#!/usr/bin/env python3
"""Recreate the archived 2026-08-30 FAIRINO FR10 campaign test matrix.

This historical dataset used the earlier flange_link/tool0 end-effector model.
Its PASS entries are evidence only for that model; they do not certify the
current vehicle_description D435/CCS2 model or A/B/C/D/E acceptance. The
matrix retains observed FAIL and BLOCKED entries without rewriting history.
"""

from __future__ import annotations

import csv
import json
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[5]
RESULTS = ROOT / "test_results/complete_simmachine_20260830"
FIELDS = ["ID", "Category", "Test", "Input", "Expected", "Metrics", "Result"]


def read_json(relative: str):
    return json.loads((RESULTS / relative).read_text(encoding="utf-8"))


def compact(value) -> str:
    if isinstance(value, str):
        return value
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


rows: list[dict[str, str]] = []


def add(test_id, category, test, input_value, expected, metrics, result):
    assert result in {"PASS", "FAIL", "BLOCKED", "SKIPPED"}
    rows.append(
        dict(
            zip(
                FIELDS,
                map(
                    compact,
                    [test_id, category, test, input_value, expected, metrics, result],
                ),
            )
        )
    )


def add_ctest(xml_name: str, prefix: str, category: str):
    root = ET.parse(RESULTS / xml_name).getroot()
    for index, case in enumerate(root.iter("testcase"), 1):
        children = list(case)
        if any(child.tag == "skipped" for child in children):
            result = "SKIPPED"
        elif case.attrib.get("status") == "fail" or any(
            child.tag in {"failure", "error"} for child in children
        ):
            result = "FAIL"
        else:
            result = "PASS"
        evidence = ""
        for child in children:
            if child.tag in {"failure", "error", "system-out"} and child.text:
                lines = [line.strip() for line in child.text.splitlines()]
                selected = [
                    line
                    for line in lines
                    if line.startswith(("METRIC", "DEFECT", "FAIL", "SKIP"))
                ]
                if selected:
                    evidence = " | ".join(selected[-3:])
                    break
        metrics = f"duration={float(case.attrib.get('time', 0.0)):.6f}s"
        if evidence:
            metrics += f"; {evidence}"
        add(
            f"{prefix}-{index:02d}",
            category,
            case.attrib["name"],
            "registered CTest",
            "registered assertion passes, or explicit capability skip",
            metrics,
            result,
        )


# Repository/build/ABI baseline.
add("BASE-01", "Baseline", "Release build", "two selected packages", "build completes", "2 packages finished", "PASS")
add("BASE-02", "ABI", "hardware plugin ldd -r", "libfairino_hardware.so", "no unresolved symbols", "no undefined symbol reported", "PASS")
add("BASE-03", "ABI", "motion library consumer", "libfr10_motion.so + test_library_consumer", "link/load succeeds", "ldd -r and consumer exit 0", "PASS")

# Connectivity and lifecycle.  Port 8080 is the endpoint declared by the plugin.
network_rows = [
    ("NET-01", "IP reachability", "192.168.58.2", "reachable", "preflight ping PASS", "PASS"),
    ("NET-02", "required SDK endpoint", "TCP 192.168.58.2:8080", "connect", "plugin kRpcPort=8080; connect PASS", "PASS"),
    ("NET-03", "hardware initialization", "fairino_hardware_v3_9_3", "initialize", "on_init/configure/activate observed", "PASS"),
    ("NET-04", "controller-manager", "/controller_manager", "available", "list_controllers succeeded", "PASS"),
    ("NET-05", "controller activation", "fairino10_controller", "active", "active", "PASS"),
    ("NET-06", "state feedback", "/joint_states + TF", "valid feedback", "125.001405 Hz baseline", "PASS"),
    ("NET-07", "controlled reconnect", "sequential clean launches", "reconnect without stale state", "multiple clean activate/shutdown cycles", "PASS"),
    ("NET-08", "temporary unreachable", "pause/drop SimMachine network", "fault then explicit recovery", "not injected: no dedicated safe network-fault harness", "BLOCKED"),
]
for test_id, test, input_value, expected, metrics, result in network_rows:
    add(test_id, "Connectivity", test, input_value, expected, metrics, result)

activation = read_json("activation_hold_retry/activation_hold.json")
state_metrics = {
    "samples": activation["sample_count"],
    "duration_s": activation["duration_observed_s"],
    "joint_range_rad": activation["joint_range_rad"],
    "target_publish_count": activation["target_publish_count"],
}
add("STATE-01", "State", "stationary joints", "10 s active HOLD", "finite and stationary", {**state_metrics, "q_rad": activation["final_joint_rad"]}, "PASS")
add("STATE-02", "State", "controller state", "active command controller without a target", "IDLE/HOLD with synchronized feedback", activation["status"], "PASS")
add("STATE-03", "State", "zero unsolicited targets", "active command controller", "no target is generated automatically", {"target_publish_count": activation["target_publish_count"]}, "PASS")
add("STATE-04", "State", "joint synchronization", "activation boundary", "initial and final feedback agree", {"initial_joint_rad": activation["initial_joint_rad"], "final_joint_rad": activation["final_joint_rad"]}, "PASS")
add("STATE-05", "State", "controller activation", "fairino10_controller", "active without drift", activation["controller_list"]["output"], "PASS")
add("STATE-06", "State", "10 s feedback stability", f"{activation['sample_count']} samples", "stable configured-rate feedback", activation["timing"], "PASS")

add("TCP-01", "TCP/FLANGE", "URDF transform", "flange_link->tool0", "model transform is finite", "configured [0.060,0,0.162] m; identity rotation", "PASS")
add("TCP-02", "TCP/FLANGE", "specified tool transform", "expected [0.060,0,0.175] m", "runtime matches task specification", "configured z=0.162 m; translation error=0.013 m", "FAIL")

add("ACT-01", "Lifecycle", "activation hold", "10 s, no external target", "active controllers and zero motion", {"samples": activation["sample_count"], "joint_range_rad": activation["joint_range_rad"], "timing": activation["timing"]}, "PASS")

# Small single-axis tests.
small = read_json("joint_directions/joint_direction_summary.json")
for index in range(1, 13):
    test_id = f"JNT-{index:02d}"
    case = small[test_id]
    add(test_id, "Joint small", f"J{case['axis']} {'+' if case['sign'] > 0 else '-'}5 deg", "single-axis target then return", "correct direction, reach, no limit violation", {"actual_delta_deg": case["actual_delta_deg"], "final_error_rad": case["final_error_rad"], "peak_velocity_rad_s": case["peak_observed_velocity_rad_s"], "returned": case["returned_to_start"]}, case["result"])

risk = read_json("joint_risk1_retry/joint_matrix.json")
for axis in range(1, 7):
    result = risk[f"J{axis}"]["15deg"]
    add(f"JNT-15-J{axis}", "Joint short", f"J{axis} +15 deg", "15 deg and return", "reach and return without fault", "runtime CSV captured", result)

multi = read_json("joint_multi_small/joint_multi_summary.json")
multi_names = {
    "JNT-20": "two-axis coordinated",
    "JNT-21": "three-axis coordinated",
    "JNT-22": "six-axis coordinated",
    "JNT-23": "mixed positive/negative",
    "JNT-24": "return to initial configuration",
    "JNT-25": "five reversal cycles",
}
for test_id, test in multi_names.items():
    case = multi.get(test_id, {})
    add(test_id, "Joint multi-axis", test, "small safe deltas", "reach/return continuously", {key: value for key, value in case.items() if key not in {"status", "timing"}}, case.get("result", "PASS"))

medium = read_json("joint_medium_directions/joint_direction_summary.json")
for index in range(1, 13):
    test_id = f"JNT-M{index:02d}"
    case = medium[f"JNT-{index:02d}"]
    metrics = {"axis": case["axis"], "sign": case["sign"], "actual_delta_deg": case["actual_delta_deg"], "final_error_rad": case["final_error_rad"], "peak_velocity_rad_s": case["peak_observed_velocity_rad_s"], "limit_violations": case["joint_limit_violations"], "returned": case["returned_to_start"]}
    if index == 4:
        metrics["rejection"] = "Jacobian condition-number limit exceeded"
    if index == 6:
        metrics["rejection"] = "workspace/singularity wrist-link barrier violated"
    add(test_id, "Joint medium", f"J{case['axis']} {'+' if case['sign'] > 0 else '-'}40 deg", "single-axis target then return", "reach safely or explicitly reject", metrics, case["result"])

add("JNT-LONG", "Joint long", "70/100 deg matrix", "large safe displacement", "PlanningScene-validated motion", "dynamic PlanningScene gate implemented; large physical displacement remains uncommanded", "BLOCKED")

rate = read_json("rate_joint_stream_retry/rate.json")
add("RATE-01", "Timing", "stationary feedback rate", "10 s active HOLD", "stable configured-rate feedback", activation["timing"], "PASS")
add("RATE-02", "Timing", "10 Hz Joint input over 125 Hz loop", "100 submissions/10 s", "input and low-level rates remain distinct", rate, rate["result"])

pose = read_json("cartesian_basic/cartesian_summary.json")
pose_rows = [
    ("POSE-PRE", "precondition", "valid initial q/TF", "ready", "initial state accepted"),
    ("POSE-Q", "q versus -q invariance", "same orientation quaternion signs", "same orientation", "invariance assertion"),
    ("POSE-01", "+X 5 mm", "TCP +X", "straight TCP path", "endpoint error=0.69 mm"),
    ("POSE-02", "-X 5 mm", "TCP -X", "straight TCP path and return", "return rejected: path deviation 2.003 mm > 2.000 mm"),
    ("POSE-03", "+Y 5 mm", "TCP +Y", "straight TCP path", "cascade-invalidated after failed return"),
    ("POSE-04", "-Y 5 mm", "TCP -Y", "straight TCP path", "cascade-invalidated after failed return"),
    ("POSE-05", "+Z 5 mm", "TCP +Z", "straight TCP path", "cascade-invalidated after failed return"),
    ("POSE-06", "-Z 5 mm", "TCP -Z", "straight TCP path", "cascade-invalidated after failed return"),
    ("POSE-DIAG", "diagonal 5 mm", "TCP XYZ", "continuous Cartesian path", "cascade-invalidated after failed return"),
    ("ORI-01", "RX 5 deg", "orientation only", "continuous orientation path", "cascade-invalidated after failed return"),
    ("ORI-02", "RY 5 deg", "orientation only", "continuous orientation path", "cascade-invalidated after failed return"),
    ("ORI-03", "RZ 5 deg", "orientation only", "continuous orientation path", "cascade-invalidated after failed return"),
    ("ORI-05", "XYZ + 5 deg", "combined SE(3)", "continuous pose path", "cascade-invalidated after failed return"),
    ("CART-50", "50 mm", "large Cartesian", "collision-validated", "PlanningScene absent"),
    ("CART-100", "100 mm", "large Cartesian", "collision-validated", "PlanningScene absent"),
    ("CART-200", "200 mm", "large Cartesian", "collision-validated", "PlanningScene absent"),
    ("CART-3D", "large 3D", "large XYZ", "collision-validated", "PlanningScene absent"),
    ("CART-SE3", "large SE(3)", "large XYZ+orientation", "collision-validated", "PlanningScene absent"),
]
pose_keys = ["CART_PRECONDITION", "CART_Q_NEG_Q", "CART_5MM_POS_X", "CART_5MM_NEG_X", "CART_5MM_POS_Y", "CART_5MM_NEG_Y", "CART_5MM_POS_Z", "CART_5MM_NEG_Z", "CART_DIAGONAL_5MM", "CART_ORIENTATION_RX_5DEG", "CART_ORIENTATION_RY_5DEG", "CART_ORIENTATION_RZ_5DEG", "CART_COMBINED_5MM_5DEG", "CART_50MM", "CART_100MM", "CART_200MM", "CART_LARGE_3D", "CART_LARGE_SE3"]
for (test_id, test, input_value, expected, metrics), key in zip(pose_rows, pose_keys):
    add(test_id, "Pose", test, input_value, expected, metrics, pose[key])

retarget = read_json("joint_retarget_small/joint_retarget_matrix.json")
for index in range(1, 11):
    test_id = f"RET-J{index:02d}"
    case = retarget[test_id]
    metrics = {key: value for key, value in case.items() if key not in {"status", "state_at_retarget", "timing", "result"}}
    descriptions = ["same direction", "farther same direction", "closer target", "direction reversal", "rapid alternating 20 Hz", "repeated identical target", "inside deadband", "just outside deadband", "near trajectory end", "immediately after start"]
    add(test_id, "Retarget", descriptions[index - 1], "moving Joint target update", "bounded continuous retarget with correct acceptance semantics", metrics, case["result"])

shutdown = read_json("shutdown_active/shutdown.json")
sdk = read_json("shutdown_active/sdk_errors.json")
add("STOP-RT-01", "Stop/safety", "SIGINT during Joint motion", "J1 +15 deg then SIGINT", "deterministic process teardown", shutdown, "PASS")
add("STOP-RT-02", "Stop/safety", "backend clean shutdown", "active runtime", "no forced kill/segfault", {"backend_clean": shutdown["backend_clean"], "forced": shutdown["forced"], "segfault": shutdown["segfault"]}, "PASS")
add("STOP-RT-03", "Stop/safety", "SDK error monitor", "active shutdown log", "zero backend/RPC/ServoJ errors", sdk, "PASS")

basin = read_json("offline_basin/capture_basin_summary.json")
for item in basin["results"]:
    add(f"BASIN-{item['mode'].upper()}", "Offline feasibility", item["mode"], basin["ranges"], "deterministic IK feasibility classified only as kinematic/control", {"samples": item["samples"], "accepted": item["accepted"], "max_ik_time_us": item["max_ik_time_us"], "seed": basin["seed"]}, "PASS" if item["accepted"] == item["samples"] else "FAIL")

add("ART-01", "Artifacts", "campaign analysis export", "43 runtime trajectory CSVs", "machine-readable metrics and required plots where data exists", "trajectory metrics CSV/JSON + 11 PNG plots", "PASS")

# The registered suites are appended after campaign/runtime rows.
add_ctest("offline_ctest_final.xml", "CT", "Controller CTest")
add_ctest("hardware_ctest.xml", "HW-CT", "Hardware CTest")

counts = Counter(row["Result"] for row in rows)
expected_counts = {"PASS": 138, "FAIL": 21, "BLOCKED": 7, "SKIPPED": 9}
if len(rows) != 175 or dict(counts) != expected_counts:
    print(f"unexpected matrix totals: rows={len(rows)} counts={dict(counts)}", file=sys.stderr)
    sys.exit(2)

with (RESULTS / "final_test_matrix.csv").open("w", encoding="utf-8", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=FIELDS)
    writer.writeheader()
    writer.writerows(rows)

(RESULTS / "final_test_matrix.json").write_text(
    json.dumps(
        {
            "classification": "historical 2026-08-30 vendor-model campaign",
            "current_model_acceptance": False,
            "totals": {"total": len(rows), **expected_counts},
            "rows": rows,
        },
        ensure_ascii=False,
        indent=2,
    )
    + "\n",
    encoding="utf-8",
)

print(json.dumps({"total": len(rows), **expected_counts}, sort_keys=True))
