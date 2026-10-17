"""Independent joint/TCP source-stamp evidence for FR10 regression runs."""

from bisect import bisect_left
import json
import math

# JointState is converted through a double in RobotState. At Unix epoch
# seconds this loses at most a fraction of a microsecond; 1 us is a generous
# representation tolerance, not a motion or feedback-freshness budget.
SOURCE_STAMP_MATCH_TOLERANCE_S = 1e-6


def summarize_feedback_sources(joint_rows, tcp_rows):
    joint_sources = sorted(row[1] for row in joint_rows)
    tcp_sources = [row[0] for row in tcp_rows]
    mismatches = []
    for source in tcp_sources:
        index = bisect_left(joint_sources, source)
        candidates = joint_sources[max(0, index - 1):index + 1]
        mismatches.append(min((abs(source - item) for item in candidates),
                              default=math.inf))
    return {
        'joint_feedback_samples': len(joint_rows),
        'tcp_source_samples': len(tcp_rows),
        'tcp_source_monotonic': bool(tcp_sources) and all(
            later > earlier for earlier, later in zip(tcp_sources, tcp_sources[1:])),
        'tcp_source_joint_max_difference_s': max(mismatches, default=None),
        'tcp_source_matches_joint': bool(mismatches) and all(
            error <= SOURCE_STAMP_MATCH_TOLERANCE_S for error in mismatches),
        'joint_max_receive_gap_s': max((later[0] - earlier[0]
            for earlier, later in zip(joint_rows, joint_rows[1:])), default=None),
        'joint_max_source_gap_s': max((later[1] - earlier[1]
            for earlier, later in zip(joint_rows, joint_rows[1:])), default=None),
    }


def write_feedback_trace(probe, directory):
    with probe.lock:
        joints = list(probe.joint_source_history)
        tcp = list(probe.tcp_source_history)
    (directory / 'joint_timing.json').write_text(json.dumps(joints) + '\n')
    (directory / 'tcp_source_timing.json').write_text(json.dumps(tcp) + '\n')
    return summarize_feedback_sources(joints, tcp)
