#!/usr/bin/env python3
"""Inventory experiment artifacts without modifying them; emit JSON to stdout.

Missing static references do not establish that a file is disposable. This
script has no delete, move, untrack, or robot-control operation.
"""

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys


ARTIFACT_ROOTS = ("docs/ode_debug", "test_results")
MENTION = re.compile(r"(?:docs/ode_debug|test_results)/[\w./+{},*=-]+")
SOURCE_SUFFIXES = {".py", ".cpp", ".hpp", ".h", ".sh", ".urdf", ".sdf",
                   ".world", ".xacro", ".yaml", ".yml", ".xml"}
MEASUREMENT_SUFFIXES = {".csv", ".jsonl", ".log", ".bag", ".db3", ".mcap"}
BUILD_PARTS = {"build", "CMakeFiles", "__pycache__", ".pytest_cache"}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def classification(path):
    """Conservative content-role labels, never deletion authorization."""
    if path.suffix == ".md":
        return "keep_documentation"
    if path.name in {"manifest.json", "summary.json", "result.json",
                     "measured_result.json"}:
        return "keep_metadata"
    if path.suffix in SOURCE_SUFFIXES or path.name == "CMakeLists.txt":
        return "keep_source_or_config_snapshot"
    if path.suffix in MEASUREMENT_SUFFIXES or path.suffix == ".json":
        return "keep_measurements_or_metadata"
    if path.suffix in {".png", ".svg", ".pdf", ".zip", ".gz", ".stl", ".dae"}:
        return "keep_evidence_or_asset"
    if (set(path.parts) & BUILD_PARTS or
            path.suffix in {".o", ".pyc", ".pyo"}):
        return "review_generated_cache_keep_until_verified"
    return "keep_unknown"


def inspect(root, hash_tracked_raw=False):
    tracked = set(subprocess.check_output(
        ["git", "ls-files", "-z"], cwd=root).decode().split("\0")) - {""}
    refs = defaultdict(set)
    for name in sorted(tracked):
        if any(name.startswith(prefix + "/") for prefix in ARTIFACT_ROOTS):
            continue
        path = root / name
        if path.is_symlink() or not path.is_file() or path.stat().st_size > 2_000_000:
            continue
        raw = path.read_bytes()
        if b"\0" in raw:
            continue
        for mention in MENTION.findall(raw.decode("utf-8", errors="replace")):
            refs[mention.rstrip(".,/")].add(name)

    records = []
    totals = {}
    runs = defaultdict(lambda: {"files": 0, "bytes": 0, "tracked_files": 0})
    roles = defaultdict(lambda: {"files": 0, "bytes": 0})
    errors = []
    for prefix in ARTIFACT_ROOTS:
        total = {"files": 0, "bytes": 0, "allocated_bytes": 0, "tracked_files": 0,
                 "tracked_bytes": 0, "symlinks": 0}
        directory = root / prefix
        if not directory.exists():
            errors.append(f"Missing directory: {prefix}")
        for parent, dirs, files in os.walk(directory, followlinks=False,
                                            onerror=lambda e: errors.append(str(e))):
            # Record symlinks, but never follow them outside the audited roots.
            entries = sorted(files + [d for d in dirs if (Path(parent) / d).is_symlink()])
            dirs[:] = sorted(d for d in dirs if not (Path(parent) / d).is_symlink())
            for name in entries:
                path = Path(parent) / name
                relative = path.relative_to(root)
                key = relative.as_posix()
                try:
                    stat = path.lstat()
                    if path.is_symlink():
                        total["symlinks"] += 1
                        records.append({"path": key, "symlink": os.readlink(path),
                                        "role": "keep_symlink", "tracked": key in tracked})
                        continue
                    if not path.is_file():
                        errors.append(f"Non-regular file: {key}")
                        continue
                    role = classification(relative)
                    cited_by = set()
                    for ancestor in (relative, *relative.parents):
                        cited_by.update(refs.get(ancestor.as_posix(), ()))
                    item = {"path": key, "bytes": stat.st_size,
                            "allocated_bytes": stat.st_blocks * 512,
                            "mtime_ns": stat.st_mtime_ns, "tracked": key in tracked,
                            "role": role, "static_citations": sorted(cited_by),
                            "decision": "keep"}
                    if hash_tracked_raw and item["tracked"] and path.suffix == ".jsonl":
                        item["sha256"] = sha256(path)
                        after = path.stat()
                        if (stat.st_size, stat.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                            raise RuntimeError(f"Changed during hashing: {key}")
                    records.append(item)
                    total["files"] += 1
                    total["bytes"] += stat.st_size
                    total["allocated_bytes"] += stat.st_blocks * 512
                    total["tracked_files"] += int(item["tracked"])
                    total["tracked_bytes"] += stat.st_size if item["tracked"] else 0
                    run_parts = relative.parts[:len(Path(prefix).parts) + 1]
                    run = runs["/".join(run_parts)]
                    run["files"] += 1
                    run["bytes"] += stat.st_size
                    run["tracked_files"] += int(item["tracked"])
                    roles[role]["files"] += 1
                    roles[role]["bytes"] += stat.st_size
                except (OSError, RuntimeError) as exc:
                    errors.append(f"{key}: {exc}")
        totals[prefix] = total
    return {"generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "git_head": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
            "policy": "Keep unknown or unique evidence. Static mentions are incomplete. No deletion approved by this inventory.",
            "totals": totals, "roles": dict(roles), "runs": dict(sorted(runs.items())),
            "files": records, "errors": errors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--hash-tracked-jsonl", action="store_true",
                        help="also SHA-256 the raw JSONL files currently tracked by Git")
    args = parser.parse_args()
    result = inspect(args.root.resolve(), args.hash_tracked_jsonl)
    json.dump(result, sys.stdout, ensure_ascii=False, indent=2)
    sys.stdout.write("\n")
    return 1 if result["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
