#!/usr/bin/env bash
set -euo pipefail
FR10_BENCH_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cmake -S "$FR10_BENCH_DIR" -B "$FR10_BENCH_DIR/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
cmake --build "$FR10_BENCH_DIR/build" --parallel 2
ctest --test-dir "$FR10_BENCH_DIR/build" --output-on-failure
