#!/usr/bin/env bash
set -euo pipefail
s31_root=$(cd -- "$(dirname -- "$0")/../.." && pwd)
s31_output=${1:-"$s31_root/artifacts/native/s31_rx"}
mkdir -p -- "$(dirname -- "$s31_output")"
"${CXX:-g++}" -O3 -std=c++17 -Wall -Wextra -Wpedantic -pthread \
    "$s31_root/host/native/s31_rx.cpp" -o "$s31_output"
printf 'Built %s\n' "$s31_output"
