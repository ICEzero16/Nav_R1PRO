#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
include="${NANOFLANN_INCLUDE_DIR:-/opt/ros/humble/include/karto_sdk}"
test -f "$include/nanoflann.hpp"
g++ -O3 -std=c++17 -fPIC -shared -ffp-contract=off -I"$include" \
  "$root/src/r1pro_waypoint_nav/native/knn_mean.cpp" \
  -o "$root/src/r1pro_waypoint_nav/r1pro_waypoint_nav/libknn_mean.so"
