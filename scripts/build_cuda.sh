#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
"${CUDA_HOME:-/usr/local/cuda}/bin/nvcc" -O3 --fmad=false -std=c++17 -arch=sm_87 -shared -Xcompiler=-fPIC \
  "$root/src/r1pro_waypoint_nav/native/crop_cuda.cu" \
  -o "$root/src/r1pro_waypoint_nav/r1pro_waypoint_nav/libcrop_cuda.so"
