#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
dependency_prefix="${NGD_DEPENDENCY_PREFIX:-$(dirname "$repo_dir")/dependencies/install}"
test_dir="$(mktemp -d /tmp/ngd-tracking-test.XXXXXX)"
trap 'rm -rf "$test_dir"' EXIT
c++ -std=c++17 -march=native -DHAVE_GLEW "$repo_dir/tests/tracking_safety_test.cc" \
  -I"$repo_dir/include" -I"$repo_dir/include/CameraModels" -I"$repo_dir" \
  -I"$repo_dir/Thirdparty/Sophus" -I/usr/include/eigen3 \
  -I"$dependency_prefix/include" -I"$dependency_prefix/include/opencv4" \
  -L"$repo_dir/lib" -L"$dependency_prefix/lib" \
  -L"$repo_dir/Thirdparty/DBoW2/lib" -L"$repo_dir/Thirdparty/g2o/lib" \
  -lORB_SLAM3 -lDBoW2 -lg2o -lopencv_core -lopencv_imgproc -lopencv_dnn \
  "$dependency_prefix"/lib/libpango_*.so -lGLEW -lGL -pthread -o "$test_dir/check"
export LD_LIBRARY_PATH="$repo_dir/lib:$dependency_prefix/lib:$repo_dir/Thirdparty/DBoW2/lib:$repo_dir/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"
cd "$test_dir"
./check "$repo_dir"
