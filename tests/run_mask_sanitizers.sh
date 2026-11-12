#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
dependency_prefix="${NGD_DEPENDENCY_PREFIX:-$(dirname "$repo_dir")/dependencies/install}"
test_dir="$(mktemp -d /tmp/ngd-mask-test.XXXXXX)"
trap 'rm -rf "$test_dir"' EXIT
# Instrument the exact production function without rebuilding the SLAM dependencies.
python3 - "$repo_dir/src/Tracking.cc" "$test_dir/check.cc" <<'PY'
import sys
from pathlib import Path
s=Path(sys.argv[1]).read_text()
a=s.index('void Tracking::ComputePixelPotential(')
b=s.index('\n\nvoid Tracking::ExtractDynaPoints',a)
body=s[a:b].replace('Tracking::ComputePixelPotential','ComputePixelPotential',1)
main='''
int main() {
    cv::Mat image=cv::Mat::ones(256,256,CV_8U);
    for(int y=0;y<256;++y) for(int x=0;x<256;++x) {
        int score;
        ComputePixelPotential(score,cv::Point2f(x,y),image);
        if(x<3 || y<3 || x>=253 || y>=253) assert(score==-1);
    }
    std::cout << "PASS: ASan/UBSan image-wide dynamic-pixel border sweep\\n";
}
'''
Path(sys.argv[2]).write_text('#include <opencv2/core.hpp>\n#include <vector>\n#include <algorithm>\n#include <cmath>\n#include <cassert>\n#include <iostream>\n'+body+main)
PY
c++ -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer "$test_dir/check.cc" \
  -I"$dependency_prefix/include/opencv4" -L"$dependency_prefix/lib" \
  -Wl,-rpath,"$dependency_prefix/lib" -lopencv_core -o "$test_dir/check"
ASAN_OPTIONS=detect_leaks=0 "$test_dir/check"
