#!/bin/bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/../../.." && pwd)"
bundled_opencv_lib="$workspace_dir/third_party/opencv-3.4/lib"

export LD_LIBRARY_PATH="$bundled_opencv_lib:/usr/local/lib:/home/developer/opencv-3.4.16/build/lib:/home/bob/opencv-3.4.16/build/lib:${LD_LIBRARY_PATH:-}"
exec "$@"
