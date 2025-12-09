#!/bin/bash

# fail on error, undefined variable, or error in a pipeline
set -euo pipefail

# set fail message
trap 'echo "An error occurred. Exiting..." >&2' ERR

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo "Build completed successfully."
exit 0