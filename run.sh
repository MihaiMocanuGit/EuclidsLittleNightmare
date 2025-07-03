#!/bin/bash
# unofficial bash strict mode
set -euo pipefail
IFS=$'\n\t'

if [ $# -eq 0 ]
then 
	build_type=""
else
	build_type="$1"
fi

if [ "$build_type" != "debug" ]
then
	build_type="release"
fi

echo "build_type = $build_type"

build_dir="$(dirname $0)/build_$build_type"
echo "build_dir = $build_dir"

echo "Setting build options"
time (cmake "-DCMAKE_POLICY_VERSION_MINIMUM=3.5" -DCMAKE_BUILD_TYPE="$build_type" -S . -B "$build_dir" && echo "Building $build_type" && cmake --build "$build_dir" -j 16) &&

echo "Running $build_dir/EuclidsNightmare" &&
"$build_dir/EuclidsNightmare"
