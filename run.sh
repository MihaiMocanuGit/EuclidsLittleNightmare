#!/bin/bash
# unofficial bash strict mode
set -euo pipefail
IFS=$'\n\t'

project="EuclidsLittleNightmare"

build_type="release"
job_count="12"

Help()
{
   echo "Builds and runs the project."
   echo
   echo "Syntax: run [-h|b|j]"
   echo "options:"
   echo "h     Print this Help."
   echo "b     Sets the build type (debug/release/relwithdebinfo/minsizerel).
         DEFAULT: release."
   echo "j     Sets the thread job count.
         DEFAULT: 12."
   echo
}

while getopts ":hb:j:" option; do
   case $option in
      h) # help
         Help
         exit;;
      b) # build type
         build_type=$OPTARG;;
      j) # job count
         job_count=$OPTARG;;
     \?) # Invalid option
         echo "Error: Invalid option"
         exit;;
   esac
done

if ! [[ "$build_type" =~ ^(debug|relwithdebinfo|minsizerel)$ ]]
then
    build_type="release"
fi

echo "build_type = $build_type"

build_dir="$(dirname $0)/build_$build_type"
echo "build_dir = $build_dir"

echo "Setting build options"
time (cmake "-DCMAKE_POLICY_VERSION_MINIMUM=3.5" \
    -DCMAKE_BUILD_TYPE="$build_type" -S . -B "$build_dir" \
    && echo "Building $build_type" && cmake --build "$build_dir" -j "$job_count") &&

echo "Running $build_dir/$project" &&
"$build_dir/$project"
