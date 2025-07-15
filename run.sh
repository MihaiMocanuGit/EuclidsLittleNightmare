#!/bin/bash
# unofficial bash strict mode
set -euo pipefail
IFS=$'\n\t'

project="EuclidsLittleNightmare"

build_type="release"
build_only=false
job_count="12"

Help()
{
   echo "Builds and runs the project."
   echo
   echo "Syntax: run [-h|b|B|j]"
   echo "options:"
   echo "h     Print this Help."
   echo "b     Sets the build type (debug/release/relwithdebinfo/minsizerel).
         DEFAULT: release."
   echo "B     Build only, do not run"
   echo "j     Sets the thread job count.
         DEFAULT: 12."
   echo
}

while getopts ":hb:Bj:" option; do
   case $option in
      h) # help
         Help
         exit;;
      b) # build type
         build_type=$OPTARG;;
      B) # build only
         build_only=true;;
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

if [ "$build_only" = false ]
then
    echo "Running $build_dir/$project" && "$build_dir/$project"
fi
