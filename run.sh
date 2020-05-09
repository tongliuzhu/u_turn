#!/bin/bash
# Go into the directory where this bash script is contained.

cd `dirname $0`

echo -e "\e[33m clang format chechking!...\e[0m"
source clang-format.sh $(pwd)/src
source clang-format.sh $(pwd)/include
echo -e "\e[33m clang format done!...\e[0m"

mkdir -p build
cd build
cmake ..
make -j && ./hybrid_astar
