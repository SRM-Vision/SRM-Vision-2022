#!/usr/bin/env bash
[ ! -d "./build" ] && mkdir -p build
cd build || exit
rm -rf ./* || exit
cmake -G "CodeBlocks - Unix Makefiles" -DCMAKE_BUILD_TYPE=Release .. || exit
make -j"$(cat /proc/cpuinfo | grep "processor" | wc -l)"
