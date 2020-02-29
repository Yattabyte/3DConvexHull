#!/usr/bin/env bash

# Run CPPCheck
echo "
**************************************************
Starting CPPCheck
**************************************************"
cppcheck src tests -iexternal/ -I src/ -I tests/ --enable=all --quiet --suppress=missingIncludeSystem || exit 1

# Run Clang-Tidy using cmake
echo "
**************************************************
Starting Clang-Tidy
**************************************************"
cmake -DBUILD_TESTING=ON -DCODE_COVERAGE=OFF -DSTATIC_ANALYSIS=OFF -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_CLANG_TIDY=clang-tidy-9 . || exit 1
cmake --build . --clean-first -- -j $(nproc) || exit 1
ctest --output-on-failure --quiet -j $(nproc) -C Debug . || exit 1

# Run Valgrind
echo "
**************************************************
Starting Memory Sanitizer (valgrind)
**************************************************"
cmake -DSTATIC_ANALYSIS=ON -UCMAKE_CXX_CLANG_TIDY . || exit 1
cmake --clean-first . || exit 1
ctest --output-on-failure --quiet -j $(nproc) -C Debug -D ExperimentalBuild . || exit 1
ctest --output-on-failure --quiet -j $(nproc) -C Debug -D ExperimentalTest . || exit 1
ctest --output-on-failure --quiet -j $(nproc) -C Debug -D ExperimentalMemCheck . || exit 1
echo "Dumping Logs"
echo "$(<${TRAVIS_BUILD_DIR}/Testing/Temporary/MemoryChecker.1.log)"

# Address Sanitizers
echo "
**************************************************
Starting Address Sanitizer
**************************************************"
cmake -DSTATIC_ANALYSIS=OFF -DCMAKE_CXX_FLAGS="-g -O0 -fno-omit-frame-pointer -fno-optimize-sibling-calls -fsanitize=address" -DCMAKE_EXE_LINKER_FLAGS="-g -O0 -fno-omit-frame-pointer -fno-optimize-sibling-calls -fsanitize=address" . || exit 1
cmake --build . --clean-first -- -j $(nproc) || exit 1
ctest --output-on-failure --quiet -j $(nproc) -C Debug . || exit 1

# Undefined Behaviour Sanitizer
echo "
**************************************************
Starting Undefined-Behaviour Sanitizer
**************************************************"
cmake -DCMAKE_CXX_FLAGS="-g -O0 -fno-omit-frame-pointer -fno-optimize-sibling-calls -fsanitize=undefined" -DCMAKE_EXE_LINKER_FLAGS="-g -O0 -fno-omit-frame-pointer -fno-optimize-sibling-calls -fsanitize=undefined" . || exit 1
cmake --build . --clean-first -- -j $(nproc) || exit 1
ctest --output-on-failure --quiet -j $(nproc) -C Debug . || exit 1

# Thread Sanitizer
echo "
**************************************************
Starting Thread Sanitizer
**************************************************"
cmake -DCMAKE_CXX_FLAGS="-g -O0 -fno-omit-frame-pointer -fno-optimize-sibling-calls -fsanitize=thread" -DCMAKE_EXE_LINKER_FLAGS="-g -O0 -fno-omit-frame-pointer -fno-optimize-sibling-calls -fsanitize=thread" . || exit 1
cmake --build . --clean-first -- -j $(nproc) || exit 1
ctest --output-on-failure --quiet -j $(nproc) -C Debug . || exit 1

# OCLint quality reporting
echo "
**************************************************
Starting OCLint
**************************************************"
cmake -DCMAKE_CXX_FLAGS="-g -O0" -DCMAKE_EXE_LINKER_FLAGS="-g -O0" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON . || exit 1
cmake --build . --clean-first -- -j $(nproc) || exit 1
oclint-json-compilation-database -i src -i tests -e external