#!/bin/sh

set -e

if [ -z $1 ]; then
  echo "Please specify clang or gcc e.g. $0 gcc or $0 clang"
  exit -1
fi

c_compiler=clang
cpp_compiler=clang++
if [ "$1" == "clang" ]; then
  c_compiler=gcc
  cpp_compiler=g++
fi

run_cmake_host()
{
  if [ -d output/$1 ]; then
    rm -rf output/$1
  fi
  mkdir -p output/$1
  currentdir=`pwd`
  cd output/$1
  cmake -DCMAKE_BUILD_TYPE=$1 -DCMAKE_C_COMPILER=$2 -DCMAKE_CXX_COMPILER=$3 ../../cpp/
  cd $currentdir
}

run_cmake_host Release $c_compiler $cpp_compiler
run_cmake_host Debug $c_compiler $cpp_compiler
