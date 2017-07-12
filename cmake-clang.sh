#!/bin/sh

CC=$(which clang)
CXX=$(which clang++)
CC=${CC} CXX=${CXX} cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_MYSTUFF=1 ..
