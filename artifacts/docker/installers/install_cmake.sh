#!/bin/bash
 set -ex

 wget https://github.com/Kitware/CMake/releases/download/v3.26.4/cmake-3.26.4-linux-x86_64.sh
 chmod +x cmake-3.26.4-linux-x86_64.sh
 mkdir /opt/cmake
 ./cmake-3.26.4-linux-x86_64.sh --prefix=/opt/cmake --skip-license
 rm cmake-3.26.4-linux-x86_64.sh
 