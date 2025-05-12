#!/bin/bash
 set -ex

 pushd /opt
 git clone https://github.com/microsoft/vcpkg.git
 pushd vcpkg
 bash bootstrap-vcpkg.sh
 popd
 popd
 