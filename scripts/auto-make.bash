#!/bin/bash

export DAHENG_ROOT="/home/artinx-002/workspace/package/Galaxy_U2"
source /opt/intel/openvino_2021/bin/setupvars.sh
cmake -B ./build -DARTINX_HUB_CAMERA=USB2 -DCMAKE_MAKE_PROGRAM=make -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build ./build --config Release -j 6
sudo touch /opt/env_setup.sh
sudo bash ./scripts/setup-service.bash sentry
