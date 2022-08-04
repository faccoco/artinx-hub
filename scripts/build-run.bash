#!/bin/bash

cd ~/workspace/codes/artinx-hub/build
cmake ..
make
./src/ArtinxHub ~/workspace/codes/artinx-hub/deploy_config/fixArmorDetector.conf
