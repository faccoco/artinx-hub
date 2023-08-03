# openvino
sleep 3
source /opt/intel/openvino_2021/bin/setupvars.sh
source /opt/env_setup.sh

# cd to the bash script location
str=$0 # the bash script location
path=$(dirname "$str") # the directory of the location
cd "$path" || exit # if fails to cd, exits.
cd ..
#ArtinxHub
while [ true ]; do
  /home/artinx-7/Desktop/workspaces/codes/artinx-hub/cmake-build-release/src/ArtinxHub /home/artinx-7/Desktop/workspaces/codes/artinx-hub/deploy_config/radar_recorder.conf 2>/home/artinx-7/Desktop/workspaces/codes/artinx-hub/radar.log
  sleep 1
done
