# openvino
sleep 3
source /opt/intel/openvino_2021/bin/setupvars.sh
# cd to the bash script location
str=$0 # the bash script location
path=$(dirname "$str") # the directory of the location
cd "$path" || exit # if fails to cd, exits.
cd ..
#ArtinxHub
while [ true ]; do
  /opt/artinx-hub/build/src/ArtinxHub /opt/artinx-hub/deploy_config/$(cat /opt/deploy_target.conf).conf > /opt/artinx-hub.log
  sleep 1
done
# Guidance:
#  To use this sh script, add the following command to Ubuntu Application Startup:
#   gnome-terminal -- bash [absolute file path to this script]  //deprecated, doesn't work on two nuc.
#   gnome-terminal -e "bash [absolute file path to this script]" // tested to work on two nuc.
#  for example,
#   gnome-terminal -e "bash /home/artinx-004/workspace/codes/artinx-hub/bringup_templates/bringup.bash"
