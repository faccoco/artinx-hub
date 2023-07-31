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
if [ ! -d "/opt/artinx_log" ]; then
  mkdir /opt/artinx_log
fi
while [ true ]; do
  LOG_FILE=/opt/artinx_log/`date +"%m-%d-%k-%M-%S"`.log
  /opt/artinx-hub/build/src/ArtinxHub /opt/artinx-hub/deploy_config/$(cat /opt/deploy_target.conf).conf 2>$LOG_FILE
  sleep 1
done

