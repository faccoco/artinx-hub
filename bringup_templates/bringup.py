# configs
build_path = "cmake-build-release-default"
config_path = "config/deploy/general.conf"
exe_path = "/src/ArtinxHub"
# main
import os
import subprocess

parent_directory = os.path.dirname(os.path.abspath(__file__)) + "/.."
os.chdir(parent_directory)

# subprocess.Popen('. /opt/intel/openvino_2021/bin/setupvars.sh', shell=True)
# subprocess.Popen('gnome-terminal -e "bash /home/artinx-004/workspace/codes/artinx-hub/bringup_templates/bringup.bash"', shell=True)

a = subprocess.Popen(build_path+exe_path+' '+config_path, shell=True, stdout=io.
                     )
while True:
    print(a.)
