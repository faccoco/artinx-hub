#!/bin/bash

sudo systemctl stop ArtinxHub.service
sudo kill $(pidof ArtinxHub)
sudo rm -rf /opt/artinx-hub
sudo cp -r ./ /opt/artinx-hub
sudo cp -f ./bringup_templates/ArtinxHub.service /lib/systemd/system/ArtinxHub.service
sudo echo $1 > /opt/deploy_target.conf
sudo systemctl enable ArtinxHub.service
sleep 5
sudo systemctl status ArtinxHub.service
