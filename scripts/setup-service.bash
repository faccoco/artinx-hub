#!/bin/bash

systemctl stop ArtinxHub.service
kill $(pidof ArtinxHub)
rm -rf /opt/artinx-hub
cp -r ./ /opt/artinx-hub
cp -f ./bringup_templates/ArtinxHub.service /lib/systemd/system/ArtinxHub.service
echo $1 > /opt/deploy_target.conf
systemctl enable ArtinxHub.service
sleep 5
systemctl status ArtinxHub.service
