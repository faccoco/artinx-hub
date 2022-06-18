# Procedure to set up boot service 1. Write a shell script ```shell
cd */artinx-hub/cmake-build-release/src
# rember to use absolute path as *
./ArtinxHub ../../config/angleSolver.conf
```
2. Create ArtinxHub.service at `/lib/systemd/sysetm` and write these
```
[Unit]
Description=ArtinxHub-service
[Service]
ExecStart=bash /*the shell script you have written*/
Restart=always
RestartSec=0
KillMode=none
[Install]
WantedBy=multi-user.target
Alias=ArtinxHub_autostart_service
```
3. use`systemctl enable ArtinxHub.serivec` in bash
---
如果串口寄了(permision denied)  
- ~~use `groups ${USER}` to check groups~~
- sudo gpasswd --add ${USER} dialout

