#!/bin/bash
sleep 10
/usr/bin/jetson_clocks --fan &
python3 /etc/rc.local/main.py 2> /home/artinx/Desktop/log.txt &
