# Clion Remote Debug Setting
## 1. Install ssh server
```shell
sudo apt update
sudo apt install openssh-server -y
sudo systemctl status ssh #查看状态
# 如果你的防火墙开启了，使用下面语句
sudo ufw allow ssh
```
## 2. Clion config
*Reference*: https://blog.csdn.net/wads23456/article/details/116924722
