# Remote Debug Setting
## 1. Install ssh server
```shell
sudo apt update
sudo apt install openssh-server -y
sudo systemctl status ssh #查看状态
# 如果你的防火墙开启了，使用下面语句
sudo ufw allow ssh
```
## 2. Window vscode 
(1)vscode 安装一下插件:

+ Remote-SSH
+ Remote Development
+ REST Client

(2) 打开Remote-ssh

```shell
ssh xxx.xxx.xxx.xxx@username
```
## 3、httpServer visualization
打开浏览器输入`xxx.xxx.xxx.xxx:5630/pages/index.html`,注意远程调试主机和机器人NUC连同一个wifi, `xxx.xxx.xxx.xxx`为机器人NUC的IP地址