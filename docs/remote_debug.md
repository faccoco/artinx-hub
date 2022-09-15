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

(2) 打开Remote-ssh

```shell
ssh username@xxx.xxx.xxx.xxx
```
## 4.build and run
(1) vscode远程ssh连接后，可以直接在vscode里编辑远程主机代码

(2) 在vscode终端`artinx_hub`目录下运行以下命令，即可编译并运行程序
```shell
sudo bash ./scripts/auto-make.bash    
```
## 3. httpServer visualization
打开浏览器输入`xxx.xxx.xxx.xxx:5630/pages/index.html`,注意远程调试主机和机器人NUC连同一个wifi, `xxx.xxx.xxx.xxx`为机器人NUC的IP地址。
