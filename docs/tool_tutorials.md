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
# docker
## 1. docker 安装
**reference**: https://docs.docker.com/engine/install/ubuntu/

（1）赋予docker管理员权限
```shell
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker
```
## 2. 配置阿里云镜像加速
**reference**: https://www.cnblogs.com/qican/p/15507934.html
## 3. docker常用命令
## docker启动
```shell
systemctl start docker   #启动docker
systemctl enable docker  #设置开机自启动
```

### 镜像命令
```shell
docker images    #查看本机所有镜像 
docker search    #搜索镜像
docker pull      #拉取镜像
docker rmi       #删除镜像

docker commit -m "描述信息" -a "作者" 容器名 目标镜像名:[tag]  #编辑容器后提交容器成为一个新镜像
```
### 容器命令
```shell
docker run [可选参数] [镜像名]          #新建容器并启动        
docker exec [可选参数] [容器名]         #在容器中运行命令
#可选参数说明
--name="Name"   #容器运行时的名字
-d              #后台方式运行
-it             #使用交互方式运行，进入容器查看内容 
-p              #指定主机端口映射到容器端口 （-p ip:主机端口：容器端口 -p 主机端口:容器端口）
-P              #随机制定端口

docker run -it ubuntu /bin/bash    #启动并进入容器，并且运行bash
docker exec -it ubuntu /bin/bash

docker ps [可选参数]   #列出正在运行的容器

#退出容器
exit
#后台运行容器
Ctrl + P + Q
```
### 容器数据卷
为了实现数据持久化，使容器之间可以共享数据。可以将容器内的目录，挂载到宿主机上或其他容器内，实现同步和共享的操作。即使将容器删除，挂载到本地的数据卷也不会丢失。
```shell
#直接使用命令
docker run -it -v 主机内目录：容器内目录 镜像名
#具名挂载卷
docker run -d -v 卷名:容器内目录 镜像名

docker run -d -v volume01:/home ubuntu /bin/bash
#通过命令 docker volume inspect 卷名  可以找到主机内目录/
```
### dockerfile
```shell
docker build -f <dockerfile path> -t <image name> . #构建镜像
docker push         #发布镜像
``` 