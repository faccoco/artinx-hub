## 目标

只使用一根网线和自己的PC来调试机器人nuc

## 环境

假定nuc环境为 Ubuntu 20.04，PC环境为 Ubuntu 20 以上

## 设备

一根网线; 若PC没有网口，则另需一个USB转网口转换器（百兆速率的够用）

## 配置步骤

首先用网线连接两个设备

### 配置有线局域网的静态IP

nuc中打开设置 > Network > Wired，若连接没有问题则会有一条“Connected - xxx Mb/s"，启用，点击齿轮图标进行设置

在IPv4选项卡中，IPv4 Method 选 Manual，在Addresses表格里写入一条静态IP，学过计算机网络的随意配，没学过的参照步骤:

- IP地址: 

    形式为192.168.x.y，建议取192.168.1.1，y不填0，因为那是预留地址;

- 子网掩码:

    填255.255.255.0

- 路由:

    填自己的地址，即刚才的192.168.x.y

然后在PC中重复刚才的步骤: 打开设置 > Network > Wired > “Connected - xxx Mb/s"，启用，点击齿轮图标进行设置\

在IPv4选项卡中，IPv4 Method 选 Manual，在Addresses表格里写入一条静态IP，这次需要写另一条IP地址:

- IP地址: 

    192.168.x.z，保持x与前面给nuc选的x一致，z不等于y，若nuc按建议配了192.168.1.1则这边建议取192.168.1.2

- 子网掩码:

    填255.255.255.0

- 路由:

    填nuc的地址，192.168.x.y，不能填自己的

**完成后请牢记nuc的IP地址**

### (建议)启用nuc的ssh服务端
```
sudo apt-get install openssh-server
sudo systemctl enable ssh
# 打开防火墙22端口
sudo ufw allow 22
sudo systemctl restart ufw
```

### 开启nuc的vnc服务

Ubuntu自带vnc服务（不知道从哪个版本开始支持，但Ubuntu 20 及以上是有的）

nuc打开设置 > Sharing > Screen Sharing，点开，勾选"Allow connections to control the screen", 

下面单选框点"Require a password", 设定密码, 记住（记不住就弄成登录密码）

Networks选择我们的有线网络

然后打开左上角的开关，即可启用VNC桌面

然后命令行输入
```
sudo ufw allow 5900
sudo systemctl restart ufw
```
打开防火墙5900端口

**目前已知有一个bug: 若nuc连接了无线网络，那么打开Screen Sharing时有线网络将无法被选择**

**在nuc需要上网，同时又需要被PC控制时，有两种解决方法**

1. 换用Todesk等软件，可通过（且必须通过）线上互联网控制nuc，此时PC和nuc都需通过无线网络连入互联网才可控制

2. 在PC和nuc的IP设置里都把路由从nuc的IP地址改成PC的IP地址，此时若PC能上网，则nuc也能上网; 

    但是这样做，除了你自己的PC以外，其他人的电脑就不能有线连上这个nuc了，除非TA的电脑静态IP设置的跟你一样（

具体如何解决自己决定吧（

### PC使用Remmina连接VNC桌面

此处以Remmina 1.4.25为例

Ubuntu 22.04预装了Remmina 1.4.25，你的PC没有预装的话也可以去下载一个

点击Remmina左上角按键，新建配置，Protocol选择Remmina VNC Plugin，Server填<nucIP地址>:5900，或者点击右边的省略号进行自动搜索，在找到的两个服务器里面选择IPv4的那个

点击Save and Connect保存并连接，输入密码，即可连接

## 常见问题

1. 连接后黑屏

    若没有连接HDMI显示器，则Xorg服务器不提供图形界面，解决方法(二选一):

    1. 淘宝一个显卡诱骗器，插上去

    2. 配置虚拟显示器:

        https://zhuanlan.zhihu.com/p/514763585

        1. 安装软件

            ```
            sudo apt-get install  xserver-xorg-core-hwe-18.04
            sudo apt-get install  xserver-xorg-video-dummy
            ```

        2. 添加配置文件

            在/usr/share/X11/xorg.conf.d/ 目录中新增 xorg.conf 文件，把下面内容复制进去

            ```
            Section "Device"
                Identifier  "Configured Video Device"
                Driver      "dummy"
            EndSection
            Section "Monitor"
                Identifier  "Configured Monitor"
                HorizSync 31.5-48.5
                VertRefresh 50-70
            EndSection
            Section "Screen"
                Identifier  "Default Screen"
                Monitor     "Configured Monitor"
                Device      "Configured Video Device"
                DefaultDepth 24
                SubSection "Display"
                Depth 24
                Modes "1920x1080"
                EndSubSection
            EndSection
            ```

        3. 重启电脑，默认会进入虚拟显示器

            **进入系统后真实显示器里就不会显示了，注意！！！！**

            关闭虚拟显示器的方式是命令行删除刚才的/usr/share/X11/xorg.conf.d/xorg.conf文件，然后重启，此时真实显示器中即可恢复正常显示

            **如果没有以下其一: 有线网络+ssh、有线网络+VNC(即此文档介绍的方式)/RDP、Todesk等远程桌面的终端控制、或者其他能ssh上NUC终端的方式，那么切勿配置虚拟显示器！**

            另外，虚拟显示器配置文件里写了1920x1080，但事实上它只支持1300多x多少忘了的分辨率，所以窗口会略小

    Todesk等远控软件的黑屏也是由此原因导致的，同样适用以上解决方法