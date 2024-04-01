# Jetson Startup Set

## 使用 crontab 命令设置开机启动项（建议转到root用户执行）
当需要有程序在systemd前执行时，可以使用`crontab -e`命令编辑crontab文件。
1. 在/etc/rc.local/（或任意一个文件夹，随你喜欢，但没试过）文件夹下创建启动脚本。这里以startup.sh为名。
2. 输入`crontab -e`命令，在文件末尾添加`@reboot /etc/rc.local/startup.sh`行。将`/etc/rc.local/startup.sh`更换为你的脚本路径。
3. `chmod +x /etc/rc.local/startup.sh`给脚本可执行权限。

## 拉满风扇转速
jetson orin nx 的风扇不是太行，放任它自己调节经常会导致cpu降频。使用`/usr/bin/jetson_clocks --fan `命令提至满转能有效避免发热问题。

## jetson 时间戳
jetson orin nx没有硬件时钟(rtc)，开机时的时钟默认为1970年1月1日。fake-hwclock命令能够保存当前时间戳，结合开机执行的fake-hwclock load命令可以一定程度避免log时间戳重复的问题。

## startup.sh文件内容
```
#!/bin/bash
sleep 10
/usr/bin/jetson_clocks --fan    #拉满风扇转速
fake-hwclock load               #使用上一次保存的时间戳作为系统时间
fake-hwclock                    #保存当前时间戳
while true; do
        fake-hwclock            
        sleep 30                #每隔30s保存一次时间。
done
```