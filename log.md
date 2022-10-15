# develop分支更新日志
## 10.04更新
+ 重构了装甲板识别部分的代码，删除了没必要的判断
+ 注意代码注释中灯条矩形、装甲板矩形长宽及角度的定义
+ 将装甲板排序标准改为矩形倾角
## 10.06更新
+ 修改装甲板识别部分代码部分注释
+ 发现minAreaRect对轮廓拟合返回的旋转矩形形状很奇怪，无法拟合灯条形状
+ 将fitEllipse()函数返回的旋转矩形作为拟合灯条的矩形
+ 增加`tool_tutorials`文件的内容
## 10.14 更新
+ 去掉了HeadInfo结构体中yawspeed和pitchspeed变量
+ 去掉了PostureData结构体中的角速度和角加速度等多余的变量
+ 去掉fakeCarDetector这一层actor
+ 修改了装甲板识别actor的处理结果的结构体，具体见DetectedArmor.hpp
