# Artinx 2023 赛季 自瞄

Ref: 华南师范大学 https://github.com/chenjunnn/rm_vision(已失效) author:陈君

替代仓库：https://gitlab.com/rm_vision 

本md仅为算法的粗略解释，官方算法详解参见nas： 视觉共享文件夹/rm_vision/陈君-毕设.pdf

## 装甲板识别

ArmorDetector.cpp使用传统视觉，通过二值化筛选轮廓的方式获得装甲板目标

### 二值化方案

按灰度阈值二值化。相比红蓝通道相减，此方式能够避免相机动态范围过低导致的灯条镂空。

cv::Mat grayImg;
 cv::cvtColor(src, grayImg, cv::COLOR_BGR2GRAY);
 
 cv::Mat binaryImg;
 cv::threshold(grayImg, binaryImg, mConfig.binaryThresh, 255,    cv::THRESH_BINARY);

“装甲板识别”部分中，"灯条"代指灯条轮廓的外接矩形，"装甲板"代指成功匹配的两灯条轮廓，外接矩形的短边中点（共四个点，每个灯条轮廓提供两短边）构成的矩形。



### 灯条筛选

1. 灯条轮廓外接矩形角度
2. 灯条轮廓外接矩形长宽比



### 灯条匹配

1. 灯条长度比例
2. 灯条角度差
3. 灯条轮廓外接矩形长宽比
4. 装甲板长宽比
5. 装甲板角度

额外的，满足这些条件的装甲板将送入数字识别器中进行处理

 



### 数字识别

数字识别器采用两层MLP，使用CIFAR-100作为负样本

下载地址：https://www.cs.toronto.edu/~kriz/cifar.html

 



## 整车跟踪（反陀螺）

CarPredictor.cpp使用EKF(拓展卡尔曼滤波)来估计装甲板的状态。
[卡尔曼滤波与拓展卡尔曼滤波](https://www.zhihu.com/tardis/zm/art/81404580?source_id=1003)

EKF状态量与观测量如下：

状态量：[xc, yc, zc, yaw, v_xc, v_yc, v_zc, v_yaw, r]
 \- xc, yc, zc：车辆中心位置。
 \- yaw：装甲板yaw角。
 \- v_xc, v_yc, v_zc：车辆中心线速度。
 \- v_yaw：装甲板的角速度。
 \- r：装甲板相对车辆中心的距离。
 
 \- 观测向量：[xa, ya, za, yaw]，其中：
 \- xa, ya, za：装甲板在相机坐标系下的位置。
 \- yaw：装甲板的yaw角。
 
 \- 状态转移函数
 𝑥𝑎 = 𝑥𝑐 − 𝑟∗𝑐𝑜𝑠(𝜃)
 𝑦𝑎 = 𝑦c − 𝑟∗𝑠𝑖𝑛(𝜃)
 𝑧𝑎 = 𝑧𝑎 
 𝜃=𝜃

每一帧，程序会根据上一帧的装甲板位置，依据状态转移方程推断一个预测位置。程序会将观测到的，离预测位置最近的装甲板，与预测位置做对比，距离小于一定值才会更新。

特别地，当距离大于一定值，但候选目标中存在与原装甲板id相同的装甲板，则处理装甲板跳变

### 装甲板跳变

当怀疑出现装甲板跳变时，用观测的yaw角替代原始yaw角重新推断出一个预测位置。若更新后观测位置与推断位置匹配，则更新，否则重置线速度。



## 角度解算

见 angle_solver.md

## 日志
### 命令行
`LogInfo()`。

### web可视化
`HubLogger::watch()`

### 文件
`HubLogger::visualLog()`
日志文件保存在data/logs目录下。

