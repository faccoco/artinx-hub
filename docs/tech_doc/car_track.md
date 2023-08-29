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

跟踪器使用EKF(拓展卡尔曼滤波)来估计装甲板的状态。
[卡尔曼滤波与拓展卡尔曼滤波](https://www.zhihu.com/tardis/zm/art/81404580?source_id=1003)

EKF状态量与观测量如下：

状态量：[xc, yc, zc, yaw, v_xc, v_yc, v_zc, v_yaw, r]
 \- xc, yc, zc：车辆中心位置。
 \- yaw：装甲板yaw角。
 \- v_xc, v_yc, v_zc：车辆中心线速度。
 \- v_yaw：装甲板的角速度。
 \- r：装甲板相对车辆中心的距离。

 \- 观测向量：[xa, ya, za, yaw]，其中：
 \- xa, ya, za：装甲板在世界坐标系下的位置。
 \- yaw：装甲板的yaw角, 正对装甲板为 90°

 \- 观测方程
 $x_a = x_c - r * cos(\theta)$
 $y_a = y_c - r * sin(\theta)$
 $z_a = z_a$
 $\theta = \theta$

 \- 状态转移函数
 $x'_c = x_c + v_{xc} * dt$
 $y'_c = y_c + v_{yc} * dt$
 $z'_c = z_c + v_{zc} * dt$
 $\theta' = \theta +v_\theta*dt$

 \- 状态转移误差协方差矩阵

```
double t = mDt, x = mConfig.sigma2Qxyz, y = mConfig.sigma2Qyaw, r = mConfig.sigma2QR;
            double Qxx = pow(t, 4) / 4 * x, QxVx = pow(t, 3) / 2 * x, QVxVx = pow(t, 2) * x;
            double Qyy = pow(t, 4) / 4 * y, QyVy = pow(t, 3) / 2 * x, QVyVy = pow(t, 2) * y;
            double QR = pow(t, 4) / 4 * r;

            // clang-format off
            //    xc        yc      zc      yaw     vxc     vyc     vzc     vyaw    r
            q <<    Qxx,    0,      0,      0,      QxVx,   0,      0,      0,      0,
                    0,      Qxx,    0,      0,      0,      QxVx,   0,      0,      0,
                    0,      0,      Qxx,    0,      0,      0,      QxVx,   0,      0,
                    0,      0,      0,      Qyy,    0,      0,      0,      QyVy,   0,
                    QxVx,   0,      0,      0,      QVxVx,  0,      0,      0,      0,
                    0,      QxVx,   0,      0,      0,      QVxVx,  0,      0,      0,
                    0,      0,      QxVx,   0,      0,      0,      QVxVx,  0,      0,
                    0,      0,      0,      QyVy,   0,      0,      0,      QVyVy,  0,
                    0,      0,      0,      0,      0,      0,      0,      0,      QR;
```
\- 观测误差协方差矩阵

```
auto UR = [this](const Eigen::VectorXd& z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = mConfig.Rxyz;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), mConfig.Ryaw;
            return r;
        };
```
注：你或许会注意到Q的值与帧差有关，而R的值与大小有关。仔细想想，为什么？有什么好处？

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



## 遗留问题
### 识别相关
1. 目前识别器的二值化为了保证轮廓的完整性没有分红蓝，因此导致鲁棒性较差。
2. 在切换装甲板目标时，云台通常会发生一次幅度较大的转动；此次转动在相机曝光较大的情况下很可能导致拖影，使pnp效果变差。
### 跟踪相关
1. 目前的代码框架没有跟踪器的可视化，导致调参极其痛苦。可以考虑将跟踪器计算出的位置重投影到图像上输出。
2. 当装甲板正对相机时，pnp解算出的旋转角跳变现象十分严重。
原因可能是当装甲板正对相机时 pnp 只能通过装甲板矩形短边差来判断朝左还是朝右。朝向偏右20°与朝向偏左20°的两个装甲板，他们在图像上的差别很小；况且识别器本身也有一定误差。
目前已知的解决方案有两种：
较为简单方法的是二值化时尽可能将二值化阈值拉低，这样获取到的灯条比较稳定(灯条边缘一般较暗，高二值化阈值很容易将灯条边缘过滤掉)。但这样带来的问题是，调低阈值后，二值图的处理(查找轮廓，轮廓匹配等)需要的时间将是一个非常恐怖的数字。因此，上赛季我们的思路是使用轻量yolo网络先筛选出装甲板的外接矩形作为roi，再在roi中进行低阈值二值化操作。应该可行，但尚待验证。
第二种方法由上海交通大学提出，并验证过可行性，不过实现难度较大且使用有一定限制。
大体思路是利用装甲板的自由度。规则规定：机器人装甲板的安装必须满足roll=0°, pitch=15°(相对机器人)。在pnp解算出装甲板中心的位置后，我们可以固定pitch、roll，以yaw作为自变量对装甲板进行重投影，再设计一个损失函数衡量重投影的准确度。这样，我们能够得到一个自变量为yaw, 因变量为loss的下凸曲线，loss曲线的最低点就是yaw角的最佳值。
由于运算速度的限制不可能把整个曲线求出来，因此可以使用三分法(上交采用, 打过OI的应该很熟悉)或梯度下降法进行迭代最优化。
这个方法，似乎没法处理敌方机器人处于斜坡上的情况，试试想想有没有优化方法？
