# AngleSolver

## 流程图
```flow
start=>start: 收到车辆状态
tf=>operation: 转换到角度解算坐标系（右x前y上z）
select=>operation: 选择下一块装甲板
iter=>operation: 通过预估时间迭代求解
solve=>operation: 按预估位置匀速运动求解
solveCond=>condition: 有解
iterCond=>condition: 迭代成功
iterTimeCond=>condition: 迭代超时
yaw=>operation: 计算入射角（yaw角差）
yawCond=>condition: 角度够小
record=>operation: 记录解算结果
recordCond=>condition: 有解算结果
end=>end: 发送结算结果
start->tf->select->iter->solve->solveCond
solveCond(yes)->iterCond
solveCond(no)->recordCond
iterCond(yes)->yaw->yawCond
iterCond(no)->iterTimeCond
iterTimeCond(yes)->recordCond
iterTimeCond(no)->iter
yawCond(yes)->record->recordCond
yawCond(no)->recordCond
recordCond(yes)->end
recordCond(no)->select
```


## 解算方法：
+ 以车辆自身作为参考系
+ 考虑视觉算法处理的延迟、串口通信的延迟，云台电机执行动作的延迟以及子弹从拨弹轮到射出的延迟，这几类延迟之和设为$t_{f}$
    $V_0$: 子弹相对车速度    $V_{0h}$ : 子弹的相对车水平面方向速度 	$V_{0v}$:子弹竖直方向的净速度  $V_2$: 目标移动的速度
    $V_1:$ 车的速度   $S$: 目标到枪口水平面方向的距离      $t$: 子弹从枪口射出到命中障碍物的时间   $h$: 目标到枪口的竖直高度
+ 未知量有$\vec{V_{0h}}$  $V_{0v}$  $t$，其余量已知

经过固定延迟$t_f$后，目标移动到$\vec{X_f}$处

​$\vec{S_f} = \vec{S} + (\vec{V_2}-\vec{V_1})t_f$
$V_{0v}t + \frac{1}{2}gt^2 = h$                             --式一
$\vec{V_{0h}}t = \vec{S_f} + (\vec{V_2} - \vec{V_1})t$
$\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ 
$V_{0h}cos\theta t= S_{fx} + ({V_2} - {V_1})_xt$            --式二
$V_{0h}sin\theta t= S_{fy} + ({V_2} - {V_1})_yt$            --式三
$V_0^2 = V_{0h}^2 + V_{0v}^2 $                              --式四

联立式一、式二、式三、式四，利用`matlab`可解得一个关于`t`的四次方程， `matlab`脚本如下，图片不方便插入，想看结果自己运行查看一下。

```matlab
syms V0v V0hx V0hy t; 
syms Sx Sy Vx Vy h g V0;
S1 = V0v*t + 1/2 * g * t * t - h;
S2 = V0hx*t - Sx  - Vx * t;
S3 = V0hy*t - Sy  - Vy * t;
S4 = V0 * V0 - V0v * V0v - V0hx * V0hx - V0hy * V0hy;
[V0v, V0hx, V0hy, t] = solve(S1, S2, S3, S4, V0v, V0hx, V0hy, t)
```

通过ferrari解一元四次方程解得t后，其他量可简单求之，最后

```c++
double pitchAngle = std::asin(verticalSpeed / bulletSpeed);
double yawAngle = std::atan2(horizontalSpeedY, horizontalSpeedX);
```


## 迭代方法：
+ 因装甲板运动复杂，无法直接计算，故对于每个装甲板都采用迭代法求解，先预估位置再将其当做匀速直线运动求解，当预估时间和计算所得飞行时间加上延迟的差值小于阈值则认为迭代成功
+ 因子弹速度快，装甲板移动对子弹飞行时间基本影响不大，每次更新迭代时将预测时间直接改为飞行时间加延迟即可，该方法迭代次数一般为两三次
+ 若迭代失败，很有可能是传入的速度、角速度异常大


## 入射角：
+ 此处入射角仅考虑入射时水平平面角度
