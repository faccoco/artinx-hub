# 反前哨站：计算周期并以固定yaw角击打

## predictor：
+ 因位置观测误差较大，无法简单通过观测位置拟合圆找出圆心
+ 电控发送是否处于前哨站模式
+ 屏蔽静止装甲板
+ 因前哨站装甲板为下倾，光照少，反射少，亮度低，数字识别易失效，故将数字识别失效的小型装甲板和前哨站装甲板都当做目标装甲板
+ 因探头风险大，故直接利用先前记录周期
##### 流程图
```flow
start=>start: 收到观测到的装甲板们
initCond=>condition: 收到初始化
init=>operation: 记录枪管方向的yaw为目标yaw
yaw=>operation: 查找与目标yaw小于阈值且非静止的目标装甲板
yawCond=>condition: 找到
yawFail=>end: 直接退出
recordPos=>operation: 记录位置到结果
firstCond=>condition: 为第一个目标
first=>operation: 将第一个目标的pitch作为目标pitch并记录当前时间
lastCond=>condition: 存在上次周期
firstYesStdCond=>condition: 方差小于阈值
firstYesFail=>end: 清空周期并退出
firstYesRecordPeriod=>operation: 记录周期均值到结果
intervalLargeCond=>condition: 周期不太大
firstNoFail=>end: 清空周期并退出
intervalSmallCond=>condition: 周期不太小
intervalSmallFail=>end: 鉴定为上次记录的装甲板还未离开，直接退出
pitchCond=>condition: 与目标pitch差值小于阈值
firstNoStdCond=>condition: 方差小于阈值
firstNoRecordPeriod=>operation: 记录周期均值到结果
firstPeriodCond=>condition: 第一次算出周期
cleanPos=>operation: 删除结果中的位置
firstYesEnd=>end: 将结果发送给solver
firstNoEnd=>end: 将结果发送给solver
start->initCond
initCond(yes)->init->yaw
initCond(no)->yaw
yaw->yawCond
yawCond(yes)->recordPos->firstCond
yawCond(no)->yawFail
firstCond(yes)->first->lastCond
lastCond(yes)->firstYesStdCond
lastCond(no)->firstYesEnd
firstYesStdCond(yes)->firstYesRecordPeriod->firstYesEnd
firstYesStdCond(no)->firstYesFail
firstCond(no)->intervalLargeCond
intervalLargeCond(yes)->intervalSmallCond
intervalLargeCond(no)->firstNoFail
intervalSmallCond(yes)->pitchCond
intervalSmallCond(no)->intervalSmallFail
pitchCond(yes)->firstNoStdCond
pitchCond(no)->firstNoFail
firstNoStdCond(yes)->firstNoRecordPeriod->firstPeriodCond
firstNoStdCond(no)->firstNoFail
firstPeriodCond(yes)->cleanPos->firstNoEnd
firstPeriodCond(no)->firstNoEnd
```
##### 输出
输出结果有三种情况
+ 结果只含位置，指第一次检测到目标但周期未知，目的为朝大致方向提前转头
+ 结果含位置和周期，指第一次检测到周期，因第一次检测可能离相机中心远导致观测误差大，目的为进一步修正摆头位置并根据周期算出打弹时机
+ 结果只含周期，指后续更新周期


## solver
+ 延迟包含程序运行延迟、发弹延迟、子弹飞行延迟、固定偏置
+ 测试中发现观测周期稳定，但因发弹延迟波动、位置观测不准等原因，导致打击位置经常有小范围偏移，故采取第一次确定目标位置后不再修正枪管位置、只控制发弹时机的策略
+ 收到一次输入实际是根据周期打击一次后续时间上最近的可能的装甲板
+ 为防止actor阻塞，故延迟发弹采用开新线程sleep实现
+ 因采用开新线程sleep的方法实现延迟发弹，为保证切换前哨站模式时上次计划发送的指令不会被发出去，故使用一个计数，每次进入、退出前哨站模式时该计数会增加，每次发送指令前会检查该计数与刚分线程时的计数是否相等，若不相等，则不发送
##### 输入
输入有三种情况，见上文predictor输出
##### 流程图
```flow
start=>start: 收到输入
posCond=>condition: 输入含位置
solve=>operation: 角度解算
periodCond=>condition: 输入含周期
save=>operation: 保存角度解算结果
sendNo=>end: 发送角度解算结果，不发弹
calDelay=>operation: 计算等待时间
delay1=>operation: 第一次等待
send1=>operation: 发送保存的解算结果，不发弹
delay2=>operation: 第二次等待
send2=>operation: 发送保存的解算结果，发弹
end=>end: 结束
start->posCond
posCond(yes)->solve->periodCond
posCond(no)->calDelay
periodCond(yes)->save->calDelay
periodCond(no)->sendNo
calDelay->delay1->send1->delay2->send2->end
```

# 反陀螺：在反前哨算法的基础上更改
## predictor
+ 同反前哨站算法
+ 改为分别记录四个装甲板上次出现时间，故周期为一整圈的周期，以防相邻装甲板之间出现间隔不同
+ 因高速小陀螺观测飘的很厉害，删除了pitch限制以放宽匹配标准
+ 新增周期纠错功能，若方差超过阈值，会先尝试纠错，纠错失败或纠错次数过多才会重置
+ 因小陀螺可能只是因为短暂卡住而导致装甲板出现时间整体偏移，纠错会尝试通过对装甲板上次出现时间施加整体偏移来修正

## solver
+ 同反前哨站算法