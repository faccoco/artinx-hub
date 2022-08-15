# Code Details

## Identifier

```c++
//代码中的key就是一个Identifier结构体，本质是一个64位整型值
//key的生成见HubHelper类
struct Identifier {
    uint64_t val;
};

//带上BlackBoard上的数据类型进行ACTOR_PROTOCOL_CHECK
template <typename T>
struct TypedIdentifier final : Identifier {
    using Payload = T;
};
```

## HubHelper

+ `std::variant`
    + 类似C语言中的union，保存可能存储的类型列表之一的对象。

+ `noexcept`
    + 修饰函数表示不会抛出异常。

+ `reinterpre_cast`
    + 强制转型，用来处理无关类型之间的转换， 它会产生一个新的值，这个值会有与原始参数值有完全相同的比特位。

+ `std::get(std::tuple)`
    + `std::get<N>(t)` 提取`tuple`的第N个元素
    + `std::get<typename>(t)`提取`tuple`中`typename`类型的元素，如果`tuple`中不只一个该类型元素，则编译失败。

```c++
//T: 继承的actor基类    config: 配置文件参数   Succeed: 需要发送的atom
template <typename T, typename Config, typename... Succeed>
class HubHelper : public T {
    //静态断言，编译期判断T的基类是否为caf::abstract_actor
    static_assert(std::is_base_of_v<caf::abstract_actor, T>);

    //模板参数Lable指明了需要发送Atom的类型
    template <typename Label>
    struct SucceedAddress final {
        std::variant<std::vector<std::string>, std::vector<std::pair<caf::actor_addr, GroupMask>>> val;
    };
	
    //传啥返回啥，这样做的好处？
    template <typename Arg>
    static const Arg& wrap(const Arg& arg) noexcept {
        return arg;
    }
	
    //抹掉TypedIdentifier的类型
    template <typename Arg>
    static const Identifier& wrap(const TypedIdentifier<Arg>& arg) noexcept {
        return static_cast<const Identifier&>(arg);
    }

    //可变参数模板 ex: for armorDetector, mDest ==> std::tuple<SucceedAddress<armor_detect_available_atom>, SucceedAddress<image_frame_atom>>
    std::tuple<SucceedAddress<Succeed>...> mDest;

    template <typename Atom>
    const auto& getDest() {
        auto& dest = std::get<SucceedAddress<Atom>>(mDest).val;
        if(dest.index() == 0)
            dest = detail::parseSucceed(this->system(), std::get<0>(dest));
        return std::get<1>(dest);
    }

protected:

   //在编译期，给定不同的参数，返回不同的类型
   //std::is_void_v<config>为true，返回类型char,否则返回Config
    std::conditional_t<std::is_void_v<Config>, char, Config> mConfig;
    
    //GroupMask 32位整型，哨兵两个云台需要辨别上下云台数据，引入该Groupmask机制进行区分
    GroupMask mGroupMask;

   /**
   * @brief				根据this指针生成Key
   * @param thisPointer self类型的this指针
   * @retrun			某个actor类实例化的对象对应的key， key=类型的hash_code 异或 类实例化后的对象this指针的值的结果	
   */
    template <typename Self>
    static Identifier generateKey(Self* thisPointer) {
        return { typeid(Self).hash_code() ^ reinterpret_cast<uintptr_t>(thisPointer) };
    }

public:
    /**
   * @brief				构造函数，读取配置文件，初始化actor
   						1.如果配置文件中，如果设置了group_mask则mGroupMask为group_mask,如果设置了						                group_id则mGroupMask为1<<group_id,，否则默认为1
   * @param base 		actor基类
   * @param config		配置文件
   */
    HubHelper(caf::actor_config& base, const HubConfig& config)
        : T{ base }, mDest{ SucceedAddress<Succeed>{ detail::parseSucceed(config, typeid(Succeed).name()) }... } {
        if constexpr(!std::is_void_v<Config>) {
            if(auto configValue = caf::get_as<Config>(config)) {
                mConfig = std::move(configValue.value());
            } else {
                logError("Bad config");
            }
        }

        const auto& dict = config.to_dictionary();
        if(const auto iter1 = dict->find("group_mask"); iter1 != dict->cend()) {
            mGroupMask = static_cast<uint32_t>(iter1->second.to_integer().value());
        } else if(const auto iter2 = dict->find("group_id"); iter2 != dict->cend()) {
            mGroupMask = 1U << static_cast<uint32_t>(iter2->second.to_integer().value());
        } else {
            mGroupMask = 1U;
        }
    }
    /**
   * @brief				根据配置文件内容给对应的actor发送atom，从而触发相应lambada函数调用
   * @param atom
   * @param args	
   */
    template <typename Atom, typename... Args>
    void sendAll(Atom atom, Args&&... args) {
        ACTOR_PROTOCOL_CHECK(Atom, std::decay_t<Args>...);
        for(auto&& [address, mask] : getDest<Atom>())
            this->send(caf::actor_cast<caf::actor>(address), atom, wrap(std::forward<Args>(args))...);
    }

    template <typename Atom, typename... Args>
    void sendMasked(Atom atom, GroupMask mask, Args&&... args) {
        ACTOR_PROTOCOL_CHECK(Atom, std::decay_t<Args>...);
        for(auto&& [address, maskRhs] : getDest<Atom>())
            if(mask & maskRhs)
                this->send(caf::actor_cast<caf::actor>(address), atom, wrap(std::forward<Args>(args))...);
    }
};
```

## BlackBoard

+ `std::shared_mutex`
    + 用于多个读线程能够同时访问同一资源而不导致数据竞争，但只有一个写线程能访问的情形。

+ `std::lock_guard`
    + 根据RAII，将锁的持有视为资源，构造函数里调用`mtx.lock()`获取锁，则上锁；退出函数作用域时，调用`mtx.unlock()`析构释放锁，即解锁。

+ `std::any `和`std::any_cast<T>`
    + `std::any`可用于任何类型单个值的类型安全的容器, `std::any_case<T>`类型转换

+ `std::nullopt`
    + 空类类型，指示具有未初始化状态的`optional`类型

```c++
class BlackBoard final {
  	//存放大型结构体的Map, actor发送的数据都存放在这里, size_t为值类型的哈希编码和key异或的结果， 下面简称哈希码
    std::unordered_map<size_t, std::pair<std::shared_mutex, std::any>> mItems;
    //读写锁，访问mItems时上锁
    std::shared_mutex mMutex;
    
  /**
   * @brief				向黑板上插入值
   * @param hashValue 	哈希码
   * @param val 	  	需要插入的值	
   */
    void insertImpl(size_t hashValue, std::any val);
    
   /**
   * @brief 			根据哈希码从黑板上拿值
   * @param hashValue   值类型的哈希码
   * @return 	  		拿到的值	
   */
    std::pair<std::shared_mutex, std::any>* getImpl(size_t hashValue);

public:
    /**
   * @brief 			根据key从黑板上拿值
   * @param key   		每个actor？atom?对应的整形值?
   * @return 	  		拿到的值	
   */
    template <typename T>
    std::optional<T> get(const Identifier key) {
        if(const auto ptr = getImpl(typeid(T).hash_code() ^ key.val)) {
            std::shared_lock guard{ ptr->first };
            return std::any_cast<T>(ptr->second);
        }
        return std::nullopt;
    }
    /**					
   * @brief 			更新黑板上的值
   * @param key   		每个actor？atom?对应的整形值?
   * @return 	  		key的值	
   */
    template <typename T>
    TypedIdentifier<T> updateSync(const Identifier key, T val) {
        const auto hashCode = typeid(T).hash_code() ^ key.val;
        if(const auto ptr = getImpl(hashCode)) {
            std::lock_guard guard{ ptr->first };
            ptr->second = std::move(val);
        } else
            insertImpl(hashCode, std::move(val));
        return { { key.val } };
    }

    static BlackBoard& instance();
};
```

## SolvePnP

+ *Reference*:[OpenCV: Perspective-n-Point (PnP) pose computation](https://docs.opencv.org/3.4/d5/d1f/calib3d_solvePnP.html)

## Coordinate system regulation and coordinate transformation

### Coordinate system regulation

+ 坐标系统一采用右手系

+ 装甲板坐标以装甲板中心为原点，装甲板平板为xy平面。
+ 相机坐标系原点为相机光心的位置，沿着相机方向向后为z轴正，垂直相机向右为x轴正，垂直相机向上为y轴正。
+ 枪管坐标系的原点规定在枪管pitch轴旋转的两个支点的中点处，沿枪管朝前为z轴负，垂直枪管为向右为x轴正。
+ 机器人坐标系原点规定在底盘中心，向正右方为x轴正，正上方为y轴正，正后方为z轴正。

+ 世界坐标系原点和机器人坐标系原点重合

### Coordinate transformation

+ 从装甲板坐标系到相机坐标系

    通过`SolvePnP`能得到，从装甲板坐标系到相机坐标系的变化矩阵$^cT_w$

    取装甲板中心，即$X_w = 0，Y_w = 0, Z_w = 0$,则 $X_c = t_x, Y_c=t_y, Z_c=t_z$,由于`opencv solvePnP`中规定的y轴方向和z轴方向相反，所以y轴和z轴方向还需要做一个取负的运算。

+ 从相机坐标系到枪管坐标系

    由于相机安装会和枪管有一个固定的偏置，所以从相机坐标系到枪管坐标系需要有一个平移变换。

+ 从机器人坐标系从枪管坐标系

    利用`glm::lookAtRH`函数，以机器人坐标系为原点，去看枪管坐标系。

    *Reference*:[摄像机+LookAt矩阵+视角移动+欧拉角 - Garrett_Wale - 博客园 (cnblogs.com)](https://www.cnblogs.com/GarrettWale/p/11336589.html)
    
    ```c++
        /**					
       * @brief 				根据要变换到的坐标系的原点在当前坐标系的位置，和三个向量的方向在当前坐标系的位置来求得两个坐标系之间的变换
       * @param1 eye			要变换到的坐标系原点在当前坐标系的位置，
       							枪管坐标系原点在机器人坐标系（0.0, mConfig.headHeightOffset1, mConfig.headForwardOffset1）
       * @param center 	  	    要变换到的坐标系三个向量的方向，根据yaw角和pitch角推出，可以自己想一想怎么推出来的
       * @param up				上向量
       */
    const HeadInfo infoUp{ SynchronizedClock::instance().now(),
                                       decltype(HeadInfo::transform){ glm::lookAtRH(
                                           glm::dvec3{ 0.0, mConfig.headHeightOffset1, mConfig.headForwardOffset1},
                                           glm::dvec3{ std::cos(static_cast<double>(fdb.yaw) + glm::half_pi<double>()) *
                                                           std::cos(static_cast<double>(fdb.pitch)),
                                                       mConfig.headHeightOffset1 + std::sin(static_cast<double>(fdb.pitch)),
                                                       mConfig.headForwardOffset1 - std::sin(static_cast<double>(fdb.yaw) + glm::half_pi<double>()) *
                                                           std::cos(static_cast<double>(fdb.pitch)) },
                                           glm::dvec3{ 0.0, 1.0, 0.0 }) },
    ```
    
    

## Angle Solver

$V_0$: 子弹的净速度    $V_{0h}$ : 子弹的水平面方向净速度 	$V_{0v}$:子弹竖直方向的净速度   $V_h$:子弹水平面方向的

 $V_1:$ 车的速度     $V_x$: 子弹x方向的合速度     $V_y$:子弹y方向的合速度

$S$: 水平方向的距离

$V_{x} = V_{1x} + V_{0hx}$   $V_y = V_{1y} + V_{0hy}$

$V_hsin\theta = V_{ohy} + V_{1y}$  $V_hcos \theta = V_{0hx} + V_{1x}$

$V_h^2=V_x^2+V_y^2$

$V_{0v}\frac{S}{V_h}-\frac{1}{2}g\frac{S^2}{V_h^2}=h$

$V_0^2= V_{0h}^2 + V_{0v}^2$

$\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ 

$V_{0v}^2S^2V_h^2=h^2V_h^4+1/4gs^2+hgs^2V_h^4$

$(V_0^2 - V_{0h}^2)S^2V_h^2=h^2V_h^4+1/4gs^2+hgs^2V_h^4$ 

$(V_0^2-(V_hsin \theta - V_{1y})^2-(V_hsin \theta - V_{1x})^2)S^2h^2=h^2V_h^4+1/4gs^2+hgs^2V_h^4$

$(V_0^2 - V_h^2 + 2V_hV_{1y}sin \theta + 2V_hV_{1x}cos \theta - (V_{1y}^2+V_{1x}^2))S^2h^2=h^2V_h^4+1/4gs^2+hgs^2V_h^4$

+ 以车辆自身作为参考系

+ 考虑视觉算法处理的延迟、串口通信的延迟，云台电机执行动作的延迟以及子弹从拨弹轮到射出的延迟，这几类延迟之和设为$t_{f}$

    $V_0$: 子弹相对车速度    $V_{0h}$ : 子弹的相对车水平面方向速度 	$V_{0v}$:子弹竖直方向的净速度  $V_2$: 目标移动的速度

     $V_1:$ 车的速度   $S$: 目标到枪口水平面方向的距离      $t$: 子弹从枪口射出到命中障碍物的时间   $h$: 目标到枪口的竖直高度

+ 未知量有$\vec{V_{0h}}$  $V_{0v}$  $t$，其余量已知

经过固定延迟$t_f$后，目标移动到$\vec{X_f}$处

​					$\vec{S_f} = \vec{S} + (\vec{V_2}-\vec{V_1})t_f$

$V_{0v}t + \frac{1}{2}gt^2 = h$     												 --式一

$\vec{V_{0h}}t = \vec{S_f} + (\vec{V_2} - \vec{V_1})t$

$\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ 

$V_{0h}cos\theta t= S_{fx} + ({V_2} - {V_1})_xt$   						--式二

$V_{0h}sin\theta t= S_{fy} + ({V_2} - {V_1})_yt$							--式三

$V_0 = V_{0h}^2 + V_{0v}^2 $														--式四

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

解得t后，其他量可简单求之，最后

```c++
//由于机器人的yaw轴的零点在视觉定义的坐标系的pi/2处，所以求得的pitch角要减pi/2
double pitchAngle = std::asin(verticalSpeed / bulletSpeed);
double yawAngle = std::atan2(horizontalSpeedY, horizontalSpeedX) - glm::half_pi<double>();
```

## Sentry actor workflow

+ `camera_up `和`camera_down`
  
    + 初始化由于类实例化的对象地址不同，所以`mkey`值不相同，对应的在`blackboard`上的`CameraFrame`的哈希值不同。
    + `mGroup`未设置，都为1。
+ `sendAll(image_frame_atom_v, BlackBoard::instance().updateSync(mKey, std::move(frameData)))`。
  
+ `serial`

    + `mGroup`未设置，为1。
    + `sendAll(update_posture_atom_v, BlackBoard::instance().updateSync(mKey, posture))` 发送姿态信息
    + `sendMasked(update_head_atom_v, 1U, 1U, BlackBoard::instance().updateSync(mKey, infoUp))`。发送上云台枪管坐标系到机器人坐标变换矩阵信息。
    + ` sendMasked(update_head_atom_v, 2U, 2U, BlackBoard::instance().updateSync(Identifier{ mKey.val ^ 0xffffffff }, infoDown))`;发送下云台枪管坐标系到机器人坐标系的变换矩阵信息。

    + 接收 `[this](set_target_info_atom, GroupMask mask, Clock::rep begin, double yawAngle, double pitchAngle, bool isFire)`    `mGroupMask == 1U` 为上云台数据，否则为下云台数据。

+ `detector_up`和`detector_down`
    + 接收 `image_frame_atom` ，发送`sendAll(car_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)))`
    + config 文件中配置了`detector_up`和`detector_down`分别接收`camera_up`和`camera_down`的`atom`和`key`。

+ `armor_detector_up`和`aromor_detector_down`
  
+ 接受 `car_detector_available_atom`, 发送`sendAll(armor_detect_available_atom_v, BlackBoard::instance().updateSync(mKey, std::move(res)))`
  
+ `armor_locator_up` 和`armor_locator_down`

    + 在config文件中`armor_loctor_up` 的`group_id`为0， `armor_locator_down ` 的 `group_id`为 1。

    + 接收`armor_detect_available_atom`, 发送`sendAll(detect_available_atom_v, mGroupMask, BlackBoard::instance().updateSync(mKey, std::move(res)))`
    + 接收 `(update_head_atom, GroupMask, Identifier key)`, 更新`actor`的 `mHeadKey`

+ `strategy`

    + 接收`(detect_available_atom, GroupMask mask, Identifier key)`

    + 如果`selected`有数据，` (mask == 1U ? mLastSelected1 : mLastSelected2) = selected`;

        如果`selected`没有数据，`selected = (mask == 1U ? mLastSelected2 : mLastSelected1)`,指从另外一个云台拿数据。

    + 发送`sendMasked(set_target_atom_v, mask, BlackBoard::instance().updateSync<SelectedTarget>(Identifier{mKey.val ^ mask}, selected));`，注意`sendMasked`指定了接收对象
    + 接收`(update_head_atom, GroupMask mask, Identifier key)`,更新`mHead1`和`mHead2`的数据。

+ `angleSolve_up` 和`angleSolve_down`
    +  接收`(set_target_atom, Identifier key)`
    + 发送 `sendAll(set_target_info_atom_v, mGroupMask, data.value().lastUpdate.time_since_epoch().count(), yawAngle, pitchAngle, isFire)`
    + 接收`(update_head_atom, GroupMask, Identifier key)`, 更新`mHeadKey`
    + 接收 `(update_posture_atom, Identifier key)`, 更新`mIMUKey`