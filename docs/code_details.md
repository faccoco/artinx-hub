# Code Details

## HubClassRegister

+ 宏定义中`#`的作用

    + 一个`#`:将其后面的宏参数进行字符串化

    + 两个`##`:在带参数数的宏定义中将两个子串(token)连接起来，从而形成一个个新的字串。子串(token)指编译器能够识别的最小语法单元。

        ```c++
        #define PRINT(N) printf("token"#N" = %d", token##N)
        
        int token9 = 3;
        PRINT(N)   //printf("token9 = %d", token9)
        
        ```

      总之，宏中的参数仅仅只有一个替换的作用。

+ 初始化时机

    + **全局变量、文件域中的静态变量、类中的成员静态变量在main函数执行前初始化；局部变量中的静态变量在第一次调用时初始化。**

```c++
//所有actor类由于声明定义了一个static静态变量的HubClassRegister类， 因此都会在进入main函数之前初始化一个hubClassRegister##CLASS_NAME的变量。
#define HUB_REGISTER_CLASS(CLASS_NAME) static detail::HubClassRegister<CLASS_NAME> hubClassRegister##CLASS_NAME

//HubClassRegister类的构造函数中，调用registerComponent函数，将acotor类的名称、生成的方法注册到工厂类的mClasser变量中，由工厂类负责生成actor类对象。
void registerComponent(const char* name, std::function<caf::actor(caf::actor_system&, const HubConfig&)> spawnFunction) {
    NodeFactory::get().addNodeType(std::string{ name }, std::move(spawnFunction));
}

template <typename NodeType>
class HubClassRegister final : Unmovable {
    public:
    HubClassRegister() {
        registerComponent(typeid(NodeType).name(), [](caf::actor_system& system, const HubConfig& config) -> caf::actor {
            return system.spawn<NodeType>(config);
        });
    }
};

```

## NodeFactory

```c++
//工厂类，制造生成acotr
class NodeFactory final : Unmovable {
    //存放actor的名称和制造acotr的方法
    std::unordered_map<std::string, std::function<caf::actor(caf::actor_system&, const HubConfig&)>> mClasses{};

public:
    //由HubClassRegister类调用，把actor的名称和制造方法存放到mClass中。
    void addNodeType(std::string name, std::function<caf::actor(caf::actor_system&, const HubConfig&)> spawnFunction);
 	//根据配置文件中给定actor的type，在mClasser中寻找与其名称相对的actor，并调用其制造方法生成该actor，并将其注册到系统中。
    caf::actor buildNode(caf::actor_system& system, std::string_view name, const HubConfig& config);
   //单例模式
    static NodeFactory& get() {
        static NodeFactory instance;
        return instance;
    }
};
```

## Atom

```C++
/*为一个结构体,用于作为标识符传递消息
例如：CAF_ADD_ATOM(ArtinxHub, image_frame_atom)
该宏展开为：
struct image_frame_atom{};
static constexpr image_frame_atom image_frame_atom_v = image_frame_atom{};
...
*/
#  define CAF_ADD_ATOM(...)                                                    \
    CAF_PP_OVERLOAD(CAF_ADD_ATOM_, __VA_ARGS__)(__VA_ARGS__)
#endif 
```

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

## parseSucceed

```C++
   /**
   * @brief				根据config文件解析actor类中的atom对应的需要发送到的actor
   * @param config 		配置文件
   * @param name		atom的名字
   * @retrun			需要发送到的actor的名称
   **/
std::vector<std::string> parseSucceed(const HubConfig& config, std::string_view name) {
    std::string_view nameNormalized = name;
    //由于name由typeid.name(atom) 传入， name会带上struct关键，所以调用demangle函数去掉
    demangle(nameNormalized);
    const auto attr = config.to_dictionary().value();
    //根据atom名称在config文件中查找
    const auto iter = attr.find(nameNormalized);
    if(iter == attr.cend()) {
        return {};
    }

    const auto succeed = iter->second.to_list().value();
    std::vector<std::string> res;
    res.reserve(succeed.size());
    //将config文件中的发送到目标actor的名称以字符串形式保存
    for(const auto& id : succeed) {
        res.push_back(caf::to_string(id));
    }
    return res;
}

   /**
   * @brief				根据actor名称返回actor的地址和组码
   * @param string		actor的名称
   * @retrun			acotr的地址和组码
   **/
    std::vector<std::pair<caf::actor_addr, GroupMask>> parseSucceed(caf::actor_system& system, const std::vector<std::string>& succeed) {
        const auto& registry = system.registry();
        std::vector<std::pair<caf::actor_addr, GroupMask>> res;
        res.reserve(succeed.size());
        for(const auto& id : succeed) {
            if(const auto addr = registry.get<caf::actor_addr>(id))
                res.emplace_back(addr, maskLUT[id]);
            else {
                logError("Undefined actor " + id + " (call sendAll before start_atom?)");
            }
        }
        return res;
    }
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
+ `std::decay_t`
    + 返回去除`cv`属性的类型

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

   /**
   * @brief				得到atom要发送的目标actor
   * @retrun			返回一个std::vector<std::pair<caf::actor_addr, GroupMask>>> 类型的值	
   */    
    template <typename Atom>
    const auto& getDest() {
        //从mDest中拿到要发送的actor的名称
        auto& dest = std::get<SucceedAddress<Atom>>(mDest).val;
   		//如果dest为std::variant中的std::vector<std::string>类型，则将其变为std::vector<std::pair<caf::actor_addr, GroupMask>>类型     
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
    HubHelper(caf::actor_config& base, const HubConfig& config, std::string name)
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
   * @brief				给对应的actor发送atom，从而触发相应lambada函数调用
   * @param atom		需要发送的atom
   * @param args		其他参数
   */
    template <typename Atom, typename... Args>
    void sendAll(Atom atom, Args&&... args) {
        ACTOR_PROTOCOL_CHECK(Atom, std::decay_t<Args>...);
        for(auto&& [address, mask] : getDest<Atom>())
            this->send(caf::actor_cast<caf::actor>(address), atom, wrap(std::forward<Args>(args))...);
    }
    /**
   * @brief				当目标actor的mask和mask相同时， 给目标actor发送atom，从而触发相应lambada函数调用
   * @param atom		需要发送的atom
   * @param mask		目标acotr的mask
   * @param args		其他参数
   */
    template <typename Atom, typename... Args>
    void sendMasked(Atom atom, GroupMask mask, Args&&... args) {
        ACTOR_PROTOCOL_CHECK(Atom, std::decay_t<Args>...);
        for(auto&& [address, maskRhs] : getDest<Atom>())
            //检查目标acotr的mask是否和mask一致
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
   * @param key   		每个actor类实例的对象对应的整形值
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

# ACTOR_PROTOCOL_DEFINE

+ **该宏定义atom与发atom时需要从`blackboard`上拿的对应的结构体类型**

+ 使用方法`ACTOR_PROTOCOL_DEFINE(atom, typename)`，`atom`为一个结构体类型，`typename`为与之对应的结构体类

    ```c++
    //该宏定义了一个模板类，该模板类相当于
    struct __ImplActorProtocol<atom, typename> final{
        static constexpr bool check() noexcept{
            return true;
        }
    }
    #define ACTOR_PROTOCOL_DEFINE(...)                  \
        template <>                                     \
        struct __ImplActorProtocol<__VA_ARGS__> final { \
            static constexpr bool check() noexcept {    \
                return true;                            \
            }                                           \
        }
    ```

# ACTOR_PROTOCOL_CHECK

+ 该宏检查`atom`与参数类型是否对应

```c++
//该宏调用 __ImplActorProtocol<Args...>:：check()进行判断
#define ACTOR_PROTOCOL_CHECK(...) static_assert(__impl_actor_protocol_call<__VA_ARGS__>(), "Mismatched protocol")

template <typename... Args>
constexpr bool __impl_actor_protocol_call() noexcept {
    return __ImplActorProtocol<Args...>::check();
}

//没有经过__ACTOR_PROTOCOL_DEFINE__定义的其他类型参数传入时，利用模板偏特化技术，会匹配到该类，从而返回false
template <typename... T>
struct __ImplActorProtocol final {
    static constexpr bool check() noexcept {
        return false;
    }
};
```

# ACTOR_EXCEPTION_PROBE

+ `__FILE__`: 当前源文件名
+ `__LINE_`: 当前程序行的行号
+ `__FUNCTION__`:当前函数的函数名
+ `feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW)`
    + 该函数使能浮点数异常检查功能
    + `FE_DIVBYZERO `表示被0除的异常
    + `FE_INVALID`表示不合法的浮点运算
    + `FE_OVERFLOW`表示浮点数溢出

+ `std::uncaught_exceptions`

    + 检测当前线程中是否有活动异常对象，及异常已抛出或重新抛出，并且尚未输入匹配的`CATHCH`字句

+ `__builtin_trap`

    + 本质上通过执行非法命令来中止程序。

      ```css
  __builtin_trap function causes the program to exit abnormally. GCC implements this function by using a
  target-dependent mechanism (such as intentionally executing an illegal instruction) or by calling abort. The mechanism
  used may vary from release to release so you should not rely on any particular implementation.
  ```

```c++
//定义ExceptionProbe __probe{}类
#define ACTOR_EXCEPTION_PROBE()          \
    ExceptionProbe __probe {             \
        __FILE__, __FUNCTION__, __LINE__ \
    }

void installFPEProbe() {
#ifdef ARTINXHUB_DEBUG
#ifdef ARTINXHUB_WINDOWS
    _control87(_EM_DENORMAL | _EM_INEXACT | _EM_UNDERFLOW, _MCW_EM);
#else
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif
#endif
}
void uninstallFPEProbe();

class ExceptionProbe final {
    const char* mFile;
    const char* mFunction;
    const uint32_t mLine;

    static constexpr auto highLatency = 50ms;
    Clock::time_point mStart;

public:
    ExceptionProbe(const char* file, const char* function, const uint32_t line)
        : mFile{ file }, mFunction{ function }, mLine{ line }, mStart{ Clock::now() } {
#ifdef ARTINXHUB_DEBUG
        installFPEProbe();
#endif
    }
    ExceptionProbe(const ExceptionProbe& rhs) = delete;
    ExceptionProbe& operator=(const ExceptionProbe& rhs) = delete;
    ExceptionProbe(ExceptionProbe&& rhs) = delete;
    ExceptionProbe& operator=(ExceptionProbe&& rhs) = delete;

    ~ExceptionProbe() {
#ifdef ARTINXHUB_DEBUG
        uninstallFPEProbe();

        if(std::uncaught_exceptions()) {
#ifdef ARTINXHUB_WINDOWS
            __debugbreak();
#else
            __builtin_trap();
#endif
        }
#else
        if(Clock::now() - mStart > highLatency) {
            logWarning(fmt::format("High latency detected {} {} {}", mFile, mFunction, mLine));
        }
#endif
    }
};
```


## SolvePnP

+ *Reference*:[OpenCV: Perspective-n-Point (PnP) pose computation](https://docs.opencv.org/3.4/d5/d1f/calib3d_solvePnP.html)
+ *Reference*:https://www.cnblogs.com/singlex/category/911880.html

