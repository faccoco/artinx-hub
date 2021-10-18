+ 所有类型名使用大驼峰ThisIsType，宏使用大写+下划线THIS_IS_MACRO，其余使用小驼峰thisIsVariable
+ 使用根目录下的.clang-format文件格式化代码
+ 未经允许禁止引入新的第三方库
+ 使用C++17标准，不在使用宏支持跨平台的情况下使用编译器/操作系统相关的代码
+ 未经允许禁止引入新的单例
+ 在vector大小已知时使用resize/reserve预分配空间
+ 使用智能指针，一般情况下不允许出现任何形式的new/delete
+ 使用函数/模板重用代码
+ 仅允许Resharper C++和clang的linter标记
+ 按值传递所有权，其余情况一遍按const引用传递参数。拥有SSO优化的string二者均可。
+ 未经允许禁止更改公共API
+ 尽量使用Transform.hpp提供的编译期量纲分析和参考系检查的Point/Vector/Normal/Transform，不直接使用glm库
+ 使用Identifier和BlackBoard系统传递大型结构体
+ 禁止使用C动态内存管理和字符串API
+ 使用SynchronizedClock作为同步系统时钟
+ 使用Clock获取系统时钟的相关类型信息
+ 未经允许禁止添加新的atom

