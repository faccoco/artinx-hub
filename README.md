# ARTINX-HUB

Artinx视觉组 集成框架

## 入门

没有速成，速成的都是垃圾

[CheckList](docs/checklist.md)

### 计算机基础

推荐书籍（按照难度排序）：
+ Computer Systems: A Programmer's Perspective (CSAPP)
+ 计算机程序的构造和解释（SICP）
+ 计算机组成与设计：硬件/软件接口
+ 操作系统概念（恐龙书）
+ 程序员修炼之道2：通向务实的最高境界
+ 编译原理（龙书）

### C++

基础
+ C++ Primer(Plus)
+ [于仕琪老师的Bilibili网课](https://www.bilibili.com/video/BV1Vf4y1P7pq)
+ C++ Programming Language

进阶
+ Effective C++
+ More Effective C++
+ Effective Modern C++
+ Modern C++ Tutorial: C++11/14/17/20 On the Fly
+ https://github.com/AnthonyCalandra/modern-cpp-features
+ http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines
+ [小彭老师的Bilibili公开课](https://space.bilibili.com/263032155/channel/collectiondetail?sid=53025)

骨灰
+ https://www.youtube.com/user/CppCon
+ https://www.open-std.org/jtc1/sc22/wg21/docs/papers/
+ http://purecpp.org/

### Linux/Git/Shell

+ https://git-scm.com/docs/user-manual
+ https://linuxtools-rst.readthedocs.io/zh_CN/latest/

## 快速跳转

+ [cppreference](https://en.cppreference.com/w/)
+ [glm manual](https://github.com/g-truc/glm/blob/master/manual.md)
+ [OpenCV doc](https://docs.opencv.org/4.x/)
+ [规则手册](https://www.robomaster.com/zh-CN/resource/pages/announcement/1370)

## 开发规范

### 代码规范
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
+ 所有Atom必须使用**ACTOR_PROTOCOL_CHECK**和**ACTOR_PROTOCOL_DEFINE**检查参数类型

### Commit规范
+ commit信息应遵循Angular规范，建议使用VSCode的插件commitizen
+ 每个commit的更改内容应该尽可能保持小范围且集中
+ 尽可能确保commit时源码能够正常编译，正常运行，通过测试
+ 未经允许禁止提交二进制文件，禁止提交个人配置文件
+ 一切新功能代码修改均在feature-（小写和-组成）分支下进行，一切bug修复均在hotfix-分支下进行，在review通过后，需向develop分支发起**merge request**，由管理员merge至develop分支
+ develop分支的功能稳定后，将由管理员merge至main分支，非管理员无法直接对develop分支和main分支做修改

+ 工作流样例：

  ```bash
  git branch <branch name>
  git checkout branch
  # do some modifications
  git add .
  git commit -m "<message>"
  git push -u origin <branch name>
  ```

### GitLab工作流

1. 某人发起新的issue对应新的功能/bug修复，此时新的补充或纠正等讨论内容发在issue上
2. 管理员指定给某人在该issue上工作，创建新的merge request以跟踪进度
3. assignee完成工作后push代码，由reviewer审核代码，完成审核后由管理员merge入develop分支
4. merge完成后issue被自动关闭

## 仓库目录结构

```
.
|-- bringup_templates 自启动/自动部署脚本
|-- config_templates actor使用示例
|-- data 数据
    |-- camera_calibration 相机标定结果
    |-- weights 神经网络/SVM 权重文件
|-- deploy_config 机器人部署配置
|-- docs 次级文档
|-- include 头文件
|-- pages 可视化网页文件
|-- scripts 脚本
|-- src 源代码
|-- tests 单元测试文件
|-- .clang-format C/C++格式化配置文件
|-- .gitattributes
|-- .gitignore
|-- .gitlab-ci.yml GitLab CI配置文件
|-- CMakeLists.txt CMake根目录配置文件
|-- CMakeSettings.json Visual Studio 2019 CMake配置文件
|-- Folder.DotSettings Resharper++ Lint配置文件
|-- README.md 自述文件
```

## 本地构建指南

### Windows

### Linux

### Genetic

## 机器人部署指南

环境配置

CI配置

## CI持续部署指南

### 自启动

### 自动部署

## 框架工具使用指南

actor

atom

probe
