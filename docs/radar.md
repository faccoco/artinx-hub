# 2023赛季雷达

<!-- vim-markdown-toc GitLab -->

- [Dependencies](#dependencies)
- [如何编译](#如何编译)
- [代码说明](#代码说明)
    - [程序流程](#程序流程)
- [常见问题](#常见问题)
    - [关于模型转换](#关于模型转换)
    - [关于推理](#关于推理)

<!-- vim-markdown-toc -->

## Dependencies

- Nvidia GPU with CUDA support
- [Nvidia TensorRT](https://developer.nvidia.com/tensorrt)
- [Nvidia cuDNN](https://developer.nvidia.com/cudnn)
- [Nvidia CUDA toolkit](https://developer.nvidia.com/cuda-toolkit)
- TensorRT Infer，用[原作者的](https://github.com/shouxieai/infer)或者[我改的cmake版](https://github.com/Vollate/TensorRT-Infer)，只有编译安装上的区别

> 能上包管理器就别手装，ubuntu不好用就上Arch或者Manjaro(AUR万岁)

## 如何编译

1. 设置环境变量`CUDA_PATH TensorRT_PATH TRT_INFER_PATH(就是上面说的要自己编译的推理库)`
2. 定义环境变量`ARTINX_RADAR`，随便什么值定义了就行

> 如果链接infer库的时候寄了，换个编译器，建议infer库和artinxhub使用同一个编译器编译

## 代码说明

```
src/Detect/RadarNNetDetector.cpp        神经网络推理
src/Solver/BotLocator.cpp               解算敌方机器人坐标
src/Solver/RadarLocator.cpp             雷达自身定位及透射变换矩阵获取
src/DataLink/PosSynchronization.cpp     发送给裁判系统
src/DataLink/HttpServer.cpp             http后端，性能太差,建议换库整个重构
```

### 程序流程

看word，懒得放图片了

## 常见问题

### 关于模型转换
- 参考infer库文档，如果`trtexec`运行报错（非找不到动态库）检查显卡驱动，尝试换装dkms版来解决。不行试试nvidia-docker里面能不能跑，能跑就是驱动问题，跑不了就是tensorrt问题，换低版本。
- 一般半精度就行`fp16`

### 关于推理

- 扔exception: 模型位置没写对，模型和gpu型号不符,转换用的tensorrt和运行时的版本不符等等，基本看log就能明白
- 程序成功运行，但是网络无输出，检查模型转换是否正确，尝试用不同精度

