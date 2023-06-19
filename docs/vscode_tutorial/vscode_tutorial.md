# vscode工作流（教你用vscode编辑、调试代码）

## ！！！vscode只是一个文本编辑器，他不会编译不会运行更不会调试！！！

### 前言
+ 一个程序要从代码到运行是要经历编译、运行的，vscode就是个高级记事本，要编译、运行得自己写配置文件告诉vscode怎么做（就是下面提到的三个json文件）（c++插件能让你运行单个cpp文件就是因为插件调用了内置了配置文件）  
+ 没装好各种库你用vscode也是编译不了的
+ 此方法不使用vscode中的cmake拓展，使用cmake拓展的方法可自行探索  
+ 你甚至可以vscode远程调试

### 步骤
+ 先照着[README.md](../../README.md#本地构建指南)的教程配，但不用下Clion，Windows的话VS还是要的
+ 下[cmake](https://cmake.org)（可以尝试sudo apt-get install cmake获取，Ubuntu20.04通过apt所获取的最新cmake一般为3.16，可通过命令行输入cmake --version查看，但cmake3.16很可能会报找不到LibArchive::LibArchive）[cmake历史版本](https://cmake.org/files/)
+ 在vscode中下载“C/C++”拓展
+ 把该文件夹下的另外仨json文件（c_cpp_properties.json、tasks.json、launch.json）复制到项目根目录下的".vscode"文件夹里（指ARTINX-HUB/.vscode）（没有就自己建一个，要是有对应文件你想留就备份不想留删了都行反正只是vscode的配置文件而且你都要看这个教程了多半那配置文件也没法让你好好用vscode）（别忘了文件夹名字最前面有个点）
+ 好好看看那仨文件，直接用不大现实，有啥可能要改的注释都写好了，不懂哪个参数是什么意思直接把鼠标放上去vscode有提示
  + c_cpp_properties.json：这是c++拓展自带Intelli Sense(语法检查、自动跳转啥的)的配置文件，配置不好也就语法检查报错、自动跳转无效之类的，配置好了你编译也不一定不报错，和编译没有半毛钱关系（c++自带的这个Intelli Sense又慢又占空间，建议再看看拓展设置，里面有些缓存存哪、占多大的设置改一改比较好（指默认硬盘缓存5G），不如换clangd但这东西的配置方法估计也能写一篇文档了）
  + tasks.json：编译任务写这
  + launch.json：运行任务写这
+ vscode左侧栏“运行和调试”可选运行任务
+ 选好运行任务你就可以直接运行或者调试程序了

### 最后
+ 注意vscode的调试（默认快捷键f5）和运行（默认快捷键ctrl+f5）与你运行的程序为debug版本或release版本没任何关系，只与你运行的任务本身所运行的程序是debug版本还是release版本有关，运行任务设置在launch.json中，运行任务所运行的程序本身是什么版本由编译决定，编译任务设置在tasks.json中  
$\begin{array}{c|cc}
断点 & debug & release \\
\hline
调试 & 可    & 不可 \\
运行 & 不可  & 不可 \\
\end{array}\qquad
\begin{array}{c|cc}
异常退出时暂停 & debug & release \\
\hline
调试          & 可    & 不可 \\
运行          & 可    & 不可 \\
\end{array}$
+ vscode可以只编译不运行（默认快捷键ctrl+shift+b（你要下了cmake拓展这个快捷键可能会被夺舍，左下角齿轮->键盘快捷方式可以改，甚至还能录制按键查找））
+ vscode远程调试就多下个“Remote - SSH”插件，照提示输入目标计算机ip、用户名、密码啥的，固定ip可参照[wired_vnc.md](../wired_vnc.md)，连接成功后点击左侧拓展栏可直接在目标计算机上下载插件，再照着上面教程配就对了
+ 报错找不到LibArchive::LibArchive换官网上3.17及以上版本cmake可以解决，但有的3.16cmake又无问题，原因未知