# 关于git仓库清理
- 由于此前不小心上传了大模型到git仓库导致现在克隆极慢，现在需要执行以下操作来清理你当前所有工作的分支:
首先删除 `./data/weights/radar_detect` 文件夹
- 方法一：参见该[链接](https://stackoverflow.com/a/17890278),比下面的好处是能保留每个commit
- 方法二:
1. 在你放代码的文件夹执行`git clone https://mirrors.sustech.edu.cn/git/artinx/artinx-hub.git --branch develop --single-branch <folder>`, folder 为你要保存的文件夹名
2. 进入刚刚创建的文件夹，执行`git checkout -b <branch name>`, <branch name>为你要提交的分支名
3. 在你原来的代码文件夹下`cp -r .git ../git-bp`备份你原来的.git文件夹
4. `rm -rf .git&cp -r ../<folder>/.git`将新的.git文件夹拷贝过来
5. `git commit`
6. `git push --set-upstream origin <branch name> -f`强制提交 
## 影响
- 你将保留当前分支所有代码但是损失当前分支从 develop 迁出后的所有 commit 历史记录而合并为一个 commit (方法一不会损失而是重写所有 commits)
- 你需要为所有你维护的分支执行该操作
- 当所有分支执行完该操作后，仓库大小应降至200M左右

