+ commit信息应遵循Angular规范，建议使用VSCode的插件commitizen

+ 每个commit的更改内容应该尽可能保持小范围且集中

+ 尽可能确保commit时源码能够正常编译，正常运行，通过测试

+ 未经允许禁止提交二进制文件，禁止提交个人配置文件

+ 一切新功能代码修改均在feature-（小写和-组成）分支下进行，一切bug修复均在hotfix-分支下进行，在review通过后，需向develop分支发起merge request

+ develop分支的功能稳定后，将被merge至main分支，任何人都无法直接对develop分支和main分支做修改

+ 工作流样例：

  ```bash
  git branch <branch name>
  git checkout branch
  # do some modifications
  git add .
  git commit -m "<message>"
  git push -u origin <branch name>
  ```

  

