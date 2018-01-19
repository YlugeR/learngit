#Git
##安装Git
##创建版本库
1. 创建空目录
 '$ mkdir learngit
  $ cd learngit
  $ git init'
2. 提交文件到仓库
 '$ git add readme.txt
  $ git commit -m "wrote a readme file"'
  *or*
 '$ git add file1.txt
  $ git add file2.txt file3.txt
  $ git commit -m "add 3 files."'
##时光穿梭
1. 查看仓库抓状态
 '$ git status'
2. 查看修改内容
 '$ git diff'
3. 历史记录
 'git log'
 *or*
 'git log --pretty=oneline'
4. 回退到上一个版本
 'git reset --hard HEAD^'
5. 回退到某一版本
 'git reset --hard 3628164'
6. 记录命令
 'git reflog'
7. 丢弃工作区的修改
 'git checkout -- file'
8. 把暂存区的修改撤销掉
 'git reset HEAD file'
9. 删除文件
 'rm test.txt'
 *then*
 'git rm test.txt
  git commit -m "remove test.txt"'
  *or*
  'git checkout -- test.txt'
##远程仓库
1. 创建SSH Key
 'ssh-keygen -t rsa -C "youremail@example.com"'
2. 登陆GitHub，然后，在右上角找到“Create a new repo”按钮，创建一个新的仓库,在Repository name填入learngit，其他保持默认设置，点击“Create repository”按钮
3. 从这个仓库克隆出新的仓库，也可以把一个已有的本地仓库与之关联，然后，把本地仓库的内容推送到GitHub仓库。
 'git remote add origin https://github.com/YlugeR/learngit.git
  git push -u origin master'
  *then*
  'git push origin master'
4. 从远程克隆
 'git clone git@github.com:michaelliao/gitskills.git'
##分支管理
