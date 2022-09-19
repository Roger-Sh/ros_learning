### 1.2.4 安装 ROS

Ubuntu 安装完毕后，就可以安装 ROS 操作系统了，大致步骤如下:

1. 配置ubuntu的软件和更新；

2. 设置安装源；

3. 设置key；

4. 安装；

5. 配置环境变量。

---

#### 1.配置ubuntu的软件和更新

配置ubuntu的软件和更新，允许安装不经认证的软件。

首先打开“软件和更新”对话框，具体可以在 Ubuntu 搜索按钮中搜索。

打开后按照下图进行配置（确保勾选了"restricted"， "universe，" 和 "multiverse."）

![](/assets/00ROS安装之ubuntu准备.png "00ROS安装之ubuntu准备")

#### 2.设置安装源

官方默认安装源:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

或来自国内清华的安装源

    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

或来自国内中科大的安装源

    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

PS:

1. 回车后,可能需要输入管理员密码
2. 建议使用国内资源，安装速度更快。

#### 3.设置key

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

#### 4.安装

首先需要更新 apt\(以前是 apt-get, 官方建议使用 apt 而非 apt-get\),apt 是用于从互联网仓库搜索、安装、升级、卸载软件或操作系统的工具。

```
sudo apt update
```

等待...

然后，再安装所需类型的 ROS:ROS 多个类型:**Desktop-Full**、**Desktop**、**ROS-Base**。这里介绍较为常用的Desktop-Full\(官方推荐\)安装: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception

```
sudo apt install ros-noetic-desktop-full
```

等待......\(比较耗时\)

友情提示: 由于网络原因,导致连接超时，可能会安装失败，如下所示:![](/assets/09_安装异常.PNG "09\_安装异常")可以多次重复调用 更新 和 安装命令，直至成功。

#### 5.配置环境变量

配置环境变量，方便在任意 终端中使用 ROS。

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

#### 卸载

如果需要卸载ROS可以调用如下命令:

```
sudo apt remove ros-noetic-*
```

注意: 在 ROS 版本 noetic 中无需构建软件包的依赖关系，没有`rosdep`的相关安装与配置。

---

另请参考：[http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)。

---

---

---

### 后记

#### 6.安装构建依赖

在 noetic 最初发布时，和其他历史版本稍有差异的是:没有安装构建依赖这一步骤。随着 noetic 不断完善，官方补齐了这一操作。

首先安装构建依赖的相关工具

```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

ROS中使用许多工具前，要求需要初始化rosdep\(可以安装系统依赖\) -- 上一步实现已经安装过了。

```
sudo apt install python3-rosdep
```

初始化rosdep

```
sudo rosdep init
rosdep update
```

如果一切顺利的话，rosdep 初始化与更新的打印结果如下:

![](/assets/rosdep正常初始化.PNG)

![](/assets/rosdep正常更新.PNG)

---

但是，在 rosdep 初始化时，多半会抛出异常。

**问题:**![](/assets/noetic异常提示.PNG)

**原因:**

境外资源被屏蔽。

**解决:**

百度或google搜索，解决方式有多种\([https://github.com/ros/rosdistro/issues/9721](https://github.com/ros/rosdistro/issues/9721)\)，可惜在 ubuntu20.04 下，集体失效。

新思路:_将相关资源备份到 gitee,修改 rosdep 源码,重新定位资源。_

**实现:**

1.先打开资源备份路径:[https://gitee.com/zhao-xuzuo/rosdistro](https://gitee.com/zhao-xuzuo/rosdistro)，打开 rosdistro/**rosdep**/**sources.list.d**/**20-default.list**文件留作备用\(主要是复用URL的部分内容:gitee.com/zhao-xuzuo/rosdistro/raw/master\)。

![](/assets/gitee资源.PNG)

2.进入"/usr/lib/python3/dist-packages/" 查找rosdep中和`raw.githubusercontent.com`相关的内容，调用命令:

```
find . -type f | xargs grep "raw.githubusercontent"
```

![](/assets/noetic_查找包含githubusercontent的文件.PNG)

3.修改相关文件，主要有: ./rosdistro/\_\_init\_\_.py、./rosdep2/gbpdistro\_support.py、./rosdep2/sources\_list.py 、./rosdep2/rep3.py。可以使用`sudo gedit`命令修改文件:

文件中涉及的 URL 内容，如果是:`raw.githubusercontent.com/ros/rosdistro/master`都替换成步骤1中准备的`gitee.com/zhao-xuzuo/rosdistro/raw/master`即可。

修改完毕，再重新执行命令:

```
sudo rosdep init
rosdep update
```

就可以正常实现 rosdep 的初始化与更新了。

---



