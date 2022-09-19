## 3.3 Python模块导入

与C++类似的，在Python中导入其他模块时，也需要相关处理。

**需求:**首先新建一个Python文件A，再创建Python文件UseA，在UseA中导入A并调用A的实现。

**实现:**

1. 新建两个Python文件，使用 import 实现导入关系；
2. 添加可执行权限、编辑配置文件并执行UseA。

---

#### 1.新建两个Python文件并使用import导入

文件A实现\(包含一个变量\):

```py
#! /usr/bin/env python
num = 1000
```

文件B核心实现:

```py
import os
import sys

path = os.path.abspath(".")
# 核心
sys.path.insert(0,path + "/src/plumbing_pub_sub/scripts")

import tools

....
....
    rospy.loginfo("num = %d",tools.num)
```

#### 2.添加可执行权限，编辑配置文件并执行

此过程略。

