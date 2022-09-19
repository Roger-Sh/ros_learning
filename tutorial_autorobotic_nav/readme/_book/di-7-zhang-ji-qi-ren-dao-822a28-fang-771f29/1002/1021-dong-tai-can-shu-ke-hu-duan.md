### 10.2.1 动态参数客户端

**需求:**

> 编写两个节点，一个节点可以动态修改参数，另一个节点时时解析修改后的数据。

**客户端实现流程:**

* 新建并编辑 .cfg 文件;
* 编辑CMakeLists.txt;
* 编译。

---

#### 1.新建功能包 {#1新建功能包，注意添加依赖包-dynamicreconfigure}

新建功能包，添加依赖:`roscpp rospy std_msgs dynamic_reconfigure`。

#### 2.添加.cfg文件 {#2新建-cfg-文件夹，添加-xxxcfg-文件并添加可执行权限}

新建 cfg 文件夹，添加 xxx.cfg 文件\(并添加可执行权限\)，cfg 文件其实就是一个 python 文件,用于生成参数修改的客户端\(GUI\)。

```py
#! /usr/bin/env python
"""
 4生成动态参数 int,double,bool,string,列表
 5实现流程:
 6    1.导包
 7    2.创建生成器
 8    3.向生成器添加若干参数
 9    4.生成中间文件并退出
10
"""
# 1.导包
from dynamic_reconfigure.parameter_generator_catkin import *
PACKAGE = "demo02_dr"
# 2.创建生成器
gen = ParameterGenerator()

# 3.向生成器添加若干参数
#add(name, paramtype, level, description, default=None, min=None, max=None, edit_method="")
gen.add("int_param",int_t,0,"整型参数",50,0,100)
gen.add("double_param",double_t,0,"浮点参数",1.57,0,3.14)
gen.add("string_param",str_t,0,"字符串参数","hello world ")
gen.add("bool_param",bool_t,0,"bool参数",True)

many_enum = gen.enum([gen.const("small",int_t,0,"a small size"),
                gen.const("mediun",int_t,1,"a medium size"),
                gen.const("big",int_t,2,"a big size")
                ],"a car size set")

gen.add("list_param",int_t,0,"列表参数",0,0,2, edit_method=many_enum)

# 4.生成中间文件并退出
exit(gen.generate(PACKAGE,"dr_node","dr"))
```

`chmod +x xxx.cfg`添加权限

#### 3.配置 CMakeLists.txt {#3配置-cmakeliststxt}

```cmake
generate_dynamic_reconfigure_options(
  cfg/mycar.cfg
)
```

#### 4.编译 {#4编译}

编译后会生成中间文件

C++ 需要调用的头文件:

![](/assets/动态参数C++头文件.PNG)

Python需要调用的文件:

![](/assets/动态参数Python文件.PNG)

---



