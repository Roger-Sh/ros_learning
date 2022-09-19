### 3.2.1 自定义头文件调用

**需求:**设计头文件，可执行文件本身作为源文件。

**流程:**

1. 编写头文件；
2. 编写可执行文件\(同时也是源文件\)；
3. 编辑配置文件并执行。

---

#### 1.头文件

在功能包下的 include/功能包名 目录下新建头文件: hello.h，示例内容如下:

```cpp
#ifndef _HELLO_H
#define _HELLO_H

namespace hello_ns{

class HelloPub {

public:
    void run();
};

}

#endif
```

**注意:**

在 VScode 中，为了后续包含头文件时不抛出异常，请配置 .vscode 下 c\_cpp\_properties.json 的 includepath属性

```
"/home/用户/工作空间/src/功能包/include/**"
```

#### 2.可执行文件

在 src 目录下新建文件:hello.cpp，示例内容如下:

```cpp
#include "ros/ros.h"
#include "test_head/hello.h"

namespace hello_ns {

void HelloPub::run(){
    ROS_INFO("自定义头文件的使用....");
}

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"test_head_node");
    hello_ns::HelloPub helloPub;
    helloPub.run();
    return 0;
}
```

#### 3.配置文件

配置CMakeLists.txt文件，头文件相关配置如下:

```makefile
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
```

可执行配置文件配置方式与之前一致:

```makefile
add_executable(hello src/hello.cpp)

add_dependencies(hello ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hello
  ${catkin_LIBRARIES}
)
```

最后，编译并执行，控制台可以输出自定义的文本信息。

