# ROS_Learning

## 介绍

本仓库为ROS学习相关仓库. 各文件夹对应的学习内容介绍如下:

-   ros_book/
    -   ROS相关电子书
-   tutorial_autorobotic_nav/
    -   autorobotic 出品的 ROS 移动机器人导航相关学习
-   tutorial_deepblue_moveit/
    -   deepblue 出品的 ROS MoveIt! 机械臂控制规划相关的学习
-   tutorial_ros_official/
    -   ros 官方教程的学习demo





## ROS + VSCode 调试

-   准备工作

    -   vscode 安装插件

        -   ROS插件
        -   c/c++, c++ intellisense

    -   `CMakeLists.txt`中需要添加

        -   ```Cmake
            # 输出编译相关信息，方便找到include文件
            set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
            # debug模式
            set(CMAKE_BUILD_TYPE "RelWithDebInfo")
            
            # release需要被注释，不然无法调试
            #set(CMAKE_BUILD_TYPE Release)
            ```

    -   vscode `c_cpp_properties.json` 添加

        -   ```json
            "compileCommands": "${workspaceFolder}/build/compile_commands.json"
            ```

    -   vscode `launch.json` 添加 `GDB`， `Launch` 以及 `Attack` 模式

        -   ```json
            {
                // 使用 IntelliSense 了解相关属性。 
                // 悬停以查看现有属性的描述。
                // 欲了解更多信息，请访问: https://go.microsoft.com/fwlink/?linkid=830387
                "version": "0.2.0",
                "configurations": [
                    
                    
                    // GDB模式，需要在program中指定debug的程序
                    {
                        "name": "(gdb) 启动",
                        "type": "cppdbg",
                        "request": "launch",
                        "program": "/home/shan/Documents/ros_learning/tutorial_ros_official/devel/lib/beginner_tutorials/talker",
                        "args": [],
                        "stopAtEntry": false,
                        "cwd": "${fileDirname}",
                        "environment": [],
                        "externalConsole": false,
                        "MIMode": "gdb",
                        "setupCommands": [
                            {
                                "description": "为 gdb 启用整齐打印",
                                "text": "-enable-pretty-printing",
                                "ignoreFailures": true
                            },
                            {
                                "description": "将反汇编风格设置为 Intel",
                                "text": "-gdb-set disassembly-flavor intel",
                                "ignoreFailures": true
                            }
                        ]
                    },
            
                
                    {
                        "name": "ROS: Launch",
                        "type": "ros",
                        "request": "launch",
                        "target": "absolute path to launch file"
                    },
                    
                    // ROS Attach模式调试，无需提前指定节点，但需要在debug时先开启节点，debug时选择正在运行的节点
                    {
                        "name": "ROS: Attach",
                        "type": "ros",
                        "request": "attach"
                    }
            
                ]
            }
            ```

-   单节点 attach调试
    -   catkin_make编译，需要注意编译的模式是否为 `RelWithDebInfo`
    -   调试流程
        -   程序中设置断点
        -   需要先运行该 ROS 节点，包括 ROS Core
        -   VSCode debug按钮选项中选择ROS: Attach模式，搜索节点名称进入attach调试模式
-   单节点 GDB调试
    -   catkin_make编译，需要注意编译的模式是否为 `RelWithDebInfo`
    -   调试流程
        -   程序中设置断点
        -   需要先运行ROS Core,不需要提前运行该ROS节点
        -   在VSCode debug按钮选项中选择gdb模式，直接进入debug过程
-   多节点 launch调试
    -   catkin_make编译，需要注意编译的模式是否为 `RelWithDebInfo`
    -   需要对应的launch文件
    -   调试流程
        -   程序中设置断点
        -   在VSCode debug按钮选项中选择ROS: Launch模式，开始多节点debug

