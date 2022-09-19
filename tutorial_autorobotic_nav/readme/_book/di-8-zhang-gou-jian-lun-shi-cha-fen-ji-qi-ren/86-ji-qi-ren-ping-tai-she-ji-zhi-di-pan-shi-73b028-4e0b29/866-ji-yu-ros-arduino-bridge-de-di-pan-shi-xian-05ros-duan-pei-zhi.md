### 8.5.4 控制系统实现\_安装ros\_arduino\_bridge

如果你已经搭建并测试通过了分布式环境，下一步，就可以将ros\_arduino\_bridge功能包上传至树莓派，并在PC端通过键盘控制小车的运动了，实现流程如下:

1. 系统准备；
2. 程序修改；
3. 从PC端上传程序至树莓派；
4. 分别启动PC与树莓派端相关节点，并实现运动控制。

#### 1.系统准备

ros\_arduino\_bridge是依赖于python-serial功能包的，请先在树莓派端安装该功能包，安装命令:

```
$ sudo apt-get install python-serial
```

或

```
$ sudo pip install --upgrade pyserial
```

或

```
$ sudo easy_install -U pyserial
```

#### 2.程序修改

ros\_arduino\_bridge的ROS端功能包主要是使用 ros\_arduino\_python，程序入口是该包launch目录下的arduino.launch文件，内容如下:

```xml
<launch>
   <node name="arduino" pkg="ros_arduino_python" type="arduino_node.py" output="screen">
      <rosparam file="$(find ros_arduino_python)/config/my_arduino_params.yaml" command="load" />
   </node>
</launch>
```

需要载入yaml格式的配置文件，该文件在 config 目录下已经提供了模板，只需要复制文件并按需配置即可，复制文件并重命名，配置如下：

```yaml
# For a direct USB cable connection, the port name is typically
# /dev/ttyACM# where is # is a number such as 0, 1, 2, etc
# For a wireless connection like XBee, the port is typically
# /dev/ttyUSB# where # is a number such as 0, 1, 2, etc.

port: /dev/ttyACM0 #视情况设置，一般设置为 /dev/ttyACM0 或 /dev/ttyUSB0
baud: 57600 #波特率
timeout: 0.1 #超时时间

rate: 50
sensorstate_rate: 10

use_base_controller: True  #启用基座控制器
base_controller_rate: 10   

# For a robot that uses base_footprint, change base_frame to base_footprint
base_frame: base_footprint #base_frame 设置

# === Robot drivetrain parameters
wheel_diameter: 0.065 #车轮直径
wheel_track: 0.21 #轮间距
encoder_resolution: 3960#编码器精度(一圈的脉冲数 * 倍频 * 减速比)
#gear_reduction: 1 #减速比
#motors_reversed: False #转向取反

# === PID parameters PID参数，需要自己调节
Kp: 5
Kd: 45
Ki: 0
Ko: 50
accel_limit: 1.0

# === Sensor definitions.  Examples only - edit for your robot.
#     Sensor type can be one of the follow (case sensitive!):
#      * Ping
#      * GP2D12
#      * Analog
#      * Digital
#      * PololuMotorCurrent
#      * PhidgetsVoltage
#      * PhidgetsCurrent (20 Amp, DC)



sensors: {
  #motor_current_left:   {pin: 0, type: PololuMotorCurrent, rate: 5},
  #motor_current_right:  {pin: 1, type: PololuMotorCurrent, rate: 5},
  #ir_front_center:      {pin: 2, type: GP2D12, rate: 10},
  #sonar_front_center:   {pin: 5, type: Ping, rate: 10},
  arduino_led:          {pin: 13, type: Digital, rate: 5, direction: output}
}
```

#### 3.程序上传

请先在树莓派端创建工作空间，在PC端进入本地工作空间的src目录，调用程序上传命令:

```
scp -r ros_arduino_bridge/ 树莓派用户名@树莓派ip:~/工作空间/src
```

在树莓派端进入工作空间并编译:

```
catkin_make
```

#### 4.测试

现启动树莓派端程序，再启动PC端程序。

**树莓派端**

启动 ros\_arduino\_bridge 节点:

```
roslaunch ros_arduino_python arduino.launch
```

**PC端**

启动键盘控制节点：

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

如无异常，现在就可以在PC端通过键盘控制小车运动了，并且PC端还可以使用rviz查看小车的里程计信息。

