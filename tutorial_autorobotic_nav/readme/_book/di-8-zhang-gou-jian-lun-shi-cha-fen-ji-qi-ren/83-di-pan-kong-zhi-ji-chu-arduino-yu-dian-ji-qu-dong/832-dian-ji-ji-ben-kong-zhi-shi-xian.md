### 8.3.2 电机基本控制实现

在ROS智能车中，控制车辆的前进、后退以及速度调节，那么就涉及到电机的转向与转速控制，本节主要就是介绍相关知识点。

**需求:**控制单个电机转动，先控制电机以某个速率正向转动N秒，再让电机停止N秒，再控制电机以某个速率逆向转动N秒，最后让电机停止N秒，如此循环。

**实现流程:**

1. 编写Arduino程序，setup中设置引脚模式，loop中控制电机运动；
2. 上传并查看运行结果。

#### 1.编码

前提知识点：

1. 左电机的M1与M2对应的是引脚4\(DIRA\)和引脚5\(PWMA\)，引脚4控制转向，引脚5输出PWM。右电机的M1与M2对应的是引脚6\(PWMB\)和引脚7\(DIRB\)，引脚7控制转向，引脚6输出PWM。
2. 可以通过PWM控制电机转速。

**代码:**

```cpp
/*
 * 电机转动控制
 * 1.定义接线中电机对应的引脚
 * 2.setup 中设置引脚为输出模式
 * 3.loop中控制电机转动
 * 
 */

int DIRA = 4;
int PWMA = 5;

void setup() {
  //两个引脚都设置为 OUTPUT
  pinMode(DIRA,OUTPUT);
  pinMode(PWMA,OUTPUT);
}

void loop() {
  //先正向转动3秒
  digitalWrite(DIRA,HIGH);
  analogWrite(PWMA,100);
  delay(3000);
  //停止3秒
  digitalWrite(DIRA,HIGH);
  analogWrite(PWMA,0);
  delay(3000);
  //再反向转动3秒
  digitalWrite(DIRA,LOW);
  analogWrite(PWMA,100);
  delay(3000);
  //停止3秒
  digitalWrite(DIRA,LOW);
  analogWrite(PWMA,0);
  delay(3000);

  /*
   * 注意: 
   * 1.可以通过将DIRA设置为HIGH或LOW来控制电机转向，但是哪个标志位正转或反转需要根据需求判断，转向是相对的。
   * 2.PWM的取值为 [0,255],该值可自己设置。
   * 
   */

}
```

#### 2.运行

程序上传到Arduino上，如无异常，电机开始转动，转动结果与需求描述类似。

