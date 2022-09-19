## 8.2 机器人平台设计之arduino基础

在构建差分轮式机器人平台时，驱动系统的常用实现有 STM32 或 Arduino，在此，我们选用后者，因为 Arduino 相较而言更简单、易于上手。本节将介绍如下内容:

* arduino 简介
* arduino 开发环境搭建
* arduino 基本语法

---

#### 概念

Arduino是一款便捷灵活、方便上手的开源电子原型平台。在它上面可以进行简单的电路控制设计，Arduino能够通过各种各样的传感器来感知环境，通过控制灯光、马达和其他的装置来反馈、影响环境。

#### 作用

或多或少你可能听说过“集成电路”\(又称“微电路”、“微芯片”或“芯片”\)这种概念，集成电路（integrated circuit）是一种微型电子器件或部件，通过集成电路再结合一些外围的电子电子元器件、传感器等，可以感知环境\(温度、湿度、声音\)，也可以影响环境\(控制灯的开关、调节电机转速\)。但是传统的集成电路应用比较繁琐，一般需要具有一定电子知识基础，并懂得如何进行相关的程序设计的工程师才能熟练使用，而Arduino的出现才使得以往高度专业的集成电路变得平易近人，Arduino主要优点如下:

* **简单:**在硬件方面，Arduino本身是一款非常容易使用的印刷电路板。电路板上装有专用集成电路，并将集成电路的功能引脚引出方便我们外接使用。同时，电路板还设计有USB接口方便与电脑连接。
* **易学:**只需要掌握 C/C++ 基本语法即可
* **易用:**Arduino提供了专门的程序开发环境Arduino IDE，可以提高程序实现效率。

当前，Arduino已经成为全世界电子爱好者电子制作过程中的重要选项之一。

#### 组成

Arduino 体系主要包含硬件和软件两大部分。硬件部分是可以用来做电路连接的各种型号的Arduino电路板；软件部分则是Arduino IDE。你只要在IDE中编写程序代码，将程序上传到Arduino电路板后，程序便会告诉Arduino电路板要做些什么了。

![](/assets/arduino.jfif)

![](/assets/ArduinoMega2560.jfif)
