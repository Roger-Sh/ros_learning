### 8.3.4 电机测速02\_实现

**需求:**统计并输出电机转速。

**思路:**先统计单位时间内以单频或2倍频或4倍频的方式统计脉冲数，再除以一圈对应的脉冲数，最后再除以时间所得即为电机转速。

**核心:**计数时，需要在A相或B相的上升沿或下降沿触发时，实现计数，在此需要使用中断引脚与中断函数。

Arduino Mega 2560 的中断引脚:2 \(interrupt 0\), 3 \(interrupt 1\),18 \(interrupt 5\), 19 \(interrupt 4\), 20 \(interrupt 3\), 21 \(interrupt 2\)

**实现流程:**

1. 编写Arduino程序先实现脉冲数统计；
2. 编写Arduino程序再实现转速计算相关实现；
3. 上传到Arduino并测试。

#### 1.编码实现脉冲统计

核心知识点:**attachInterrupt\(\)函数**\(请参考 8.2.2 介绍\)。

**代码:**

```cpp
/*
 * 测速实现:
 *  阶段1:脉冲数统计
 *  阶段2:速度计算
 * 
 * 阶段1:
 *  1.定义所使用的中断引脚,以及计数器(使用 volatile 修饰)
 *  2.setup 中设置波特率，将引脚设置为输入模式
 *  3.使用 attachInterupt() 函数为引脚添加中断出发时机以及中断函数
 *  4.中断函数编写计算算法，并打印
 *    A.单频统计只需要统计单相上升沿或下降沿
 *    B.2倍频统计需要统计单相的上升沿和下降沿
 *    C.4倍频统计需要统计两相的上升沿和下降沿
 *  5.上传并查看结果
 *  
 * 
 */
int motor_A = 21;//中端口是2
int motor_B = 20;//中断口是3
volatile int count = 0;//如果是正转，那么每计数一次自增1，如果是反转，那么每计数一次自减1 


void count_A(){
  //单频计数实现
  //手动旋转电机一圈，输出结果为 一圈脉冲数 * 减速比
  /*if(digitalRead(motor_A) == HIGH){

    if(digitalRead(motor_B) == LOW){//A 高 B 低
      count++;  
    } else {//A 高 B 高
      count--;  
    }


  }*/

  //2倍频计数实现
  //手动旋转电机一圈，输出结果为 一圈脉冲数 * 减速比 * 2
  if(digitalRead(motor_A) == HIGH){

    if(digitalRead(motor_B) == HIGH){//A 高 B 高
      count++;  
    } else {//A 高 B 低
      count--;  
    }


  } else {

    if(digitalRead(motor_B) == LOW){//A 低 B 低
      count++;  
    } else {//A 低 B 高
      count--;  
    }  

  }

}

//与A实现类似
//4倍频计数实现
//手动旋转电机一圈，输出结果为 一圈脉冲数 * 减速比 * 4
void count_B(){
  if(digitalRead(motor_B) == HIGH){

    if(digitalRead(motor_A) == LOW){//B 高 A 低
      count++;
    } else {//B 高 A 高
      count--;
    }


  } else {

    if(digitalRead(motor_A) == HIGH){//B 低 A 高
      count++;
    } else {//B 低 A 低
      count--;
    }

  }

}

void setup() {
  Serial.begin(57600);//设置波特率  
  pinMode(motor_A,INPUT);
  pinMode(motor_B,INPUT);
  attachInterrupt(2,count_A,CHANGE);//当电平发生改变时触发中断函数
  //四倍频统计需要为B相也添加中断
  attachInterrupt(3,count_B,CHANGE);
}


void loop() {
  //测试计数器输出
  delay(2000);
  Serial.println(count);

}
```

#### 2.转速计算

思路:需要定义一个开始时间\(用于记录每个测速周期的开始时刻\)，还需要定义一个时间区间\(比如50毫秒\)，时时获取当前时刻，当当前时刻 - 上传结束时刻 &gt;= 时间区间时，就获取当前计数并根据测速公式计算时时速度，计算完毕，计数器归零，重置开始时间

核心知识点:当使用中断函数中的变量时，需要先禁止中断**noInterrupts\(\)**，调用完毕，再重启中断**interrupts\(\)**\(关于noInterrupts与interrupts请参考 8.2.2 介绍\)。

**代码\(核心\):**

2中代码除了 loop 实现，无需修改。

```cpp
int reducation = 90;//减速比，根据电机参数设置，比如 15 | 30 | 60
int pulse = 11; //编码器旋转一圈产生的脉冲数该值需要参考商家电机参数
int per_round = pulse * reducation * 4;//车轮旋转一圈产生的脉冲数 
long start_time = millis();//一个计算周期的开始时刻，初始值为 millis();
long interval_time = 50;//一个计算周期 50ms
double current_vel;

//获取当前转速的函数
void get_current_vel(){
  long right_now = millis();  
  long past_time = right_now - start_time;//计算逝去的时间
  if(past_time >= interval_time){//如果逝去时间大于等于一个计算周期
    //1.禁止中断
    noInterrupts();
    //2.计算转速 转速单位可以是秒，也可以是分钟... 自定义即可
    current_vel = (double)count / per_round / past_time * 1000 * 60;
    //3.重置计数器
    count = 0;
    //4.重置开始时间
    start_time = right_now;
    //5.重启中断
    interrupts();

    Serial.println(current_vel);

  }
}

void loop() {

  delay(10);
  get_current_vel();

}
```

#### 3.测试

将代码上传至Arduino，打开出口监视器，手动旋转电机，可以查看到转速信息。

