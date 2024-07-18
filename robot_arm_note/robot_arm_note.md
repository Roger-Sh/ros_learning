# 机械臂笔记

## 串联机械臂理论简介

- 参考视频
  - [机械臂运动学教程: 机械臂+旋转矩阵+变换矩阵+DH+逆解+轨迹规划](https://www.bilibili.com/video/BV1oa4y1v7TY?p=1&vd_source=e4455003e32fdc49fb1b368513cd60bd)
- 参考书籍
  - 《机器人学导论》，J. J. Craig
  - 《现代机器人学：力学，规划，控制》，K. M. Lynch
  - LUH 机器人学课件

### 机械臂类型

常见的机械臂类型：

- 6自由度串联机械臂

  - 工作空间大
  - 灵活度高
  - 速度一般
  - 精度一般
  - 运动控制复杂

  <img src="robot_arm_note.assets/image-20240507101615856.png" alt="image-20240507101615856" style="zoom: 80%;" />
- 4自由度SCARA机械臂

  - 工作空间大
  - 灵活度一般
  - 速度快
  - 精度高
  - 运动控制相对简单

  <img src="robot_arm_note.assets/image-20240507103705231.png" alt="image-20240507103705231" style="zoom: 80%;" />
- 并联型机械臂

  - 工作空间相对安装空间较小
  - 灵活度低
  - 速度非常快
  - 精度非常高
  - 运动控制相对复杂

  <img src="robot_arm_note.assets/image-20240507111441950.png" alt="image-20240507111441950" style="zoom:80%;" />
- 冗余机械臂

  - 关节数量大于末端的自由度
  - 能提供更多的灵活度，同样的目标位置，可以有不同的关节配置，以达到避障和避免奇异性的问题。
  - 运动控制相比普通6轴机械臂更复杂
  - 例子：KUKA-iiwa 7轴机械臂

    ![image-20240521155139751](robot_arm_note.assets/image-20240521155139751.png)
- 视频资料

  - [机械臂的进化](https://www.zhihu.com/zvideo/1733137345912598528?playTime=133.4)
  - [7轴零空间运动](https://www.bilibili.com/video/BV1rG41147C2?p=9&vd_source=e4455003e32fdc49fb1b368513cd60bd)
  - [并联机械手](https://www.bilibili.com/video/BV1qe4y1q7LA/?spm_id_from=333.337.search-card.all.click&vd_source=e4455003e32fdc49fb1b368513cd60bd)

### 坐标系表达与转换

- 可视化资料
  - [四元数可视化](https://quaternions.online/)
  - [在线旋转表达转换](https://www.andre-gaschler.com/rotationconverter/)
  - [VisRo: Making Robotics Easy to Learn through Visualization](https://vis-ro.web.app/translation)

#### 一些向量计算规则

- 向量内积，可以描述向量间的投影关系

$$
\boldsymbol{a\cdot b} = \boldsymbol{a}^\mathrm{T}\boldsymbol{b} = \sum_{i=1}^{3}a_ib_i = |\boldsymbol{a}||\boldsymbol{b}|\cos<\boldsymbol{a}, \boldsymbol{b}>
$$

- 向量外积，外积的结果是一个垂直于叉乘的两个向量的向量，大小为$|a||b|sin<a,b>$,是两个向量张成的四边形的有向面积

  $$
  \boldsymbol{a}\times\boldsymbol{b} = 
      \begin{Vmatrix} 
      \boldsymbol{e}_1 & \boldsymbol{e}_2 & \boldsymbol{e}_3 \\
      a_1 & a_2 & a_3 \\
      b_1 & b_2 & b_3 \\
      \end{Vmatrix} = 
  
      \begin{bmatrix}
      a_2 b_3 - a_3 b_2 \\
      a_3 b_1 - a_1 b_3 \\
      a_1 b_2 - a_2 b_1
      \end{bmatrix} = 
  
      \begin{bmatrix}
      0 & -a_3 & a_2 \\
      a_3 & 0 & -a_1 \\
      -a_2 & a_1 & 0
      \end{bmatrix}\boldsymbol{b} 
  
      \xlongequal{\mathrm{def}} 
  
  \boldsymbol{a}^{\land}\boldsymbol{b} \\
  $$
- 反对称矩阵：

  $$
  \boldsymbol{a}^{\wedge} = 
  	\begin{bmatrix}
      0 & -a_3 & a_2 \\
      a_3 & 0 & -a_1 \\
      -a_2 & a_1 & 0
  	\end{bmatrix}
  $$

#### 空间旋转的矩阵表达

![image-20240507133926711](robot_arm_note.assets/image-20240507133926711.png)

- 旋转矩阵 $ ^0\boldsymbol{R}_B$ 表示了坐标系 $\rm{(KS)}_B$ 在坐标系 $(\rm{KS})_0$ 中的表达

  $$
  ^0\boldsymbol{R}_\mathrm{B}=\left(_{(0)}\boldsymbol{e}_x^{(\mathrm{B})},_{(0)}\boldsymbol{e}_y^{(\mathrm{B})},_{(0)}\boldsymbol{e}_z^{(\mathrm{B})}\right) \\
  
  ^0\boldsymbol{R}_\mathrm{B}=\left(_{(0)}\boldsymbol{e}_y^{(0)}, -_{(0)}\boldsymbol{e}_x^{(0)},_{(0)}\boldsymbol{e}_z^{(0)}\right)=\left(\begin{array}{ccc}0&-1&0\\1&0&0\\0&0&1\end{array}\right)
  $$
- 三维空间下旋转矩阵大小为 3x3
- 旋转矩阵是正交矩阵，其逆为自身的转置
- 旋转矩阵的行列式为1
- 旋转矩阵每一列的平方和为1

  $$
  \mathrm{SO}(3) = \{\boldsymbol{R}\in\R^{3\times 3}|\boldsymbol{R}\boldsymbol{R}^\mathrm{T}=\boldsymbol{I}, \mathrm{det}(\boldsymbol{R}) = 1\} \\
  \boldsymbol{R}^{-1} = \boldsymbol{R}^{\mathrm{T}}\\
  \boldsymbol{a}' = \boldsymbol{R}^{-1}\boldsymbol{a} = \boldsymbol{R}^{\mathrm{T}}\boldsymbol{a}
  $$
- 旋转矩阵属于特殊正交群 SO(3) (Special Orthogonal Group)，（只有**一个**良好的运算的集合，称之为群）

  $$
  \boldsymbol{R}_{1}+\boldsymbol{R}_{2} \notin \mathrm{SO}(3) \\
  \boldsymbol{R}_{1} \boldsymbol{R}_{2} \in \mathrm{SO}(3)
  $$
- 旋转矩阵不满足交换律

  $$
  \boldsymbol{R}_1\boldsymbol{R}_2\neq \boldsymbol{R}_2\boldsymbol{R}_1
  $$

#### 空间旋转的复式变换表达（欧拉角）

- 基础角度变换

  - 任意两个坐标系之间的旋转关系，可以通过**三次**按照一定顺序的**基础旋转变换**来表达。
  - 基础旋转变换：指通过XYZ其中一个轴进行旋转得到的旋转变换
  - 旋转角度方向默认通过右手定则，即右手大拇指指向轴箭头方向，四指环绕方向指向旋转角正方向

    $$
    \begin{aligned}\boldsymbol R_x(\alpha)&=\left(\begin{array}{ccc}1&0&0\\0&\cos(\alpha)&-\sin(\alpha)\\0&\sin(\alpha)&\cos(\alpha)\end{array}\right)\\\boldsymbol R_y(\beta)&=\left(\begin{array}{ccc}\cos(\beta)&0&\sin(\beta)\\0&1&0\\-\sin(\beta)&0&\cos(\beta)\end{array}\right)\\\boldsymbol R_z(\gamma)&=\left(\begin{array}{ccc}\cos(\gamma)&-\sin(\gamma)&0\\\sin(\gamma)&\cos(\gamma)&0\\0&0&1\end{array}\right)\end{aligned}
    $$

    ![image-20240520162428492](robot_arm_note.assets/image-20240520162428492.png)
- 欧拉角的分类

  - **按照旋转轴分类**

    - **Proper Euler angles** (*z*-*x*-*z*, *x*-*y*-*x*, *y*-*z*-*y*, *z*-*y*-*z*, *x*-*z*-*x*, *y*-*x*-*y*)

      - 也被称为 classisc euler angles
      - 特点：第一次与第三次旋转角相同
    - **Tait–Bryan angles** (*x*-*y*-*z*, *y*-*z*-*x*, *z*-*x*-*y*, *x*-*z*-*y*, *z*-*y*-*x*, *y*-*x*-*z*)

      - 也被称为 Cardan angles, nautical angles, (heading, elevation, and bank), (RPY: yaw, pitch, and roll).
      - 特点：三次旋转角不同
  - **按照参考坐标系分类**

    - 欧拉角-内旋法 intrinsic rotations （又称动态欧拉角）

      - 基于**每次旋转之后的坐标系**
      - 旋转矩阵应用**右乘**顺序
      - 示例：（Cardan-Angle）

        1. 绕原坐标系的 x 轴旋转 $\alpha$
        2. 绕新坐标系的 y 轴旋转 $\beta$
        3. 绕新坐标系的 z 轴旋转 $\gamma$

        $$
        \boldsymbol{R}_{\mathrm{KARD}}(\alpha,\beta,\gamma)=
        
        \underbrace{\overbrace{\boldsymbol{R}_x(\alpha)}^{1.}\overbrace{\boldsymbol{R}_y(\beta)}^{2.}\overbrace{\boldsymbol{R}_z(\gamma)}^{3.}}_{\text{right multiplication}}
        $$

        <img src="robot_arm_note.assets/image-20240507150958916.png" alt="image-20240507150958916" style="zoom: 67%;" />
    - 欧拉角-外旋法 extrinsic rotations （又称静态欧拉角）

      - 基于**世界坐标系**
      - 旋转矩阵应用**左乘**顺序
      - 示例：（RPY-Angle）

        1. 绕世界坐标系的 x 轴旋转 $\alpha$
        2. 绕世界坐标系的 y 轴旋转 $\beta$
        3. 绕世界坐标系的 z 轴旋转 $\gamma$

        $$
        \boldsymbol{R}_{\mathrm{RPY}}(\alpha,\beta,\gamma)=
        
        \underbrace{\overbrace{\boldsymbol{R}_x(\gamma)}^{3.}\overbrace{\boldsymbol{R}_y(\beta)}^{2.}\overbrace{\boldsymbol{R}_z(\alpha)}^{1.}}_{\text{left multiplication}}
        $$
  - 欧拉角 Gimbal-Lock 万向锁问题

    - 一旦选择±90°作为第二次旋转角度，就会导致第一次旋转和第三次旋转等价（第三次的旋转相当于叠加在第一次旋转上），丢失了一个表示维度。这种性质不适合插值和迭代等应用。
    - [万向锁问题动画演示](https://www.bilibili.com/video/BV1MZ4y1m7CT/?spm_id_from=333.337.search-card.all.click&vd_source=0f805e57bae4caccd39bef0a80028bec)
    - [万向锁问题解释](https://blog.csdn.net/weixin_41364246/article/details/129402879)
  - 欧拉角的正逆解

    - 正解：通过三次基础旋转获得复式旋转矩阵

      $$
      \boldsymbol{R}_\mathrm{xyz} = \boldsymbol{R}_\mathrm{x}(\alpha)\boldsymbol{R}_\mathrm{y}(\beta)\boldsymbol{R}_\mathrm{z}(\gamma)
      $$
    - 逆解：通过复式旋转矩阵逆推得到三次基础旋转

      - 分析法
      - 数值法
      - 多解
  - 欧拉角表达法的优缺点

    - 优点
      - 标记简单，只需三个变量
      - 表达方式直观
    - 缺点
      - 旋转顺序影响结果
      - 欧拉角表达方式有多种变体
      - 万向锁问题

#### 空间旋转的轴角表达

- 任意旋转都可以用一个旋转轴和一个旋转角来刻画
- 旋转向量的方向与单位长度的旋转轴 $\boldsymbol{u}$ 一致，长度等于旋转角$\theta$，则可以表示为 $\theta \boldsymbol{u}$

  ![image-20240520171602461](robot_arm_note.assets/image-20240520171602461.png)
- **罗德里格斯公式**

  - 从旋转向量 $\theta \boldsymbol{u}$ 到旋转矩阵 $\boldsymbol{R}$

  $$
  \boldsymbol{R} = \cos(\theta )\boldsymbol{I} + (1-\cos(\theta))\boldsymbol{u}\boldsymbol{u}^{\mathrm{T}} + \sin(\theta)\boldsymbol{u}^{\land}  
  
  \\
  
  \boldsymbol{u}=(u_x,u_y,u_z)^\mathrm{T} 
  
  \\
  \mathrm{c}_\theta=\cos(\theta)
  
  \\
  
  \mathrm{s}_\theta=\sin(\theta). 
  
  \\
  
  \boldsymbol{R}=\begin{pmatrix}u_x^2(1-\mathrm{c}_\theta)+\mathrm{c}_\theta&u_x u_y(1-\mathrm{c}_\theta)-u_z \mathrm{s}_\theta&u_x u_z(1-\mathrm{c}_\theta)+u_y \mathrm{s}_\theta\\u_x u_y(1-\mathrm{c}_\theta)+u_z \mathrm{s}_\theta&u_y^2(1-\mathrm{c}_\theta)+\mathrm{c}_\theta&u_y u_z(1-\mathrm{c}_\theta)-u_x \mathrm{s}_\theta\\u_x u_z(1-\mathrm{c}_\theta)-u_y \mathrm{s}_\theta&u_y u_z(1-\mathrm{c}_\theta)+u_x \mathrm{s}_\theta&u_z^2(1-\mathrm{c}_\theta)+\mathrm{c}_\theta\end{pmatrix}
  $$

  - 从旋转矩阵 $\boldsymbol{R}$ 到旋转向量 $\theta \boldsymbol{n}$，

    - 方法1（SLAMBOOK2）：罗德里格斯公式（Rodrigues），通过对 $\boldsymbol{R}$ 求迹（tr, trace, 即求矩阵对角线元素之和）：

      $$
      \theta = \arccos(\frac{(\mathrm{tr}(\boldsymbol{R})) - 1}{2}) \\
      \theta = \arccos(\frac{(1+2\cos(\theta)) - 1}{2}) \\
      \boldsymbol{R}\boldsymbol{n} 
      = \boldsymbol{n}, (\boldsymbol{R}-\bold{I})\boldsymbol{n} = \boldsymbol{0}
      $$
    - 方法2（LUH ROBOTIK1 Script）：

      $$
      \left.\boldsymbol{R}^*=\left(\begin{array}{ccc}r_{11}&r_{12}&r_{13}\\r_{21}&r_{22}&r_{23}\\r_{31}&r_{32}&r_{33}\end{array}\right.\right) 
      \\
      \cos(\theta)=\frac{1}{2}\left(r_{11}+r_{22}+r_{33}-1\right)
      \\
      \sin(\theta)=\pm\frac{1}{2} \sqrt{(r_{32}-r_{23})^2+(r_{13}-r_{31})^2+(r_{21}-r_{12})^2}
      \\
      \theta=\arctan2\left(\sin(\theta),\cos(\theta)\right)
      \\
      \boldsymbol{u}=(u_x,u_y,u_z)^\mathrm{T}=\frac{1}{2 \sin(\theta)} (r_{32}-r_{23},r_{13}-r_{31},r_{21}-r_{12})^\mathrm{T}
      $$
- 轴角表达法的优缺点

  - 优点

    - 较少的参数数量
    - 适用于插值应用
    - 相对直观的表达
  - 缺点

    - 不方便表示连续旋转
    - 旋转向量的奇异性发生在转角 $\theta$ 超过$2\pi$时产生周期性

#### 旋转的四元数表达

- 四元数没有奇异性，可以解决三维向量描述旋转时的奇异性问题
- 四元数基本定义

  $$
  \begin{align}
  \boldsymbol{q} &= q_0 + q_1\mathrm{i} + q_2\mathrm{j} + q_3\mathrm{k} \\
  \boldsymbol{q} &= [s, \boldsymbol{v}^{\mathrm{T}}], s=q_0\in\R,\boldsymbol{v}=[q_1, q_2, q_3]^{\mathrm{T}}\in\R^3 \\
  \end{align}
  $$
- 四元数运算

  - 加减

    $$
    \boldsymbol{q}_a\pm\boldsymbol{q}_b = [s_a\pm s_b,\boldsymbol{v}_a\pm\boldsymbol{v}_b]^{\mathrm{T}}
    $$
  - 乘法

    $$
    \boldsymbol{q}_a \boldsymbol{q}_b = [s_a s_b - \boldsymbol{v}_a^{\mathrm{T}}\boldsymbol{v}_b, s_a\boldsymbol{v}_b + s_b\boldsymbol{v}_a + \boldsymbol{v}_a\times \boldsymbol{v}_b]^{\mathrm{T}}
    $$
  - 模长

    $$
    \begin{align}
    \begin{Vmatrix}\boldsymbol{q}_a\end{Vmatrix} &= \sqrt{s_a^2 + x_a^2 + y_a^2 + z_a^2} \\
    
    \begin{Vmatrix}\boldsymbol{q}_a\boldsymbol{q}_b\end{Vmatrix} &= \begin{Vmatrix}\boldsymbol{q}_a\end{Vmatrix}\begin{Vmatrix}\boldsymbol{q}_b\end{Vmatrix}
    \end{align}
    $$
  - 共轭

    $$
    \boldsymbol{q}^{*} = [s_a, -\boldsymbol{v}_a]^{\mathrm{T}} \\
    \boldsymbol{q}^{*}\boldsymbol{q} = \boldsymbol{q}\boldsymbol{q}^{*} = [s_a^2 + \boldsymbol{v}^{
    \mathrm{T}}\boldsymbol{v}, \boldsymbol{0}]^{\mathrm{T}}
    $$
  - 逆

    $$
    \boldsymbol{q}^{-1} = \boldsymbol{q}^{*}/\begin{Vmatrix}\boldsymbol{q}\end{Vmatrix}^2 \\
    \boldsymbol{q}\boldsymbol{q}^{-1} = \boldsymbol{q}^{-1}\boldsymbol{q} = 1 \\
    (\boldsymbol{q}_a\boldsymbol{q}_b)^{-1} = \boldsymbol{q}_b^{-1}\boldsymbol{q}_a^{-1}
    $$
  - 数乘

    $$
    \mathrm{k}\boldsymbol{q} = [\mathrm{k}s,\mathrm{k}\boldsymbol{v}]^{\mathrm{T}}
    $$
- 四元数旋转

  - 空间三维点 $\boldsymbol{p} = [x,y,z]\in\R^{3}$ 经过旋转 $\boldsymbol{q}$ 变为 $\boldsymbol{p}'$
    $$
    \boldsymbol{p} = [0,x,y,z]^{\mathrm{T}} = [0, \boldsymbol{v}]^\mathrm{T} \\
    \boldsymbol{p}' = \boldsymbol{q}\boldsymbol{p}\boldsymbol{q}^{-1}
    $$
- 四元数转换

  - 四元数到旋转矩阵

    $$
    \boldsymbol{R} = \boldsymbol{v}\boldsymbol{v}^{\mathrm{T}} + s^2\boldsymbol{I} + 2s\boldsymbol{v}^{\wedge} + (\boldsymbol{v}^{\wedge})^{2} 
    
    \\ \\
    
    \left.\boldsymbol{R}=\left(\begin{array}{ccc}q_0^2+q_1^2-q_2^2-q_3^2&2q_1q_2+2q_0q_3&2q_1q_3-2q_0q_2\\2q_1q_2-2q_0q_3&q_0^2-q_1^2+q_2^2-q_3^2&2q_2q_3+2q_0q_1\\2q_1q_3+2q_0q_2&2q_2q_3-2q_0q_1&q_0^2-q_1^2-q_2^2+q_3^2\end{array}\right.\right)
    $$
  - 旋转矩阵到四元数

    $$
    R=\begin{pmatrix}r_{11}&r_{12}&r_{13}\\r_{21}&r_{22}&r_{23}\\r_{31}&r_{32}&r_{33}\end{pmatrix} 
    
    \\ \\
    
    \begin{aligned}&q_0=\frac{\sqrt{tr(R)+1}}{2}\\&q_1=\frac{r_{23}-r_{32}}{4q_0}\\&q_2=\frac{r_{31}-r_{13}}{4q_0}\\&q_3=\frac{r_{12}-r_{21}}{4q_0}\end{aligned}
    $$
  - 四元数到轴角表达

  $$
  \begin{align}
  \theta &= 2\arccos(s) \\
  [n_x, n_y, n_z]^{\mathrm{T}} &= [q_1, q_2, q_3]^{\mathrm{T}}/\sin(\frac{\theta}{2})
  \end{align}
  $$
- 四元数表达法的优缺点

  - 优点
    - 参数数量少
    - 方便表达连续旋转
    - 适用于插值计算
    - 计算相比旋转矩阵更快
    - 无奇异性
  - 缺点
    - 表达方式不够直观

#### 齐次变换矩阵（空间旋转+位移）

- 三维空间下齐次变换矩阵大小为 4x4，属于特殊欧式群 SE(3)
- 在三维空间中的欧式齐次变换矩阵，由一个旋转矩阵 $\boldsymbol{R}$ 和一个位移向量 $\boldsymbol{t}$ 组成

  $$
  \mathrm{SE}(3) =
  \{
  \boldsymbol{T} = 
  	\begin{bmatrix}
  		\boldsymbol{R} & \boldsymbol{t} \\
  		\boldsymbol{0} & 1
  	\end{bmatrix} 
  	\in \R^{4\times 4} | \boldsymbol{R}\in\mathrm{SO}(3), t\in \R^3
  \}
  $$
- 求逆，逆矩阵表示相反的变换

  $$
  \left({}^0\boldsymbol{T}_{\mathrm{B}}\right)^{-1}={}^{\mathrm{B}}\boldsymbol{T}_0 = 
  	\begin{bmatrix}
  		(^0\boldsymbol{R}_\mathrm{B})^{\mathrm{T}} & -(^0\boldsymbol{R}_\mathrm{B})^{\mathrm{T}}_{(0)}\boldsymbol{t}_\mathrm{B} \\
  		\boldsymbol{0} & 1
  	\end{bmatrix}
  $$
- 行列式为1

  $$
  \det\begin{pmatrix}\boldsymbol{T}\end{pmatrix}=1.
  $$

![image-20240520181639752](robot_arm_note.assets/image-20240520181639752.png)

- 通过齐次变换矩阵 $^0\boldsymbol{T}_\mathrm{B}$，求解坐标系B中的点P $_{(\mathrm{B})}\boldsymbol{x}_{\mathrm{P}}$ 在坐标系0中的坐标 $_{(\mathrm{0})}\boldsymbol{x}_{\mathrm{P}}$，位置向量补齐时需要加元素 1：

  $$
  _{(0)}\boldsymbol{x}_{\mathrm{P}}=\begin{pmatrix}_{(0)}\boldsymbol{t}_{\mathrm{P}}^{\mathrm{T}},1\end{pmatrix}^{\mathrm{T}}=\begin{pmatrix}_{(0)}x_{\mathrm{P}},_{(0)}y_{\mathrm{P}},_{(0)}z_{\mathrm{P}},1\end{pmatrix}^{\mathrm{T}}
  
  \\ \\
  
  _{(\mathrm{B})}\boldsymbol{x}_{\mathrm{P}}=\begin{pmatrix}_{(\mathrm{B})}\boldsymbol{t}_{\mathrm{P}}^{\mathrm{T}}, 1\end{pmatrix}^{\mathrm{T}}=\begin{pmatrix}_{(\mathrm{B})}x_{\mathrm{P}}, _{(\mathrm{B})}y_{\mathrm{P}}, _{(\mathrm{B})}z_{\mathrm{P}}, 1\end{pmatrix}^{\mathrm{T}}
  
  \\ \\
  
  \left._{(0)}\boldsymbol{x}_{\mathrm{P}}={}^0\boldsymbol{T}_{\mathrm{B}(\mathrm{B})}\boldsymbol{x}_{\mathrm{P}}=\left(\begin{array}{ccc|c}{} & ^0\boldsymbol{R}_{\mathrm{B}} &  & _{(\mathrm{0})}\boldsymbol{t}_{\mathrm{P}} \\ 0 & 0 & 0 & 1\end{array}\right.\right)\left(\begin{array}{ccc}{}_{(\mathrm{B})}\boldsymbol{r}_{\mathrm{P}}\\ 1\end{array}\right)=\left(\begin{array}{ccc}{} _{(\mathrm{0})}\boldsymbol{t}_{\mathrm{B}}+   ^0\boldsymbol{R}_{\mathrm{B}(\mathrm{B})}\boldsymbol{t}_{\mathrm{P}}\\ 1\end{array}\right)
  $$
- 通过齐次变换矩阵 $^0\boldsymbol{T}_\mathrm{B}$，求解坐标系 0 中的向量 $_{(0)}\boldsymbol{a}$ 在坐标系 B 中的向量 $_{(B)}\boldsymbol{a}$，向量补齐时加元素 0：

  $$
  _{(\mathrm{B})}\boldsymbol{a}^{\prime}={}^\mathrm{B}\boldsymbol{T}_0\boldsymbol{a}^{\prime}=\left(\begin{array}{c}^\mathrm{B}\boldsymbol{R}_{0 (0)}\boldsymbol{a}\\0\end{array}\right)
  $$
- 齐次变换矩阵的链式法则

  $$
  _{(0)}\boldsymbol{x}=^0\boldsymbol{T}_1{}^1\boldsymbol{T}_2\cdots{}^{n-2}\boldsymbol{T}_{n-1}{}^{n-1}\boldsymbol{T}_{n(n)}\boldsymbol{x}={}^0\boldsymbol{T}_{n(n)}\boldsymbol{x} 
  
  \\ \\
  
  ^i\boldsymbol{T}_j=\left({}^0\boldsymbol{T}_i\right)^{-1}{}^0\boldsymbol{T}_j={}^i\boldsymbol{T}_0{}^0\boldsymbol{T}_j.
  $$

### 机械臂运动学

#### 运动学定义

![image-20240521110646434](robot_arm_note.assets/image-20240521110646434.png)

- 关节配置

  - 以六自由度的机械臂为例，则关节配置为一个6维向量，表示每个关节的角度

  $$
  \boldsymbol{q}=(q_1,q_2,\ldots,q_6)^\mathrm{T}
  $$
- 末端位姿

  - 表示机械臂末端 Endeffector 相对于机械臂基底坐标系的空间位姿，$x， y， z$ 表示三维空间坐标，$\phi, \psi, \theta$ 表示末端坐标系到基底坐标系的三个欧拉角变化角度。

  $$
  \boldsymbol{x}_\mathrm{E}=\begin{pmatrix}_{(0)}x_\mathrm{E},_{(0)}y_\mathrm{E},_{(0)}z_\mathrm{E},\phi_\mathrm{E},\psi_\mathrm{E},\theta_\mathrm{E}\end{pmatrix}^\mathrm{T}
  $$
- 正运动学

  - 已知各关节角度，求末端位姿
    $$
    \boldsymbol{x}_\mathrm{E}=f(\boldsymbol{q})
    $$
- 逆运动学

  - 已知末端位姿，求各关节角度
    $$
    \boldsymbol{q} = f(\boldsymbol{x}_{\mathrm{E}})
    $$

#### 正运动学

- 已知各关节角度，求末端位姿，即找到每个关节之间的坐标转换，通过齐次变换链式法则得到从机械臂基座到末端关节的变换矩阵。

$$
\boldsymbol{x}_\mathrm{E}=f(\boldsymbol{q})

\\ \\

{}^0\boldsymbol{T}_\mathrm{E}(\boldsymbol{q})={}^0\boldsymbol{T}_1(q_1){}^1\boldsymbol{T}_2(q_2) \cdots{}^{n-1}\boldsymbol{T}_\mathrm{E}(q_n)
$$

- DH参数（Denavit–Hartenberg parameters）：通过确立各个关节的坐标系，以及连续两个坐标系之间的齐次变换矩阵（通常为四个参数）参数，完成整个机械臂的建模。

  - 参考
    - [DH表示法(Standard Version)](https://zhuanlan.zhihu.com/p/696612275)
    - [DH表示法(Craig Version)](https://zhuanlan.zhihu.com/p/696195319)
    - [什么是改进DH参数](https://www.zhihu.com/question/23880951/answer/2957726427)
    - [Denavit–Hartenberg parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
    - [在线DH参数可视化](https://vis-ro.web.app/dh-model)
    - [为什么改进的DH模型优于标准DH模型，却大多都是使用标准D-H模型？](https://www.zhihu.com/question/22365926)
- SDH 标准DH参数

  ![img](robot_arm_note.assets/568px-Classic_DH_Parameters_Convention.png)

  1. 确定坐标系

     1. 确定每个关节以及他们的顺序 $ \boldsymbol{q} = \{q_1, q_2, \cdots, q_n \}$
     2. 坐标系下标从0开始，若有 n 个关节，则有 n+1 个坐标系
     3. 关节 $q_i$ 的旋转轴或位移轴为该关节坐标系的 $z_{i-1}$ 轴，如果是旋转轴， $z_{i-1}$ 轴方向通过右手定则确定正方向。
     4. 每个 $x_{i-1}$ 为 $z_{i-1}$ 与 $z_i$ 的公垂线
     5. 通过 $x_i$ 与 $z_i$ 确定 $y_i$ 以及 该坐标系的坐标原点 $o_i$
  2. 确定参数

     - 两个相邻坐标系之间的参数的定义
       1. $d_i$: 沿 $z_{i-1}$ 轴的位移，针对平移轴，$q_i$ 会包括在该参数中
       2. $\theta_i$: 绕 $z_{i-1}$ 轴的旋转，针对旋转轴， $q_i$ 会包括在该参数中
       3. $a_i$: 沿 $x_i$ 轴的位移
       4. $\alpha_i$: 绕 $x_i$ 轴的旋转
     - 如果有 n 个关节，则有 nx4 个参数
  3. 确定变换矩阵

     $$
     ^{i-1}T_i=\mathrm{Trans}_{z_{i-1}}(d_i)\cdot\mathrm{Rot}_{z_{i-1}}(\theta_i)\cdot\mathrm{Trans}_{x_i}(a_i)\cdot\mathrm{Rot}_{x_i}(\alpha_i)
     
     \\ \\
     
     ^0\boldsymbol{T}_\mathrm{E}(\boldsymbol{q}) = ^0\boldsymbol{T}_\mathrm{1}(q_1) ^0\boldsymbol{T}_\mathrm{2}(q_2) \cdots ^0\boldsymbol{T}_\mathrm{n}(q_n)
     $$
  4. 举例

     ![image-20240521143909424](robot_arm_note.assets/image-20240521143909424.png)

     ![image-20240521143925873](robot_arm_note.assets/image-20240521143925873.png)
- MDH 改进DH参数

  - 与 SDH 的区别

    1. 关节 $q_i$ 的旋转轴或位移轴为该关节坐标系的 $z_{i}$ 轴，而非 $z_{i-1}$
    2. 参数顺序为

       1. $a_i$: 沿 $x_{i-1}$ 轴的位移
       2. $\alpha_i$: 绕 $x_{i-1}$ 轴的旋转
       3. $d_i$: 沿 $z_i$ 轴的位移，针对平移轴，$q_i$ 会包括在该参数中
       4. $\theta_i$: 绕 $z_i$ 轴的旋转，针对旋转轴， $q_i$ 会包括在该参数中
    3. 两个关节坐标系之间的转换顺序

       $$
       ^{i-1}\boldsymbol{T}_i = \mathrm{Rot}_{x_{i-1}}\left(\alpha_i\right) \cdot \mathrm{Trans}_{x_{i-1}}\left(a_i\right)\cdot\mathrm{Rot}_{z_i}\left(\theta_i\right)\cdot\mathrm{Trans}_{z_i}\left(d_i\right)
       $$
    4. 标准的 DH 坐标系在处理树形结构（一个连杆末端连接两个关节）的时候会产生歧义；而改进的 DH 则没有这个问题。

    ![img](robot_arm_note.assets/519px-DHParameter.png)

#### 逆运动学

- 参考资料

  - [机器人的运动学解——逆向运动学解](https://zhuanlan.zhihu.com/p/450749372)
  - [机械臂的逆运动学求解有统一的方法吗](https://www.zhihu.com/question/400650301/answer/1282203168)
  - [MATLAB机器人工具箱中机器人逆解是如何求出来的](https://www.zhihu.com/question/41673569/answer/129670927)
- 已知末端位姿，求各关节角度

  $$
  \boldsymbol{q} = f(\boldsymbol{x}_{\mathrm{E}})
  $$
- 逆运动学的解法

  - 分析法
    - 特点
      - 通过正运动学公式或通过几何关系进行分析推理
      - 针对非冗余机械臂有限个解（存在多解）
      - 效率高于数值法
      - 随着自由度上升，求得解析解的难度也越来越大
      - 通用性差，适用性弱
      - 一般的6自由度机械臂需要满足**Pieper准则**，即机器人满足以下两个充分条件中的一个，就会得到封闭解，这两个条件是：
        - 三个相邻关节轴相交于一点
        - 三个相邻关节轴相互平行
    - 具体方法
      - 通过对正运动学公式利用三角函数等进行推理
      - 通过几何关系进行推理
      - Rosen Diankov 于2010年前后提出的 IKfast 算法，适用于 dof <= 7 的机械臂
  - 路径规划法
    - 在关节空间中进行路径规划
      - 基于图搜索
        - Dijkstra
        - A*
      - 基于采样
        - PRM
        - RRT
    - 例：OMPL 机械臂运动规划库
  - 数值法
    - 特点
      - 适用性广，更适合冗余机械臂
      - 需要更多时间进行迭代计算
      - 受初始位姿影响
      - 收敛性无法保证，可能陷入局部最优
    - 具体方法
      - 优化法
        - 传统优化，通过构建优化方程 $ \boldsymbol{f}(\boldsymbol{q}) - \boldsymbol{x}_\mathrm{E} = 0$ 获取目标 $\boldsymbol{q}$
          - Steepest Descent (SD) method
          - Newton–Raphson (NR) method
          - Variable metric (VM) method
          - Levenberg–Marquardt (LM) method
        - 智能优化
          - 遗传优化
          - 粒子群优化
          - 神经网络优化
          - ...
      - 迭代法
        - 基于雅克比矩阵的微分运动学

##### 分析法求解逆运动学的例子

- 例1：简单的 2DOF 平面连杆机械臂，通过**正运动学公式**分析求解逆运动学

![image-20240521161325551](robot_arm_note.assets/image-20240521161325551.png)

![image-20240521161409509](robot_arm_note.assets/image-20240521161409509.png)

![image-20240521161503363](robot_arm_note.assets/image-20240521161503363.png)

![image-20240521161514862](robot_arm_note.assets/image-20240521161514862.png)

![image-20240521161541511](robot_arm_note.assets/image-20240521161541511.png)

![image-20240521161557510](robot_arm_note.assets/image-20240521161557510.png)

- 例2：简单的 2DOF 平面连杆机械臂，通过**几何关系**分析求解逆运动学

  ![image-20240521162037761](robot_arm_note.assets/image-20240521162037761.png)

![image-20240521162050670](robot_arm_note.assets/image-20240521162050670.png)

![image-20240521162108057](robot_arm_note.assets/image-20240521162108057.png)

![image-20240521162120128](robot_arm_note.assets/image-20240521162120128.png)

![image-20240521162137157](robot_arm_note.assets/image-20240521162137157.png)

![image-20240521162147339](robot_arm_note.assets/image-20240521162147339.png)

- 其他例子：6自由度机械臂的分析法逆解
  - [工业机械臂逆运动学算法详解](https://zhuanlan.zhihu.com/p/440748878)
  - [UR和iiwa协作机械臂逆运动学解析解算法](https://zhuanlan.zhihu.com/p/326387013)
  - [UR3机械臂正逆运动学详解](https://blog.csdn.net/u011779517/article/details/111116938)
  - [遨博Aubo-i10机器人正逆运动学公式推导](https://blog.csdn.net/l1216766050/article/details/96961989)

##### 数值法求解逆运动学的例子

- cpprobotics demo

##### 其他软件工具

- Matlab Robotic Toolbox
  - 基于雅克比矩阵的迭代法
  - [MATLAB机器人工具箱中机器人逆解是如何求出来的](https://www.zhihu.com/question/41673569/answer/129670927)
- ROS MoveIt 中的工具
  - KDL
    - MoveIt 的默认运动学求解器，基于牛顿优化法
  - Track-IK
    - MoveIt 中基于雅克比矩阵的迭代法
- IKFAST - 获取 dof <= 7 自由度的解析解
  - [openrave-IKFAST](http://openrave.org/docs/latest_stable/openravepy/ikfast/)
  - [IKFAST-Generator （超好用的 IKFAST 在线工具）](https://www.hamzamerzic.info/ikfast_generator/)
  - [逆运动学库：IKFAST库的使用：ikfast](https://zhuanlan.zhihu.com/p/670483129)

#### 微分运动学（雅克比矩阵）

##### 雅克比矩阵定义

- 机械臂的微分运动学描述了末端的速度或加速度与关节的速度或加速度之间的关系。
- 雅克比矩阵描述了末端与关节在微小形变下的线性关系：

  $$
  \boldsymbol{x}_\mathrm{E} = \boldsymbol{f}(\boldsymbol{q})
  
  \\
  
  \Delta\boldsymbol{x}_\mathrm{E} = \boldsymbol{J}(\boldsymbol{q})\Delta\boldsymbol{q}
  
  \\
  
  \Delta\boldsymbol{q} = \boldsymbol{J}^{-1}(\boldsymbol{q})\Delta\boldsymbol{x}_\mathrm{E}
  $$
- 末端与关节的速度关系（一阶偏微分）：

  $$
  \dot{\boldsymbol{x}}_\mathrm{E}=\boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}}
  
  \\
  
  \dot{\boldsymbol{q}}=\boldsymbol{J}^{-1}(\boldsymbol{q}) \dot{\boldsymbol{x}}_\mathrm{E}
  $$
- 末端与关节的加速度关系（二阶偏微分）：

  $$
  \ddot{\boldsymbol{x}}_\mathrm{E}=\boldsymbol{J}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\dot{\boldsymbol{J}}(\boldsymbol{q},\dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}
  
  \\
  
  \ddot{\boldsymbol{q}}=\boldsymbol{J}^{-1}(\boldsymbol{q}) \ddot{\boldsymbol{x}}_\mathrm{E}-\boldsymbol{J}^{-1}(\boldsymbol{q}) \dot{\boldsymbol{J}}(\boldsymbol{q},\dot{\boldsymbol{q}}) \boldsymbol{J}^{-1}(\boldsymbol{q}) \dot{\boldsymbol{x}}_\mathrm{E}
  $$
- 雅克比矩阵的广义逆 $\boldsymbol{J}^\dagger$: 由于雅可比矩阵不一定是方阵，根据关节数量的不同，求逆方式如下：

  $$
  \boldsymbol{J}^{\dagger}=\begin{cases}(\boldsymbol{J}^{T}\boldsymbol{J})^{-1}\boldsymbol{J}^{T},&n<6\\\boldsymbol{J}^{-1},&n=6\\\boldsymbol{J}^{T}(\boldsymbol{J}\boldsymbol{J}^{T})^{-1},&n>6&&\end{cases}
  $$

##### 雅克比矩阵的分类

###### 几何雅克比矩阵 $\boldsymbol{J}_\mathrm{G}$

- 描述的末端速度为相对于基地坐标系的速度向量以及角速度向量

  $$
  \Delta\boldsymbol{x}_\mathrm{E} = 
  
  \begin{bmatrix}
  _{(0)}\boldsymbol{v}_\mathrm{E} \\
  _{(0)}\boldsymbol{\omega}_\mathrm{E} 
  \end{bmatrix}
  
  =
  
  \begin{bmatrix}
  _{(0)}v_\mathrm{E,x} \\
  _{(0)}v_\mathrm{E,y} \\
  _{(0)}v_\mathrm{E,z} \\
  _{(0)}{\omega}_\mathrm{E,x}\\
  _{(0)}{\omega}_\mathrm{E,y}\\
  _{(0)}{\omega}_\mathrm{E,z}\\
  \end{bmatrix}
  $$
- 推导过程基于DH参数传递链：

  $$
  _{(0)}\dot{\boldsymbol{x}}_{\mathrm{E,G}} = 
  
  \begin{bmatrix}
  _{(0)}\dot{\boldsymbol{x}}_{\mathrm{E,trans}} \\
  _{(0)}\dot{\boldsymbol{x}}_{\mathrm{E,rot}}
  \end{bmatrix}
  
  =
  \underbrace{
  \begin{bmatrix}
  \boldsymbol{j}_{\mathrm{t, 1}} & \cdots & \boldsymbol{j}_{\mathrm{t, n}} 
  \\
  \boldsymbol{j}_{\mathrm{r, 1}}  & \cdots & \boldsymbol{j}_{\mathrm{r, n}} 
  \end{bmatrix}}_{\boldsymbol{J}_\mathrm{G}(\boldsymbol{q})}
  \dot{\boldsymbol{q}}
  
  \\
  
  \begin{aligned}
  &\text{translation joint: } 
  \begin{bmatrix}
  \boldsymbol{j}_{\mathrm{t, i}}
  \\
  \boldsymbol{j}_{\mathrm{r, i}} 
  \end{bmatrix}
  = 
  \begin{bmatrix}
  _{(0)}\boldsymbol{e}_z^{i-1}
  \\
  \boldsymbol{0} 
  \end{bmatrix}
  
  \\
  &\text{rotation joint: }
  \begin{bmatrix}
  \boldsymbol{j}_{\mathrm{t, i}}
  \\
  \boldsymbol{j}_{\mathrm{r, i}} 
  \end{bmatrix}
  = 
  \begin{bmatrix}
  _{(0)}\boldsymbol{e}_z^{i-1} \times _{(0)}\boldsymbol{r}_{i-1,\mathrm{E}}
  \\
  \boldsymbol{0} 
  \end{bmatrix}
  
  \\ \\
  
  &\text{with: } \\
  & _{(0)}\boldsymbol{e}_z^{(i-1)}=\underbrace{^0\boldsymbol{R}_1{}^1\boldsymbol{R}_2\ldots{}^{i-2}\boldsymbol{R}_{i-1}}_{^0\boldsymbol{R}_{i-1}}\left(\begin{array}{c}0\\0\\1\end{array}\right)
  
  \\
  
  & \left.\left(\begin{array}{c}_{(0)}\boldsymbol{r}_{(i-1), \mathrm{E}}\\0\end{array}\right.\right)=\underbrace{^0\boldsymbol{T_1}^1\boldsymbol{T_2}\ldots^{n-1}\boldsymbol{T_\mathrm{E}}}_{^0\boldsymbol{T_\mathrm{E}}}\left(\begin{array}{c}0\\0\\0\\1\end{array}\right)-\underbrace{^0\boldsymbol{T_1}^1\boldsymbol{T_2}\ldots^{i-2}\boldsymbol{T_{i-1}}}_{^0\boldsymbol{T_{i-1}}}\left(\begin{array}{c}0\\0\\0\\1\end{array}\right)
  
  \end{aligned}
  $$

###### 解析雅克比矩阵 $\boldsymbol{J}_\mathrm{A}$

- 通过对正运动学公式进行偏微分推导而来

  $$
  \boldsymbol{x}_\mathrm{E} = \boldsymbol{f}(\boldsymbol{q})
  
  \\ \\
  
  \left.\boldsymbol{J}(\boldsymbol{q})=\frac{\partial\boldsymbol{f}(\boldsymbol{q})}{\partial\boldsymbol{q}}=\left(\begin{array}{ccc}\frac{\partial f_1}{\partial q_1}&\cdots&\frac{\partial f_1}{\partial q_n}\\\vdots&\ddots&\vdots\\\frac{\partial f_m}{\partial q_1}&\cdots&\frac{\partial f_m}{\partial q_n}\end{array}\right.\right)\in\mathbf{R}^{m\times n}
  $$
- 描述的末端速度为相对于基底坐标系的速度分量以及**选择的空间旋转方式的角速度**，比如欧拉角内旋表达 $R_x(\alpha)R_y(\beta)R_z(\gamma)$ （即RPY），所以正运动学公式 $\boldsymbol{f}$ 中需要包括从旋转矩阵 $\boldsymbol{R}$ 推导到欧拉角旋转角度 $\alpha、\beta、\gamma$ 的内容：

  $$
  \Delta\boldsymbol{x}_\mathrm{E} = 
  
  \begin{bmatrix}
  _{(0)}\boldsymbol{v}_\mathrm{E}  \\
  _{(0)}\boldsymbol{\omega}_\mathrm{E, RPY} 
  \end{bmatrix}
  
  =
  
  \begin{bmatrix}
  v_\mathrm{x} \\
  v_\mathrm{y} \\
  v_\mathrm{z} \\
  _{(0)}{\omega}_\mathrm{E,\alpha}\\
  _{(0)}{\omega}_\mathrm{E,\beta}\\
  _{(0)}{\omega}_\mathrm{E,\gamma}\\
  \end{bmatrix}
  $$
- 由于旋转速度的表达方式不同，解析雅克比矩阵 $\boldsymbol{J}_\mathrm{A}$ 与几何雅克比矩阵 $\boldsymbol{J}_\mathrm{G}$ 存在如下关系

  - 在某些特殊情况下，如平面连杆机构：

    $$
    \boldsymbol{J}_\mathrm{A} = \boldsymbol{J}_\mathrm{G}
    $$
  - 针对末端旋转为欧拉角内旋表达 $R_x(\alpha)R_y(\beta)R_z(\gamma)$ （即RPY）

    - 角速度向量转换为RPY表达有以下关系：

      $$
      \begin{align}
      _{(0)}\boldsymbol{\omega}_{\mathrm{E}} &= 
      \begin{bmatrix}
      \dot{\alpha} \\
      0 \\
      0
      \end{bmatrix}_{\mathrm{Roll}}
      +
      \boldsymbol{R}_x(\alpha) \cdot
      \begin{bmatrix}
      0 \\
      \dot{\beta} \\
      0
      \end{bmatrix}_{\mathrm{Pitch}}
      +
      \boldsymbol{R}_x(\alpha) \cdot \boldsymbol{R}_y(\beta) \cdot
      \begin{bmatrix}
      0 \\
      0 \\
      \dot{\gamma}
      \end{bmatrix}_{\mathrm{Pitch}} 
      
      \\ \\
      
      &=
      
      \underbrace{\begin{bmatrix}
      1 & 0 & \sin(\beta) \\
      0 & \cos(\alpha) & -\sin(\alpha)\cos(\beta) \\
      0 & \sin(\alpha) & \cos(\alpha)\cos(\beta)
      \end{bmatrix}}_{\boldsymbol{\Omega}_\text{RPY}}
      \cdot
      \begin{bmatrix}
      \dot{\alpha} \\
      \dot{\beta} \\
      \dot{\gamma}
      \end{bmatrix}_\mathrm{RPY}
      
      \end{align}
      $$
    - 则解析雅克比矩阵 $\boldsymbol{J}_\mathrm{A}$ 与几何雅克比矩阵 $\boldsymbol{J}_\mathrm{G}$ 转换关系如下：

      $$
      \begin{align}
      \boldsymbol{J}_\mathrm{G} &= 
      \begin{bmatrix}
      \boldsymbol{E} & \boldsymbol{0} \\
      \boldsymbol{0} & \boldsymbol{\Omega}
      \end{bmatrix} 
      \cdot
      \boldsymbol{J}_\mathrm{A}
      
      \\ \\
      \boldsymbol{J}_\mathrm{A} &= 
      \begin{bmatrix}
      \boldsymbol{E} & \boldsymbol{0} \\
      \boldsymbol{0} & \boldsymbol{\Omega}^{-1}
      \end{bmatrix} 
      \cdot
      \boldsymbol{J}_\mathrm{G}
      
      \end{align}
      $$
      

- 例：一个平面2轴机械臂的雅克比矩阵

  ![image-20240522172535167](robot_arm_note.assets/image-20240522172535167.png)

  ![image-20240522172446394](robot_arm_note.assets/image-20240522172446394.png)

  - 不涉及末端角速度的解析雅克比矩阵：

  ![image-20240522172515252](robot_arm_note.assets/image-20240522172515252.png)

  - 涉及末端角速度的几何雅克比矩阵，因为是平面连杆机构，所以与解析雅克比矩阵等效

  ![image-20240523153510969](robot_arm_note.assets/image-20240523153510969.png)

##### 雅克比矩阵的应用

- 用于机械臂轨迹插值以及逆运动学的数值解

  $$
  \begin{aligned}
  \boldsymbol{q}_{i+1}& =\boldsymbol{q}_i+\Delta\boldsymbol{q}_i \\
  &=\boldsymbol{q}_i+\boldsymbol{J}^{-1}(\boldsymbol{q}_i) \Delta\boldsymbol{x}_{\mathrm{E}, i} \\
  &=\boldsymbol{q}_{i}+\boldsymbol{J}^{-1}(\boldsymbol{q}_{i}) (\boldsymbol{x}_{\mathrm{E}, i+1}-\boldsymbol{x}_{\mathrm{E}, i}),
  \end{aligned}
  $$
- 用于计算机械臂的操控性以及奇异性分析

  - 操控性越高，同样关节变化造成的末端变换更大，同时末端误差也会越大
  - 针对满秩的方阵雅克比矩阵，通过对雅克比矩阵进行特征分解(ED: Egien Decomposition)；对于普通矩阵，进行奇异值分解（SVD: Singular Value Decompostion）（参考 [矩阵分解—特征值分解与奇异值分解](https://zhuanlan.zhihu.com/p/613284889)）获取各特征向量上的特征值 $\lambda_i$

    $$
    \text{ED: }&\quad  \boldsymbol{J} = \boldsymbol{B}\boldsymbol{D}\boldsymbol{B}^{-1} = 
    \boldsymbol{B}
    \begin{pmatrix}
    \lambda_1 & & 0 \\
     & \ddots & \\
     0 &&\lambda_n
    \end{pmatrix}
    \boldsymbol{B}^{-1}
    
    \\ \\
    
    \text{SVD: }&\quad \boldsymbol{J} = \boldsymbol{U}\boldsymbol{\Sigma}\boldsymbol{V}^{\text{T}} = 
    \boldsymbol{U}
    \begin{pmatrix}
    \lambda_1 & \cdots & 0 \\
    \vdots  & \ddots & \vdots \\
     0 &&\lambda_n \\
     0 & \cdots & 0
    \end{pmatrix}
    \boldsymbol{V}^{\text{T}}
    $$
  - 特征值 $\lambda_i$ 表示机械臂末端在对应的特征向量上的操控性或灵活度：

    - $|\lambda_i|$ 大表示大的操作性或灵活度
    - $|\lambda_i|$ 小表示小的操作性或灵活度
    - $|\lambda_i| = 0$ 表示在该方向上没有可操作性或灵活度，表示机械臂在该方向上损失了一个自由度，遭遇了奇异性问题。
  - 整体的操作性，注意当操作性为0时说明机械臂处于奇异性状态，机械臂末端损失了自由度，无法完成对应的位移。

    $$
    \mu(\boldsymbol{q}) = \left| \Pi_i^n\lambda_i \right| = \left| \text{det}(\boldsymbol{J}(\boldsymbol{q})) \right|
    $$

    - 机械臂在处于奇异性位置时的特点：

      - 此时雅克比矩阵的行列式为 0

        $$
        \left| \text{det}(\boldsymbol{J}(\boldsymbol{q})) \right| = 0
        $$
      - 损失自由度
      - 微小的末端位置改变，造成巨大的关节变化，造成关节速度突变
      - 微小的末端负载会造成巨大的关节受力
      - [What are robot singularities?](https://www.youtube.com/watch?v=lD2HQcxeNoA)
    - 例：平面三连杆机构的奇异性配置：

      - 非奇异性状态

      <img src="robot_arm_note.assets/image-20240524144413676.png" alt="image-20240524144413676" style="zoom: 50%;" />

      - 奇异性状态：通过计算雅克比矩阵的行列式为0的情况，发现当关节2的角度为0或180度时，该机构陷入奇异性状态，此时末端在沿着第一关节的方向损失一个自由度。

        ![image-20240524144540458](robot_arm_note.assets/image-20240524144540458.png)
    - 解决奇异性的方法

      - 轨迹优化
      - 关节冗余设置
      - 力学控制，避免过大的受力
- 用于描述末端负载与关节力矩之间的关系

  - 例：机械臂末端受到的力与力矩 $\boldsymbol{\mathcal{F}}$ 对末端产生微小变化 $\Delta \boldsymbol{x}_\text{E}$， 同时对关节产生力或力矩 $\boldsymbol{\tau}$ ，并发生形变 $\Delta\boldsymbol{q}$

  ![image-20240524140759045](robot_arm_note.assets/image-20240524140759045.png)

  - 通过虚功分析，可以得到以下关系：
    $$
    \boldsymbol{\mathcal{F}} = 
    \begin{pmatrix}
    \boldsymbol{F} \\
    \boldsymbol{M}
    \end{pmatrix}
    \in \boldsymbol{\mathbb{R}}^{6}
    
    \\
    
    \Delta \boldsymbol{x}_\text{E}^\text{T} \boldsymbol{\mathcal{F}} = \Delta\boldsymbol{q}^\text{T} \boldsymbol{\tau} 
    
    \\
    
    {(\boldsymbol{J}(\boldsymbol{q})\Delta{\boldsymbol{q}})}^\text{T} \boldsymbol{\mathcal{F}} = \Delta\boldsymbol{q}^\text{T} \boldsymbol{\tau} 
    
    \\
    
    \boldsymbol{\tau} = \boldsymbol{J}^{\text{T}}(\boldsymbol{q})\boldsymbol{\mathcal{F}}
    
    \\
    
    \boldsymbol{\mathcal{F}} =( \boldsymbol{J}^{\text{T}}(\boldsymbol{q}))^{-1}\boldsymbol{\tau}
    $$
- 用于刚性分析

  - 假设每个关节的弹性模量为 $k_i$，整个系统的关节弹性矩阵为 $\boldsymbol{K} = \text{diag}(k_1, \cdots, k_n) = \text{diag}(\boldsymbol{k})$，则可以得出整个系统的末端弹性矩阵 $\boldsymbol{C}_\text{x}$:

  $$
  \begin{align}
  \boldsymbol{\tau} &= \boldsymbol{K} \Delta\boldsymbol{q}
  \\
  \boldsymbol{J}^\text{T}(\boldsymbol{q})\boldsymbol{\mathcal{F}} &= \boldsymbol{K}\Delta{\boldsymbol{q}}
  \\
  \boldsymbol{J}^\text{T}(\boldsymbol{q})\boldsymbol{\mathcal{F}} &= \boldsymbol{K}\boldsymbol{J}^{-1}(\boldsymbol{q})\Delta\boldsymbol{x}_\text{E}
  \\
  \Delta\boldsymbol{x}_\text{E} &= \underbrace{\boldsymbol{J}(\boldsymbol{q})\boldsymbol{K}^{-1}\boldsymbol{J}^\text{T}(\boldsymbol{q})}_{\boldsymbol{C}_\text{x}}\boldsymbol{\mathcal{F}}
  \\
  \boldsymbol{C}_\text{x} &= \boldsymbol{J}(\boldsymbol{q})\boldsymbol{K}^{-1}\boldsymbol{J}^\text{T}(\boldsymbol{q})
  
  \\
  \Delta\boldsymbol{x}_\text{E} &= \boldsymbol{C}_\text{x}\boldsymbol{\mathcal{F}}
  \\
  \boldsymbol{C}_\text{x} &= \boldsymbol{C}_\text{x}^\text{T}
  
  \end{align}
  $$

  - 对末端的弹性矩阵 $\boldsymbol{C}_\text{x}$ 进行特征分解，有以下特点：
    - 特征值越大，对应的特征向量表示了在该方向上其末端弹性越大，其刚性越小
    - 特征值越小，对应的特征向量表示了在该方向上其末端弹性越小，其刚性越大

#### 冗余机械臂

- 冗余机械臂定义

  $$
  \dim(\boldsymbol{q}) > \dim(\boldsymbol{x}_\text{E})
  $$
- 冗余机械臂的雅克比矩阵:

  $$
  \boldsymbol{J}_\text{reduntant} \in \mathbb{R}^{m\times n}, m < n
  $$
- 求逆：

  $$
  \boldsymbol{J}^{\dagger}=\begin{cases}(\boldsymbol{J}^{T}\boldsymbol{J})^{-1}\boldsymbol{J}^{T},&n<6\\\boldsymbol{J}^{-1},&n=6\\\boldsymbol{J}^{T}(\boldsymbol{J}\boldsymbol{J}^{T})^{-1},&n>6&&\end{cases}
  $$
- 操控性计算，以及奇异性判断

  $$
  \mu(\boldsymbol{q})=\det\left(\boldsymbol{J}(\boldsymbol{q})\boldsymbol{J}^\mathrm{T}(\boldsymbol{q})\right)
  $$
- 用途

  - 避障

    - 例：cpprobotics: Demo04
  - 避免奇异性
  - 优化关节角度配置，零空间运动

    - 末端位姿不变，改变关节配置，有无数解
  - 例：

    <img src="robot_arm_note.assets/image-20240524165530033.png" alt="image-20240524165530033" style="zoom:50%;" />
- 零空间运动的推导：

  - 视频演示：[7 DOF Robotic Arm Null space motions](https://www.youtube.com/watch?v=XSsroN7dUlM)
  - 通过对冗余机械臂的雅克比矩阵进行高斯变换，可以求解当前末端变化对应的关节角度变化的特解 (particular solution) 和通解 (homogenous solution)。当此刻关节角度的变化满足**通解**的条件时，将不会对末端产生影响。

  $$
  \dot{\boldsymbol{x}}_\mathrm{E}=\boldsymbol{J}(\boldsymbol{q})\dot{\boldsymbol{q}},\quad\mathrm{with}\quad\boldsymbol{J}(\boldsymbol{q})\in \boldsymbol{\mathbb{R}}^{m\times n}(m<n)\quad\mathrm{und}\quad\mathrm{Rang}\left(\boldsymbol{J}(\boldsymbol{q})\right)=m
  
  \\ \\
  
  \text{after Gauss transformation}:
  
  \\
  
  \left.\boldsymbol{J}^*=\left(\begin{array}{cccc}\boxed{j_{11}}&j_{12}&0&0\\0&0&\boxed{j_{23}}&0\\0&0&0&\boxed{j_{34}}\end{array}\right.\right)
  
  \\
  
  \boldsymbol{\delta} = \boldsymbol{J}^*\dot{\boldsymbol{q}}
  
  \\
  
  \left(\begin{array}{cccc|c}\boxed{j_{11}}&j_{12}&0&0&\delta_1\\0&0&\boxed{j_{23}}&0&\delta_2\\0&0&0&\boxed{j_{34}}&\delta_3\end{array}\right)\quad(\equiv\boldsymbol{J}^*\dot{\boldsymbol{q}}=\boldsymbol{\delta})
  
  \\
  
  \dot{\boldsymbol{q}}=\underbrace{{\begin{pmatrix}\frac{\delta_{1}}{j_{11}}\\0\\\frac{\delta_{2}}{j_{23}}\\\frac{\delta_{3}}{j_{34}}\end{pmatrix}}}_{{\text{particular solution}}}+\underbrace{{\lambda\begin{pmatrix}-\frac{j_{12}}{j_{11}}\\1\\0\\0\end{pmatrix}}}_{{\text{homogenous solution}}}=\dot{\boldsymbol{q}}_{{\mathrm{p}}}+\dot{\boldsymbol{q}}_{{\mathrm{h}}}
  
  \\
  
  \boldsymbol{J}^*\dot{\boldsymbol{q}}_\text{h} = \boldsymbol{0}
  $$

### 机械臂运动规划

#### 路径规划与轨迹规划概念

- 路径：只包含几何意义上的点位数据

- 路径规划：从所有可能的几何路径集合中找到所需的路径。该路径描述了从起始点到终点。在期间无需考虑时间因素。

- 轨迹：包含点位数据以及时间相关信息

- 轨迹规划：路径规划的下一阶段，考虑该系统如何以合适的速度、加速度等运动学约束，将路径规划后的离散点位进行链接。相比路径规划，轨迹规划更多的考虑时间、速度、加速度等与运动性能相关的指标。



#### 机械臂中的路径规划

- 关节空间路径规划

  - 在关节空间中进行简单直线规划，在笛卡尔空间中并不是直线运动
  - 在关节空间进行规划，可以避免奇异性问题，计算消耗小（无需逆解），但在规划完成之前无法预见笛卡尔空间中的路径，需要关注潜在的碰撞问题。

  <img src="robot_arm_note.assets/image-20240617170451017.png" alt="image-20240617170451017" style="zoom: 67%;" />

- 笛卡尔空间路径规划

  - 在笛卡尔空间中进行直线运动，在关节空间中并不是直线规划。
  - 在笛卡尔空间进行规划，可以直观设置末端路径，但无法完全避免关节奇异性。获取的路径在转换为关节轨迹时需要进行逆解算、轨迹平滑处理等操作。

  <img src="robot_arm_note.assets/image-20240617170511048.png" alt="image-20240617170511048" style="zoom:67%;" />

- 智能路径规划

  - 基于采样

    - 基于采样的方法不需要完整表示位型空间，所以非常适合高维和复杂环境问题，且相比 search-based （A* D*等）方法，能更有效的处理非凸和杂乱的环境。但是 sampling-based 方法只具有概率完备性，不一定总能找到最优解，解的质量取决于采样点的数量和分布；很难处理动力学等复杂约束问题；不能保证解的有一致性，比如相似的起点或者终点状态，生成的解不能保证是相似的，无法对规划结果进行预判，这就使得自动规划的机器人无法进入极端追求稳定性的工业领域。

    - 例：PRM，RRT，RRT*，OMPL（MoveIt 默认机械臂路径规划库）

      - [RRT,RRT-Star 和 RRT-Star-Smart](https://zhuanlan.zhihu.com/p/161829703)
      - [Rapidly exploring random tree](https://en.wikipedia.org/wiki/Rapidly_exploring_random_tree#cite_note-incremental-8)

    - demo: cpprobotics/ArmNavigation/RRTStarSevenJointArmControl (在7轴机械臂关节空间中进行RRT避障寻路)

      <img src="robot_arm_note.assets/Peek 2024-06-11 17-57.gif" alt="Peek 2024-06-11 17-57" style="zoom:50%;" />

  - 基于优化

    - 将运动规划问题表述为一个优化问题，目标是在满足机器人环境约束和动力学约束下，找到优化某个成本函数的轨迹。可以设计各种成本函数来实现不同的优化目标，比如最短路径、最优时间、最优能量等。基于优化的方法可以找到高质量的解决方案，且解具有一致性，可以处理机器人动力学和环境的复杂约束。但是 optimization-based 方法计算成本很高，尤其是对于高维系统；需要准确的机器人动力学和环境模型，这在实践中可能很难得到；在复杂环境下容易陷入局部最优解。
    - 例：CHOMP、STOMP

  

#### 机械臂中的轨迹规划

- 轨迹坐标

  - 为了简化在时间对轨迹规划的影响，引入轨迹坐标，
    $$
    s, 0<s<1, \\
    \boldsymbol{x}(t) = \boldsymbol{x}(s(t)) = \boldsymbol{x}(s)
    $$

##### 点对点轨迹规划 （PTP，PointToPoint）

- 该规划通常应用于**关节空间规划**，即主要关注关节空间轨迹插值方法

- 给定时间，初始关节状态，目标关节状态（通过逆运动学获取），计算各关节加速-减速的方式

- 简单的关节空间轨迹插值方法：

  - 速度梯形图（C1连续） VS 加速度梯形图（C2连续）

    <img src="robot_arm_note.assets/image-20240617182945319.png" alt="image-20240617182945319" style="zoom: 67%;" />

    

##### 连续路径的轨迹规划（CP Continuous-Path）

- 概念

  - 该规划通常应用于**笛卡尔空间规划**，主要关注笛卡尔空间中机械臂末端的位姿、位姿速度、位姿加速度的插值问题
  - 该规划方法一般需要满足笛卡尔空间中的特定规则，如直线轨迹规划、圆形轨迹规划，或者需要通过预先给定的路点、避免障碍物等
  - 在笛卡尔空间中的路径规划后，一般需要通过逆运动学计算各个点位在关节空间中的配置，并进行关节空间的轨迹规划。
  - 关节空间轨迹规划完成后，还可以进一步转换为笛卡尔轨迹，用于进行障碍物判断，或与目标笛卡尔空间轨迹规划进行对比。

- 线段轨迹规划
  $$
  \boldsymbol{x}_\mathrm{E}(t)=\boldsymbol{x}_\mathrm{E,S}+s(t)\left(\boldsymbol{x}_\mathrm{E,Z}-\boldsymbol{x}_\mathrm{E,S}\right)
  $$

- 圆弧轨迹规划
  $$
  \left._{(0)}\boldsymbol{x}_\mathrm{E}(t)=_{(0)}\boldsymbol{x}_\mathrm{M}+{}^0\boldsymbol{R}_\mathrm{M} \left(\begin{array}{c}\cos(s(t) \varphi_\mathrm{max})\\\sin(s(t) \varphi_\mathrm{max})\\0\end{array}\right.\right)\underbrace{\left\|_{(\mathrm{M})}\boldsymbol{x}_\mathrm{P_1}-_{(\mathrm{M})}\boldsymbol{x}_\mathrm{M}\right\|_2}_{=r}
  $$
  ![image-20240618094701372](robot_arm_note.assets/image-20240618094701372.png)



##### 轨迹插值

- 实际应用中，机械臂的轨迹通常由多段组成，当运动方向发生突变时，会产生巨大的加速度，造成运行的不稳定，因此需要针对多段轨迹进行平滑插值处理。

- 轨迹平滑插值处理在关节空间与笛卡尔空间中均有运用。

- 例：平滑处理两段轨迹构成的直角，将速度的突变用梯形速度变化代替

  ![image-20240618103553783](robot_arm_note.assets/image-20240618103553783.png)

  ![image-20240618103614637](robot_arm_note.assets/image-20240618103614637.png)

###### 位移变化的轨迹插值

- 常见算法
  - 梯形速度插值
  - 抛物线拟合线性插值
  - 三/五次多项式插值
  - 三/五次样条插值
  - 贝塞尔曲线
  - TOPP
- 参考
  - [关节空间路径/轨迹规划算法](https://zhuanlan.zhihu.com/p/446463459)
  - [二次与三次B样条曲线c++实现](https://blog.csdn.net/jiangjjp2812/article/details/100176547?utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-3.no_search_link&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-3.no_search_link)
  - [B样条曲线](https://zhuanlan.zhihu.com/p/445466952)

###### 旋转变化的轨迹插值

- 欧拉角插值
  - 通过对欧拉角插值，获取其对应的旋转角度、旋转角速度、旋转角加速度

- 轴角旋转插值
  - 将目标旋转矩阵通过轴角方式进行表达
  - 旋转轴在插值过程中保持不变
  - 旋转角在此过程中从初始角度旋转到目标角度

- 四元数插值
  - LERP 线性插值
    $$
    v=\text{Lerp}(v_0,v_1,t)=(1-t)v_0+tv_1
    $$
    
    - 用于在两个点之间进行直线插值。适用于数值、位置、颜色等，可以用于平滑过渡
    
    - 对于四元数来说，LERP 每帧得出的旋转结果是不均匀
    
      <img src="robot_arm_note.assets/7c6180ec1f6803707da58e59e807c88b.png" alt="img" style="zoom:50%;" />
    
  - SLERP 球面线性插值
    $$
    \boldsymbol{q}_t=\text{Slerp}(\boldsymbol{q}_0,\boldsymbol{q}_1,t)=\frac{\sin((1-t)\theta)}{\sin\theta}\boldsymbol{q}_0+\frac{\sin(t\theta)}{\sin\theta}\boldsymbol{q}_1 \\
    \theta=\cos^{-1}(\boldsymbol{q}_0\cdot \boldsymbol{q}_1)
    $$
    

    - 用于在两个方向之间进行插值，沿球面弧线插值。常用于旋转插值

    - 对于四元数来说，SLERP 每帧得出的旋转结果均匀

      <img src="robot_arm_note.assets/5da4c9de0f0e8c5d02247dc1c2ffae49.png" alt="img" style="zoom:50%;" />

      <img src="robot_arm_note.assets/image-20240619110011990.png" alt="image-20240619110011990" style="zoom: 67%;" />

  - 参考
    - [WIKI-SLERP](https://en.wikipedia.org/wiki/Slerp)
    - [线性插值解释](https://www.zhihu.com/question/353481946/answer/3244220249)
  - 通过四元数对方向进行插值计算的优点
    - 计算方便
    - 区分位移和方向的插值
    - 无奇异性

###### 旋转+位移的轨迹插值

$$
^\mathrm{s}\boldsymbol{T}_\mathrm{E}(s(t),t_\mathrm{rel})=
\left(\begin{array}{c|c}
^\mathrm{s}\boldsymbol{R}_\mathrm{E}(t_\mathrm{rel}) & _\mathrm{(S)}\boldsymbol{r}_\mathrm{SE}(s(t)) \\ \hline
0\ 0\ 0&1
\end{array}\right)

\\ \\

_{(0)}\boldsymbol{r_\mathrm{SE}}=s(t)\begin{pmatrix}_{(0)}\boldsymbol{r_\mathrm{Z}}-_{(0)}\boldsymbol{r_\mathrm{S}}\end{pmatrix}

\\ \\

_{(\mathrm{S})}\boldsymbol{r}_\mathrm{SE}={}^\mathrm{S}\boldsymbol{R}_{0 (0)}\boldsymbol{r}_\mathrm{SE}={}^\mathrm{S}\boldsymbol{R}_0 s(t)\left({}_{(0)}\boldsymbol{r}_\mathrm{Z}-{}_{(0)}\boldsymbol{r}_\mathrm{S}\right)
$$

<img src="robot_arm_note.assets/image-20240619110559835.png" alt="image-20240619110559835" style="zoom: 80%;" />





### 机械臂动力学

- 概念
  - 用于计算特定运动所需的机器人驱动力和/或扭矩
  - 动力学包含对有质量的物体在力和力矩影响下运动的研究。因此，动力学描述了运动变量和力变量之间的联系。

#### 正动力学

- 正动力学公式：
  $$
  \ddot{\boldsymbol{q}}(t)=\boldsymbol{f}_\mathrm{D}(\boldsymbol{q}(t),\dot{\boldsymbol{q}}(t),\boldsymbol{\tau}(t),\boldsymbol{\mathcal{F}}(t))
  $$

  - 关节角度、角速度、角加速度：$\boldsymbol{q}, \dot{\boldsymbol{q}}, \ddot{\boldsymbol{q}}$
  - 关节力矩：$\boldsymbol{\tau}$
  - 末端负载：$\boldsymbol{\mathcal{F}}$

- 应用

  - 机械臂运动仿真，需要精确的模型参数



#### 逆动力学

##### 概念

- 逆动力学公式
  $$
  \boldsymbol{\tau}(t)=\boldsymbol{g}_\mathrm{D}(\boldsymbol{q}(t),\dot{\boldsymbol{q}}(t),\ddot{\boldsymbol{q}}(t),\boldsymbol{\mathcal{F}}(t))
  $$

  - 关节力矩：$\boldsymbol{\tau}$
  - 关节角度、角速度、角加速度：$\boldsymbol{q}, \dot{\boldsymbol{q}}, \ddot{\boldsymbol{q}}$
  - 末端负载：$\boldsymbol{\mathcal{F}}$

- 逆动力学一般形式
  $$
  \boldsymbol{\tau}=\boldsymbol{M}(q)\ddot{\boldsymbol{q}}+\boldsymbol{c}(\boldsymbol{q},\dot{\boldsymbol{q}})+\boldsymbol{g}(\boldsymbol{q})+\boldsymbol{h}(\boldsymbol{q},\dot{\boldsymbol{q}})
  $$

  - 关节力矩：$\boldsymbol{\tau}, n\times 1$
  - 惯性项：$\boldsymbol{M}, n\times n$
  - 离心力项：$\boldsymbol{c}, n\times 1$
  - 重力项：$\boldsymbol{g}, n\times 1$
  - 非线性项（摩擦力等）：$\boldsymbol{h},  n \times 1$

- 逆动力学求解方法
  - 牛顿欧拉法
    - 分段分析机械臂各关节的受力情况
  - 拉格朗日法
    - 通过势能守恒与做功分析进行动力学建模





##### 牛顿欧拉法

1. 分离机械臂各关节，确认各关节段的重心 $S_i$ 

2. 确认基于基底坐标系 $\text{(KS)}_0$ 的重心位移向量 $_{(0)}\boldsymbol{r}_{S_i}$、重心位移速度 $_{(0)}\boldsymbol{\dot{r}}_{S_i}$ 以及重心位移加速度 $_{(0)}\boldsymbol{\ddot{r}}_{S_i}$

   ![](robot_arm_note.assets/image-20240619135716381.png)

3. 确认基于基底坐标系 $\text{(KS)}_0$ 各关节段的的角速度 $_{(0)}\omega_i$ 以及角加速度 $_{(0)}\dot{\omega}_i$

   - 方法1：

   $$
   \left.\boldsymbol{S}(_{(0)}\boldsymbol{\omega}_i)=_{(0)}\left(\begin{array}{ccc}0&-\omega_{z_i}&\omega_{y_i}\\\omega_{z_i}&0&-\omega_{x_i}\\-\omega_{y_i}&\omega_{x_i}&0\end{array}\right.\right)={}^0\dot{\boldsymbol{R}}_i{}^0\boldsymbol{R}_i^\mathrm{T}
   
   \\ \\
   
   _{(0)}\boldsymbol{\omega}_i=(_{(0)}\omega_{x_i},_{(0)}\omega_{y_i},_{(0)}\omega_{z_i})^\mathrm{T}
   $$

   - 方法2：
     $$
     _{(0)}\boldsymbol{\omega}_0  = _{(0)}\dot{\boldsymbol{\omega}}_0 = \boldsymbol{0}, _{(0)}\dot{\boldsymbol{r}}_0  = _{(0)}\ddot{\boldsymbol{r}}_0 = \boldsymbol{0}
     $$

     - 针对旋转关节
       $$
       \begin{aligned}
       &_{(0)}\boldsymbol{\omega}_{i} =
       _{(0)}\boldsymbol{\omega}_{i-1}
       + 
       \dot{q}_{i} {_{(0)}\boldsymbol{e}_{z}^{(i-1)}}, 
       
       \\ 
       
       &_{(0)}\dot{\boldsymbol{\omega}}_i =
       _{(0)}\dot{\boldsymbol{\omega}}_{i-1}
       +
       \ddot{q}_{i} {_{(0)}\boldsymbol{e}_{z}^{(i-1)}}
       +
       \dot{q}_{i} {_{(0)}\boldsymbol{\omega}_{i-1}}
       \times
       _{(0)}\boldsymbol{e}_z^{(i-1)}, 
       
       \\
       
       & _{(0)}\boldsymbol{r}_{i} =
       _{(0)}\boldsymbol{r}_{i-1}
       +
       _{(0)}\boldsymbol{r}_{i-1,i}, 
       
       \\
       
       &_{(0)}\dot{\boldsymbol{r}}_i =
       _{(0)}\dot{\boldsymbol{r}}_{i-1}
       +
       _{(0)}\boldsymbol{\omega}_i
       \times
       _{(0)}\boldsymbol{r}_{i-1,i}, 
       
       \\
       
       &_{(0)}\ddot{\boldsymbol{r}}_i =
       _{(0)}\ddot{\boldsymbol{r}}_{i-1}
       +
       _{(0)}\dot{\boldsymbol{\omega}}_i
       \times
       _{(0)}\boldsymbol{r}_{i-1,i}
       +
       _{(0)}\boldsymbol{\omega}_i
       \times
       \left(
       _{(0)}\boldsymbol{\omega}_i
       \times
       _{(0)}\boldsymbol{r}_{i-1,i}
       \right) 
       
       \end{aligned}
       $$
       
     - 针对位移关节
       $$
       \begin{aligned}
       &_{(0)}\boldsymbol{\omega}_{i} =_{(0)}\boldsymbol{\omega}_{i-1}, 
       
       \\
       
       &_{(0)}\dot{\boldsymbol{\omega}}_{i} =_{(0)}\dot{\boldsymbol{\omega}}_{i-1}, 
       
       \\
       
       &_{(0)}\boldsymbol{r}_{i} =_{(0)}\boldsymbol{r}_{i-1}+_{(0)}\boldsymbol{r}_{i-1,i}, 
       
       \\
       
       &_{(0)}\dot{\boldsymbol{r}}_{i} =_{(0)}\dot{\boldsymbol{r}}_{i-1}+_{(0)}\boldsymbol{\omega}_{i}\times_{(0)}\boldsymbol{r}_{i-1,i}+\dot{q}_{i } {_{(0)}}\boldsymbol{e}_{z}^{(i-1)} 
       
       \\
       
       &
       \begin{aligned}
       _{(0)}\ddot{\boldsymbol{r}}_{i} =
       &_{(0)}\ddot{\boldsymbol{r}}_{i-1}
       +
       \ddot{q}_{i}{_{(0)}}\boldsymbol{e}_{z}^{(i-1)}
       +
       _{(0)}\boldsymbol{\omega}_{i}
       \times
       \left(
       _{(0)}\boldsymbol{\omega}_{i}
       \times
       _{(0)}\boldsymbol{r}_{i-1,i}
       \right)
       + \\
       &2 \dot{q}_{i }{_{(0)}}\boldsymbol{\omega}_{i}
       \times
       _{(0)}\boldsymbol{e}_{z}^{(i-1)}
       +
       _{(0)}\dot{\boldsymbol{\omega}}_{i}
       \times
       _{(0)}\boldsymbol{r}_{i-1,i}
       \end{aligned}
       
       \end{aligned}
       $$
   
   
   
4. 确认基于基底坐标系 $\text{(KS)}_0$ 各关节段的质量惯性张量 $_{(0)}\boldsymbol{J}_i^{S_i}$，并分析各关节段之间的动量角动量守恒

   - 通过CAD获取各关节段在自身坐标系下的质量惯性张量
     $$
     \left._{\left(i\right)}\boldsymbol{J}_i^{\left(\mathcal{S}_i\right)}=\left(i\right)\left(\begin{array}{ccc}J_{xx_i}^{\left(\mathcal{S}_i\right)}&J_{xy_i}^{\left(\mathcal{S}_i\right)}&J_{xz_i}^{\left(\mathcal{S}_i\right)},\\J_{yx_i}^{\left(\mathcal{S}_i\right)}&J_{yy_i}^{\left(\mathcal{S}_i\right)}&J_{yz_i}^{\left(\mathcal{S}_i\right)},\\J_{zx_i}^{\left(\mathcal{S}_i\right)}&J_{zy_i}^{\left(\mathcal{S}_i\right)}&J_{zz_i}^{\left(\mathcal{S}_i\right)}.\end{array}\right.\right)=\mathrm{konst.}
     $$

   - 转换到基底坐标系下
     $$
     _{(0)}\boldsymbol{J}_{i}^{{(\mathcal{S}_{i})}}=\underbrace{{^{0}\boldsymbol{R}_{i\mathrm{~(}i)}\boldsymbol{J}_{i}^{{(\mathcal{S}_{i})\mathrm{~0}}}\boldsymbol{R}_{i}^{{\mathrm{T}}}}}_{{\text{Basiswechsel}}}
     $$

   - 列出各关节动量/角动量守恒

     <img src="robot_arm_note.assets/image-20240619150230212.png" alt="image-20240619150230212" style="zoom:80%;" />
     $$
     \begin{aligned}
     
     &
     \begin{aligned}
     _{(0)}\boldsymbol{F}_{i}&=
     m_{i} {_{(0)}}\ddot{\boldsymbol{r}}_{\mathrm{S}_{i}} 
     \\
     &=
     _{(0)}\boldsymbol{F}_{i-1,i}
     -
     _{(0)}\boldsymbol{F}_{i,i+1}
     +
     m_{i}{_{(0)}}\boldsymbol{g}
     
     \end{aligned}
     
     \\ \\
     
     &
     \begin{aligned}
     _{(0)}\boldsymbol{M}_i^{(\mathrm{S}_i)}
     &=
     {}_{(0)}\boldsymbol{J}_i^{(\mathrm{S}_i)}{}_{(0)}\dot{\boldsymbol{\omega}}_i+
     {}_{(0)}\boldsymbol{\omega}_i
     \times
     \left({}_{(0)}\boldsymbol{J}_i^{(\mathrm{S}_i)}{}_{(0)}\boldsymbol{\omega}_i\right)
     \\&=
     {}_{(0)}\boldsymbol{M}_{i-1,i}-
     {}_{(0)}\boldsymbol{M}_{i,i+1}+{}_{(0)}\boldsymbol{r}_{i,\mathrm{S}_i}\times
     {}_{(0)}\boldsymbol{F}_{i,i+1}-
     {}_{(0)}\boldsymbol{r}_{i-1,\mathrm{S}_i}\times
     {}_{(0)}\boldsymbol{F}_{i-1,i}
     \end{aligned}
     
     \end{aligned}
     $$
     

5. 从末端关节段向前递归计算驱动力/力矩

   - 关节承受的力
     $$
     \begin{aligned}
     &m_{n (0)}\ddot{\boldsymbol{r}}_{\mathrm{S}_{n}}={}_{(0)}\boldsymbol{F}_{n-1,n}-\overbrace{{_{(0)}}\boldsymbol{F}_{n,n+1}}^{\text{externe Kraft}}+m_{n }{_{(0)}}\boldsymbol{g} \\ 
     &\Rightarrow 
     _{(0)}\boldsymbol{F}_{n-1,n}=m_{n (0)}\ddot{\boldsymbol{r}}_{\mathrm{S}_n}+_{(0)}\boldsymbol{F}_{n,n+1}-m_{n }{_{(0)}}\boldsymbol{g}, 
     
     \\ \\
     
     &m_{n-1 (0)}\ddot{\boldsymbol{r}}_{\mathrm{S}_{n-1}}=_{(0)}\boldsymbol{F}_{n-2,n-1}-_{(0)}\boldsymbol{F}_{n-1,n}+m_{n-1}{_{(0)}}\boldsymbol{g} \\
     &\Rightarrow
     _{(0)}\boldsymbol{F}_{n-2,n-1}=m_{n-1 (0)}\ddot{\boldsymbol{r}}_{\mathrm{S}_{n-1}}+_{(0)}\boldsymbol{F}_{n-1,n}-m_{n-1}{_{(0)}}\boldsymbol{g}, 
     
     \\ \\
     &m_{n-2}{_{(0)}}\ddot{\boldsymbol{r}}_{\mathrm{S}_{n-2}}={}_{(0)}\boldsymbol{F}_{n-3,n-2}-{}_{(0)}\boldsymbol{F}_{n-2,n-1}+m_{n-2 (0)}\boldsymbol{g} \\
     &\Rightarrow
     _{(0)}\boldsymbol{F}_{n-3,n-2}=\ldots,
     \end{aligned}
     $$
     
- 关节承受的力矩
     $$
     \begin{aligned}
     &{_{(0)}}\boldsymbol{J}_{n}^{(\text{S}_n)} 
     {_{(0)}}\dot{\boldsymbol{\omega}}_{n} +
     {_{(0)}}\boldsymbol{\omega}_{n} \times
     \left(
     {_{(0)}}\boldsymbol{J}_{n}^{(\text{S}_n)} 
     {_{(0)}}\boldsymbol{\omega}_{n}
     \right)
     
     \\ &= 
     {_{(0)}}\boldsymbol{M}_{n-1, n} - 
     \underbrace{{_{(0)}}\boldsymbol{M}_{n, n+1}}_{\text{extern Moment}} +
     {_{(0)}}\boldsymbol{r}_{n, \text{S}_n} \times
     \underbrace{{_{(0)}}\boldsymbol{F}_{n, n+1}}_{\text{extern Force}} -
     {_{(0)}}\boldsymbol{r}_{n-1, \text{S}_n} \times
     {_{(0)}}\boldsymbol{F}_{n-1, n}
     \\ &\Rightarrow 
     {_{(0)}}\boldsymbol{M}_{n-1, n} = \cdots
     
     \\ \\
     
     &{_{(0)}}\boldsymbol{J}_{n-1}^{(\text{S}_{n-1})} 
     {_{(0)}}\dot{\boldsymbol{\omega}}_{n-1} +
     {_{(0)}}\boldsymbol{\omega}_{n-1} \times
     \left(
     {_{(0)}}\boldsymbol{J}_{n-1}^{(\text{S}_{n-1})} 
     {_{(0)}}\boldsymbol{\omega}_{n-1}
     \right)
     
     \\ &= 
     {_{(0)}}\boldsymbol{M}_{n-2, n-1} - 
     {{_{(0)}}\boldsymbol{M}_{n-1, n}} +
     {_{(0)}}\boldsymbol{r}_{n-1, \text{S}_{n-1}} \times
     {{_{(0)}}\boldsymbol{F}_{n-1, n}} -
     {_{(0)}}\boldsymbol{r}_{n-2, \text{S}_{n-1}} \times
     {_{(0)}}\boldsymbol{F}_{n-2, n-1}
     \\ &\Rightarrow 
     {_{(0)}}\boldsymbol{M}_{n-2, n-1} = \cdots
     
     \end{aligned}
     $$
   
- 关节驱动力（位移关节）
     $$
     \tau_i = _{(0)}\boldsymbol{M}_{i-1, i}^{\text{T}} {_{(0)}}\boldsymbol{e}_z^{i-1}
     $$
     
   - 关节驱动力矩（旋转关节）
  $$
     \tau_i = {_{(0)}}\boldsymbol{F}_{i-1, i}^{\text{T}}{_{(0)}}\boldsymbol{e}_z^{(i-1)}
  $$
  
   
  

6. 整理成矩阵形式
   $$
   \boldsymbol{\tau}=\boldsymbol{M}(q)\ddot{\boldsymbol{q}}+\boldsymbol{c}(\boldsymbol{q},\dot{\boldsymbol{q}})+\boldsymbol{g}(\boldsymbol{q})+\boldsymbol{h}(\boldsymbol{q},\dot{\boldsymbol{q}})
   $$
   



##### 拉格朗日法

1. 分离机械臂各关节，确认各关节段的重心 $S_i$ 

2. 确认基于基底坐标系 $\text{(KS)}_0$ 的重心位移向量 $_{(0)}\boldsymbol{r}_{S_i}$、重心位移速度 $_{(0)}\boldsymbol{\dot{r}}_{S_i}$ 以及重心位移加速度 $_{(0)}\boldsymbol{\ddot{r}}_{S_i}$

3. 确认基于基底坐标系 $\text{(KS)}_0$ 各关节段的的角速度 $_{(0)}\omega_i$ 以及角加速度 $_{(0)}\dot{\omega}_i$

4. 确认基于基底坐标系 $\text{(KS)}_0$ 各关节段的质量惯性张量 $_{(0)}\boldsymbol{J}_i^{S_i}$，并分析各关节段之间的动量角动量守恒

   （1-4步与牛顿欧拉法相同）

5. 计算势能 $U$ 与动能 $T$
   $$
   \begin{aligned}
   T &= \frac{1}{2}\sum_{i=1}^{n}
   \left(
   m_i
   \underbrace{
   {_{(0)}}\dot{\boldsymbol{r}}_{\text{S}_i}^{\text{T}}
   {_{(0)}}\dot{\boldsymbol{r}}_{\text{S}_i}
   }_{\text{Translation energy}}
   
   +
   
   \underbrace{
   {_{(0)}}\boldsymbol{\omega}_i^{\text{T}}
   {_{(0)}}\boldsymbol{J}_i^{\text{S}_i}
   {_{(0)}}\boldsymbol{\omega}_i
   }_{\text{Rotation energy}}
   
   \right)
   
   \\ \\
   
   U &=
   -\sum_{i=1}^n
   \left(
   m_i
   {_{(0)}}\boldsymbol{g}^{\text{T}}
   {_{(0)}}\boldsymbol{r}_{\text{S}_i}
   \right)
   
   \end{aligned}
   $$
   
6. 运用拉格朗日公式第二形式
   $$
   \begin{aligned}
   L &= T - U
   
   \\ \\
   \boldsymbol{\tau} &=
   \frac{\text{d}}{\text{d}t}
   \frac{\part L}{\part \dot{\boldsymbol{q}}} - 
   \frac{\part L}{\part {\boldsymbol{q}}} 
   
   \end{aligned}
   $$
   
7. 整理成矩阵形式
   $$
   \boldsymbol{\tau}=\boldsymbol{M}(q)\ddot{\boldsymbol{q}}+\boldsymbol{c}(\boldsymbol{q},\dot{\boldsymbol{q}})+\boldsymbol{g}(\boldsymbol{q})+\boldsymbol{h}(\boldsymbol{q},\dot{\boldsymbol{q}})
   $$
   
8. 考虑末端力与力矩（通过雅克比矩阵）
   $$
   \boldsymbol{\tau}_\text{extern}(\boldsymbol{q}) = \boldsymbol{J}^{\text{T}}(\boldsymbol{q})\boldsymbol{\mathcal{F}}
   
   \\ \\
   
   \boldsymbol{\tau}=\boldsymbol{M}(q)\ddot{\boldsymbol{q}}+\boldsymbol{c}(\boldsymbol{q},\dot{\boldsymbol{q}})+\boldsymbol{g}(\boldsymbol{q}) - \boldsymbol{\tau}_\text{extern}(\boldsymbol{q})
   $$





##### 考虑电机减速比与摩擦力

- 摩擦力模型
  $$
  \boldsymbol{h}(\boldsymbol{q},\dot{\boldsymbol{q}})=
  \operatorname{diag}(\operatorname{sign}(\dot{\boldsymbol{q}}))\boldsymbol{r}_\mathrm{c}+\operatorname{diag}(\dot{\boldsymbol{q}})\boldsymbol{r}_\mathrm{v},
  $$
  ![image-20240619170723516](robot_arm_note.assets/image-20240619170723516.png)

- 电机力矩（考虑输出力矩以及转速和电机转矩）
  $$
  \tau_{\mathrm{M}_i}=\frac{1}{u_{\mathrm{G}_i}} \tau_i+_{(\mathrm{A}_i)}J_{zz_{\mathrm{A}_i}} \ddot{q}_{\mathrm{M}_i}
  
  \\ \\
  
  \tau_{\mathrm{M}_i}=\frac1{u_{\mathrm{G}_i}} \tau_i+u_{\mathrm{G}_i (\mathrm{A}_i)}J_{zz_{\mathrm{A}_i}} \ddot{q}_i.
  $$

  - 电机传动比 $u_{\text{G}_i}$

  - 电机转动惯量 $_{(\mathrm{A}_i)}J_{zz_{\mathrm{A}_i}}$

  - 电机转速 $\ddot{q}_{\text{M}_i}$

  - 电机输出转速 $\ddot{q}_i$

  - 电机力矩 $\tau_{\text{M}_i}$

  - 电机输出力矩 $\tau_i$

    <img src="robot_arm_note.assets/image-20240619171621371.png" alt="image-20240619171621371" style="zoom: 67%;" />

- 电机层的逆动力学模型
  $$
  \begin{aligned}
  \boldsymbol{\tau}_{\mathrm{M}}
  & =
  \mathrm{diag}\left(
  \frac{1}{u_{\mathrm{G}_1}}, 
  \ldots, 
  \frac{1}{u_{\mathrm{G}_n}}
  \right) 
  \left(
  \boldsymbol{M}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+
  \boldsymbol{c}(\boldsymbol{q},\dot{\boldsymbol{q}})+
  \boldsymbol{g}(\boldsymbol{q})\right)
  \\
  &+
  \operatorname{diag}(\operatorname{sign}(\dot{\boldsymbol{q}}))\boldsymbol{r}_\mathrm{c}+\operatorname{diag}(\dot{\boldsymbol{q}})\boldsymbol{r}_\mathrm{v},
  \\ 
  &+
  \mathrm{diag}\left(
  u_{\text{G}_1}
  {_{(\text{A}_1)}}J_{{\text{zz}, \text{A}_1}}, 
  \cdots,
  u_{\text{G}_n}
  {_{(\text{A}_n)}}J_{{\text{zz}, \text{A}_n}}
  \right)
  \ddot{\boldsymbol{q}}
  
  \end{aligned}
  $$





### 机械臂视觉应用
