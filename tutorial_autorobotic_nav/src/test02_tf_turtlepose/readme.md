# Turtlepose



### TF 关系

-   world 坐标系为基底坐标系
-   turtle1 坐标系在world中为动态变化关系
-   laser1 坐标系在turtle1坐标系中为动态关系（只是动态实现，实际没有变化）
-   laser2 坐标系在turtle1坐标系中为静态关系，通过launchfile实现
-   laser3 坐标在sub中通过tf_listener的transform 监听实现，并与world绑定发布

![](readme.assets/frames.svg)
