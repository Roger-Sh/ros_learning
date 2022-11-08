# TUNING PERCEPTION AND MOTION PLANNING PARAMETERS FOR MOVEIT! FRAMEWORK

## ABSTRACT
- param of perception and planning
- initial benchmark
  - virtual simulation UR3
  - single obstacle
  - performance, means of successful 
    - runs, 
    - path planning, 
    - execution duration
- based on initial benckmark results
  - param optimized by Particle Swarm Optimization
  - test optimized param in first benckmark
  - test optimized param in four more benckmarks

## INTRODUCTION
- moveit, octomap
- choose planner
- choose three paramter for planner?
- Particle-Swarm Optimization

## BACKGROUND
- sample-based planners
  - probabilistic completeness
  - connect sampled configuration states to 
    find a feasible motion plan
  - multi-query and single-query
    - multi-query
      - preprocessed roadmap
      - multiple path search
      - static environment
    - single-query
      - each time a new tree-like graph
      - faster
      - path not smooth
- Other moveit paramter optimization papers, no perception params
- Experiment Environment
  - Gazebo
  - UR3
  - D435
- Particle Swarm Optimization 粒子群算法
  [粒子群优化算法](https://zhuanlan.zhihu.com/p/346355572)


## BENCHMARK OF PERCEPTION AND PLANNING PARAMETERS 
- introduce gazebo env, goal pos A B
- tested paramters
  - octomap resolution, octomap 地图精度
  - point subsample, 点云降采样
  - octomap max update rate
  - workspace limitation 限制机械臂的工作空间
  - collission mesh, 尽量使用简化的collision mesh
  - goal joint tolerance, 目标位置的joint宽容度
  - kinematics solver KDL
  - longest valid segment fraction 用来设置在一段距离内不执行碰撞检测
  - path simplification, 对 motion plan 进行简化的后处理
- tested metrics for 3 movement
  - duration of motion planning
  - duration of execution of planned movement
  - cycle success ratio
- Relative Performance
  - collision mesh 效果提升最明显
  - 有微弱提升的
    - points subsample 
    - goal joint tolerance 增大误差范围
    - octomap resolution 
  - Planner 有提升的
    - BiEST
    - BiTRRT, 
      - 规划速度和执行速度更快, 但更靠近障碍物, 导致整体成功率低 如果要使用这个, 就要增大物体的padding
    - PDST
    - RRTConnect
    - STRIDE

## OPTIMISATION OF PARAMETER SETTINGS 
- octomap resolution, min 0.01, max 0.1, best1 0.0678, best2 0.0285
- points subsample, min 1, max200, best1 125, best2 103
- longest valid segment fraction, min 0.001, max 0.01, best1 0.0056, best2 0.0071

## CONCLUSION
- 使用简化的碰撞模型能有效提升规划速度
- BiTRRT 规划速度最快, 但是离障碍物最近, 需要配合padding使用