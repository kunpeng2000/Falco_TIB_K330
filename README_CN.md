# Falco Navigation & Relocalization
本项目实现了一套基于 Falco 的自动导航系统，并结合 Scan Context 算法，在已有先验地图的基础上实现了全局重定位功能。该系统允许机器人在任意未知起点启动，并能够准确获取其出发时在全局地图中的初始位置。

坐标系说明：
1. `map`全局坐标系，全局点云在该坐标系下
2. `camera_init`里程计起点坐标系（即机器人出发时的里程计原点）
3. `base_link`高速里程计坐标系
4. `aft_mapped`FAST-LIVO2低速里程计坐标系

## 流程
### 全局地图及离线帧保存

该步骤用于建立环境的先验地图，并记录重定位所需的特征数据。

`roslaunch init_relocalizer save_data.launch`

启动 FAST-LIVO2 进行全局建图。在建图过程中，系统会按照设定的距离阈值自动保存局部点云帧及其对应的里程计位姿，并在建图结束时保存完整的全局点云地图。

参数：

1. `cloud_topic`接收局部点云话题
2. `odom_topic`接收里程计话题
3. `save_dir`局部帧和里程计信息保存位置
4. `save_distance_thresh`关键帧保存的距离阈值（即每移动$n$米保存一帧数据）
   
### 重定位

在已有先验地图的环境中，当机器人重新启动时，通过此节点找回自身在全局地图中的位置。

`roslaunch init_relocalizer re_loc.launch`

启动 FAST-LIVO2，系统会提取当前扫描帧并与先验数据进行 Scan Context 匹配，计算出此时的起点位置，发布`map`和`camera_init`之间tf。

当终端输出**Published AVERAGED pose**时，表示多帧平均重定位已完成，系统位姿收敛，此时即可开启下游导航模块。

参数：

1. `cloud_topic`接收局部点云话题
2. `data_dir`局部帧和里程计信息位置
3. `global_map_file`全局点云地图文件路径（.pcd 格式）
4. `map_frame`全局地图坐标系
5. `base_frame`机器人出发时里程计原点
6. `avg_count`重定位位姿平滑次数（取$n$次重定位结果的平均值以提高精度）
7. `global_map_pub_freq`全局点云地图发布频率（用于RVIZ可视化）

### 导航

完成重定位后，启动导航模块以执行移动任务。

`roslaunch terrain_analysis terrain_analysis_planner.launch`

启动Falco导航与地形分析模块，给定目标点开始发布`cmd_vel`话题