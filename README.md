# laser_follower_slam_V2.0
【ROS机器人】【激光雷达跟随】【SLAM建图】
【V2.0版本可以区分物体，跟随指定宽度的物体】
- 对雷达数据做差分，差分大于差分阈值的则为物体边界
- 根据物体边界，利用极坐标系下的余弦定理计算物体的宽度
- 找出符合跟随目标宽度阈值的物体
- 在符合宽度的物体中寻找最近的跟随
- 如果跟随物体离机器人的距离过远，则停止跟随
