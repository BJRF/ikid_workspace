# ikid_workspace

## 2023.03.01 
### v1.0.0增加ros节点
src

## 2023.03.15 
### v1.0.1增加pnp
src/calculate_position_pkg/src/calculate_position.cpp

## 2023.04.01 
### v1.0.2yolo+pnp
src/calculate_position_pkg
src/realtime_detect_pkg

## 2023.04.12 
### v1.0.3增加卡尔曼滤波
src/calculate_position_pkg/src/source/kalman_filter.cpp

## 2023.04.22 
### v1.0.4增加robot_brain节点
src/robot_brain_pkg

## 2023.05.01 
### v1.0.5yolo增加6类并改变ros msg，增加有限状态机框架
#### 有限状态机
src/robot_brain_pkg/src/state_machine.h

## 2023.10.11
### v2.0.0 2023RoboCup赛前版本
1. 增加入场
2. 增加裁判盒
3. 左右平移+旋转
4. 球门球网flag

## 2023.10.19 
### v2.1.0 2023RoboCup赛后版本
1. 增加守门员功能、带球功能脚本
2. 增加技术挑战赛跳球角球等功能。
3. 增加UDP工具包。
4. 增加辅助定位相关helper、Worker。
5. 增加华北五省点球。
6. 增加各项配置文件读写。