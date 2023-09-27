# 目标检测像素坐标点
int32[] football_xyxy
int32[] goal_xyxy
int32[] net_xyxy
int32[] robot_xyxy
int32[] penalty_mark_xyxy
int32[] center_circle_xyxy
# PNP计算距离
float64 distance # 原始PNP计算距离
float64 kf_distance # 卡尔曼滤波后的计算距离