#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include <math.h>
#include "std_msgs/Int64.h"
#include "robot_brain_pkg/cmd_walk.h"
#include "robot_brain_pkg/calculate_position_result.h"
#include "robot_brain_pkg/robot_head_pos.h"
#include "robot_brain_pkg/head_contol_by_brain.h"

#define PI acos(-1)

ros::Publisher pub_head_pos;

int head_state = 0;//0初始化， 1找球， 2跟球
double neck_rotation_theta= 0; //颈旋转关节角度
double neck_front_swing_theta = 0; // 颈前摆关节角度

std::vector<std::vector<float>> angle_vec = {{10, 0},{-60, 0}, {-60, 35}, {10, 35}, {60, 35}, {60, 70}, {10, 70}, {-60, 70}, {10, 35}};
int angle_vec_pos = 0;

// 文件输入输出流
std::fstream fout;

const char head_pos_file_path[] = "/home/hjf/project/ikid_workspace/src/robot_brain_pkg/data/head_pos_angle.txt";

// 写头部位置
void writeHeadPos(double neck_rotation_theta_angle, double neck_front_swing_theta_angle) {
    fout.open(head_pos_file_path, std::ios::out);
    if(fout.fail()){
        fout.open(head_pos_file_path, std::ios::app);
        fout.close();
        fout.open(head_pos_file_path, std::ios::out);
    }
    fout << neck_rotation_theta_angle << ' ' << neck_front_swing_theta_angle;
    fout.close();
}

// 角度转弧度
double angleToRadian(double angle) {
    double radian = angle / 180 * PI;
    return radian;
}

// 操作头部舵机(每次操作也要记录head_pos)
void operateHead(double neck_rotation_theta_angle, double neck_front_swing_theta_angle) {
    robot_brain_pkg::robot_head_pos head_pos;
    head_pos.neck_rotation_theta = angleToRadian(neck_rotation_theta_angle);
    head_pos.neck_front_swing_theta = angleToRadian(neck_front_swing_theta_angle);
    pub_head_pos.publish(head_pos);
    //写入
    writeHeadPos(neck_rotation_theta_angle, neck_front_swing_theta_angle);
}

void headControl(const robot_brain_pkg::head_contol_by_brain::ConstPtr& head_contol_by_brain) {
    // 先判断状态
    // 如果要求是转动舵机
    if(head_contol_by_brain ->is_find_state == false &&head_contol_by_brain ->is_follow_state == false) {
        operateHead(head_contol_by_brain -> neck_rotation_theta, head_contol_by_brain -> neck_front_swing_theta);
        return;
    }
    // 如果是找球
    else if(head_contol_by_brain -> is_find_state == true) {
        //第一次进入找球，angle_vec_pos从0开始
        if(head_state == 0) {
            //设置头部状态为找球
            head_state = 1;
            operateHead(angle_vec[angle_vec_pos][0], angle_vec[angle_vec_pos][1]);
            angle_vec_pos++;
            // 数组越界置零
            if(angle_vec_pos == angle_vec.size()){
                angle_vec_pos = 0;
            }
        }
        //判断是否状态已经是找球了
        else if(head_state == 1) {
            operateHead(angle_vec[angle_vec_pos][0], angle_vec[angle_vec_pos][1]);
            angle_vec_pos++;
            // 数组越界置零
            if(angle_vec_pos == angle_vec.size()){
                angle_vec_pos = 0;
            }
        }  
    }
    // 如果是跟随
    else if(head_contol_by_brain -> is_follow_state == true) {
        angle_vec_pos = 0;
        head_state = 2;
    }
}

int main(int argc, char **argv)
{
 
    //初始化ros节点
    ros::init(argc, argv, "head_control");
 
    //创建节点句柄
    ros::NodeHandle nh;

    pub_head_pos = nh.advertise<robot_brain_pkg::robot_head_pos>("/ikid_robot/robot_head_pos_msg",10);
    
    //创建Subscribe，订阅名为chatter的话题，注册回调函数chatterCallBack
    ros::Subscriber sub = nh.subscribe("/chatter_head_control", 10, headControl);

    // 半秒一回调
    ros::Rate rate(2);
    //循环等待消息回调
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
        
    return 0;
}

