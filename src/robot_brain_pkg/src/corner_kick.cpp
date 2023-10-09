#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "robot_brain_pkg/cmd_walk.h"
#include "robot_brain_pkg/calculate_position_result.h"
#include "robot_brain_pkg/robot_head_pos.h"
#include "robot_brain_pkg/state_machine.h"
// #include "robot_brain_pkg/state_machine_NorthChina.h"


// 状态机类
StateMachine state_machine;
// 华北五省点球状态机
// StateMachineNorthChina state_machine;

extern ros::Publisher pub_head_control;
// 声明
// int get_decision(float distance);

// 文件输入输出流
std::fstream corner_fout;
std::fstream corner_fin;

float neck_rotation_theta_angle = 0;
float neck_front_swing_theta_angle = 0;
int count = 0;
bool kick_flag = true;

// 机器人选择模式
enum class Mode
{
    COMPETITION = 1,
    COMPETITION_HELPER,
    COLLABORATIVE_LOCALIZATION_HELPER,
    COLLABORATIVE_LOCALIZATION_WORKER,
    TEST
};

const char corner_head_pos_file_path[] = "/home/nvidia/ikid_ws/src/robot_brain_pkg/data/corner_head_pos_angle.txt";

// 读取头部位置的信息
void readHeadPos() {
    // RobotHeadPosAngle temp_robot_head_pos_angle;
    corner_fin.open(corner_head_pos_file_path, std::ios::in);
    corner_fin >> neck_rotation_theta_angle >> neck_front_swing_theta_angle;
    corner_fin.close();
}

void CollectEnvData(const robot_brain_pkg::calculate_position_result::ConstPtr& position_res) {
    // state_machine.updateEnvData(position_res);
    // State next_state = state_machine.getNextStateByEnvCurState();
    // // 更新状态
    // state_machine.pre_state = state_machine.cur_state;
    // state_machine.cur_state = next_state;
    if(count % 30 == 0) {
        state_machine.controlHead(neck_rotation_theta_angle, neck_front_swing_theta_angle, false, false);
    }
    if(count % 150 == 0) {
        kick_flag = true;
    }
    count++;
    if(position_res -> football_xyxy.size() > 0 && kick_flag == true) {
        kick_flag = false;
        std_msgs::Int16 msg;
        // 右脚踢球
        msg.data = 6;
        std::cout << "pusblish kick:" << msg << std::endl;
        pub_spcial.publish(msg);
        std::cout << "角球！启动！！" << std::endl;
    }
}

int main(int argc, char **argv)
{
    Mode mod;
    // cout << "Select your mode: \n1.COMPETITION\n2.HELPER\n3.WORKER\n4.TEST"
    // cin >> mod;
    // if(mod == Mode::TEST) {
        //定义image_points对象
        // robot_brain_pkg::calculate_position_result position_res;
    
        //初始化ros节点
        ros::init(argc, argv, "crobot_brain");
    
        //创建节点句柄
        ros::NodeHandle nh;

        pub_walk = nh.advertise<robot_brain_pkg::cmd_walk>("/cmd_walk",10);
        pub_spcial = nh.advertise<std_msgs::Int16>("/special_gait",10);
        pub_head_control = nh.advertise<robot_brain_pkg::head_contol_by_brain>("/chatter_head_control",10);
        pub_parallelMove = nh.advertise<std_msgs::Int16>("/parallelMove",5);
        
        //创建Subscribe，订阅名为chatter的话题，注册回调函数chatterCallBack
        //缓冲区队列设置1
        ros::Subscriber sub = nh.subscribe("chatter_calculate_position", 1, CollectEnvData);

        //初始化环境数据
        state_machine.cur_env_data = new EnvData();

        //等待3秒直到head成功启动
        ros::Duration(3).sleep();

        // while循环频率每秒30次
        ros::Rate rate(30);

        // 初始颈部关节读文件置位
        readHeadPos();
        state_machine.controlHead(neck_rotation_theta_angle, neck_front_swing_theta_angle, false, false);
        std::cout << "初始颈部关节读文件置位--" << "neck_rotation_theta_angle:" << neck_rotation_theta_angle << " neck_front_swing_theta_angle:" << neck_front_swing_theta_angle << std::endl;

        //循环等待消息回调
        while(ros::ok()) {
            /* 
                CollectEnvData 更新环境信息
            */
            rate.sleep();
            ros::spinOnce();
            // EnvData *cur_env_data = state_machine.cur_env_data;
            /* 
                getCurrentState 得到当前状态
            */
            // State cur_state = state_machine.getCurrentState();
            /* 
                getNextStateByEnvCurState 由环境和当前状态得到下一个状态，并执行下一个状态
            */

            // State next_state = state_machine.getNextStateByEnvCurState();
            // // 更新状态
            // state_machine.pre_state = state_machine.cur_state;
            // state_machine.cur_state = next_state;
        }
    // }
    return 0;
}

