#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "robot_brain_pkg/cmd_walk.h"
#include "robot_brain_pkg/calculate_position_result.h"
#include "robot_brain_pkg/robot_head_pos.h"
#include "robot_brain_pkg/state_machine.h"

// 状态机类
StateMachine state_machine;
extern ros::Publisher pub_head_control;
// 声明
// int get_decision(float distance);

void CollectEnvData(const robot_brain_pkg::calculate_position_result::ConstPtr& position_res) {
    state_machine.updateEnvData(position_res);
    State next_state = state_machine.getNextStateByEnvCurState();
    // 更新状态
    state_machine.pre_state = state_machine.cur_state;
    state_machine.cur_state = next_state;
}

int main(int argc, char **argv)
{

    //定义image_points对象
    // robot_brain_pkg::calculate_position_result position_res;
 
    //初始化ros节点
    ros::init(argc, argv, "crobot_brain");
 
    //创建节点句柄
    ros::NodeHandle nh;

    pub_walk = nh.advertise<robot_brain_pkg::cmd_walk>("/cmd_walk",10);
    pub_spcial = nh.advertise<std_msgs::Int16>("/special_gait",10);
    pub_head_control = nh.advertise<robot_brain_pkg::head_contol_by_brain>("/chatter_head_control",10);
    
    //创建Subscribe，订阅名为chatter的话题，注册回调函数chatterCallBack
    //缓冲区队列设置1
    ros::Subscriber sub = nh.subscribe("chatter_calculate_position", 1, CollectEnvData);

    //初始化环境数据
    state_machine.cur_env_data = new EnvData();

    //等待3秒直到head成功启动
    ros::Duration(3).sleep();

    // while循环频率每秒30次
    ros::Rate rate(30);

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
    return 0;
}

