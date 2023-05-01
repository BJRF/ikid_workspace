#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Int64.h"
#include "robot_brain_pkg/cmd_walk.h"
#include "robot_brain_pkg/calculate_position_result.h"
#include "robot_brain_pkg/robot_head_pos.h"
#include "state_machine.h"

// 全局变量
// ros 发布
ros::Publisher pub_walk;
ros::Publisher pub_spcial;
// 状态机类
StateMachine state_machine;

// 声明
// int get_decision(float distance);

void CollectEnvData(const robot_brain_pkg::calculate_position_result::ConstPtr& position_res) {
    // state_machine.updateEnvData();

	//将接收的消息打印出来
    // ROS_INFO("收到坐标: [x1:%d, y1:%d, x2:%d, y2:%d, x3:%d, y3:%d, x4:%d, y4:%d, ]\n 收到距离: [distance:%f, kf_distance:%f]", 
    // position_res->x1, position_res->y1, position_res->x2, position_res->y2, position_res->x3, position_res->y3, 
    // position_res->x4, position_res->y4, position_res->distance, position_res->kf_distance);
    /*
    计算方位
    */
    /*
    进行决策
    */
    // int decision = get_decision(position_res -> distance);
    /*
    做出行为
    */
    // 向前行走
    // if(decision == 1) {
    //     robot_brain_pkg::cmd_walk walk;
    //     walk.sx = 0.08;
    //     walk.sy = 0.7 * 2;
    //     walk.stop_walk = false;
    //     walk.walk_with_ball = false;
    //     walk.var_theta = 0;
    //     pub_walk.publish(walk);
    // }else if(decision == 2) {// 踢球特殊步态
    //     std_msgs::Int64 msg;
    //     msg.data = 3;
    //     pub_spcial.publish(msg);
    // }
}

// int get_decision(float distance) {
//     if(distance < 50) {
//         return 2;
//     }else {
//         return 1;
//     }
// }

int main(int argc, char **argv)
{

    //定义image_points对象
    // robot_brain_pkg::calculate_position_result position_res;
 
    //初始化ros节点
    ros::init(argc, argv, "crobot_brain");
 
    //创建节点句柄
    ros::NodeHandle nh;

    pub_walk = nh.advertise<robot_brain_pkg::cmd_walk>("cmd_walk_chatter",10);
    pub_spcial = nh.advertise<std_msgs::Int64>("special_gait",10);
    
    //创建Subscribe，订阅名为chatter的话题，注册回调函数chatterCallBack
    ros::Subscriber sub = nh.subscribe("chatter_calculate_position", 100, CollectEnvData);

    //初始化环境数据
    state_machine.cur_env_data = new EnvData();

    //循环等待消息回调
    while(ros::ok()) {
        /* 
            CollectEnvData 得到环境信息
        */
        ros::spinOnce();
        EnvData *cur_env_data = state_machine.cur_env_data;
        /* 
            getCurrentState 得到当前状态
        */
        State cur_state = state_machine.getCurrentState();
        /* 
            getEventByEnvState 由环境和状态得到事件
        */
        Event event = state_machine.getEventByEnvState(cur_env_data, cur_state);
        /* 
            handleEvent 由事件和当前状态得到下一个状态
        */
        state_machine.handleEvent(event);
        /* 
            execute 执行状态内容
        */
        state_machine.execute();
    }

    // waitKey(0);
    return 0;
}

