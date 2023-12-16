#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "worker_brain_pkg/cmd_walk.h"
#include "worker_brain_pkg/calculate_position_result.h"
#include "worker_brain_pkg/robot_head_pos.h"
#include "worker_brain_pkg/new_worker_state_machine.h"
// #include "worker_brain_pkg/worker.h"
//udp
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <chrono>


// 状态机类
NewWorkerStateMachine state_machine;

extern ros::Publisher pub_head_control;

UdpData data;

void CollectEnvData(UdpData& data) {
    // std::cout << "test2" << std::endl;
    // state_machine.updateEnvData(data);
    // State next_state = state_machine.getNextStateByEnvCurState();
    // // 更新状态
    // state_machine.pre_state = state_machine.cur_state;
    // state_machine.cur_state = next_state;
    state_machine.execute_action_by_cur_data(data);
}

int main(int argc, char **argv)
{
    //初始化ros节点
    ros::init(argc, argv, "worker_robot_brain");

    //创建节点句柄
    ros::NodeHandle nh;

    // 注册
    pub_walk = nh.advertise<worker_brain_pkg::cmd_walk>("/cmd_walk",10);
    pub_spcial = nh.advertise<std_msgs::Int16>("/special_gait",10);
    pub_head_control = nh.advertise<worker_brain_pkg::head_contol_by_brain>("/chatter_head_control",10);
    pub_parallelMove = nh.advertise<std_msgs::Int16>("/parallelMove",5);
    
    // 创建Subscribe，订阅名为chatter的话题，注册回调函数chatterCallBack
    // 缓冲区队列设置1
    // ros::Subscriber sub = nh.subscribe("chatter_calculate_position", 1, CollectEnvData);

    // 初始化环境数据
    state_machine.cur_env_data = new EnvData();

    // 等待3秒直到head成功启动
    // ros::Duration(3).sleep();

    // 辅助定位worker
    // UDP接收信息
    int serverPort = 12345;
    // 在服务器端启动接收器
    UDPReceiver receiver(serverPort);

    // while循环频率每秒30次
    // ros::Rate rate(30);

    while (ros::ok()) {
        // ros::Duration(0.03).sleep();
        // rate.sleep();
        // std::cout << "test" << std::endl;
        // 接收来自helper的数据
        receiver.ReceiveData(data);
        state_machine.execute_action_by_cur_data(data);
        // std::cout << "Received data: distance=" << data.distance << std::endl;
        // state_machine.execute_action_by_cur_data(data);
        // CollectEnvData(data);
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        // std::cout << "------1" << std::endl;
    }

    return 0;
}



