#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Int64.h"
#include "robot_brain_pkg/cmd_walk.h"
#include "robot_brain_pkg/calculate_position_result.h"

// using namespace std;
// using namespace cv;

ros::Publisher pub_walk;
ros::Publisher pub_spcial;

int get_decision(float distance);

void CalculatePnp(const robot_brain_pkg::calculate_position_result::ConstPtr& position_res) {

	//将接收的消息打印出来
    ROS_INFO("收到坐标: [x1:%d, y1:%d, x2:%d, y2:%d, x3:%d, y3:%d, x4:%d, y4:%d, ]\n 收到距离: [distance:%f, kf_distance:%f]", 
    position_res->x1, position_res->y1, position_res->x2, position_res->y2, position_res->x3, position_res->y3, 
    position_res->x4, position_res->y4, position_res->distance, position_res->kf_distance);

    /*
    计算方位
    */

    /*
    进行决策
    */
    int decision = get_decision(position_res -> distance);

    /*
    做出行为
    */

    // 向前行走
    if(decision == 1) {
        robot_brain_pkg::cmd_walk walk;
        walk.sx = 0.08;
        walk.sy = 0.7 * 2;
        walk.stop_walk = false;
        walk.walk_with_ball = false;
        walk.var_theta = 0;
        pub_walk.publish(walk);
    }else if(decision == 2) {// 踢球特殊步态
        std_msgs::Int64 msg;
        msg.data = 3;
        pub_spcial.publish(msg);
    }

}

int get_decision(float distance) {
    if(distance < 50) {
        return 2;
    }else {
        return 1;
    }
}

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
    ros::Subscriber sub = nh.subscribe("chatter_calculate_position", 100, CalculatePnp);
 
    //循环等待消息回调
    ros::spin();

    return 0;
	// waitKey(0);
}

