#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include <iostream>
#include <random>
#include "std_msgs/Int16.h"
#include "robot_brain_pkg/cmd_walk.h"
#include "robot_brain_pkg/calculate_position_result.h"
#include "robot_brain_pkg/robot_head_pos.h"
#include "robot_brain_pkg/state_machine_NorthChina_keeper.h"

StateMachineNorthChina state_machine;

extern ros::Publisher pub_head_control;

std::fstream goalkeeper_fout;
std::fstream goalkeeper_fin;

float neck_rotation_theta_angle = 0;
float neck_front_swing_theta_angle = 35;
int count = 0;
bool keeper_flag = false;

const char goalkeeper_head_pos_file_path[] = "/home/ros/ikid_ws/src/robot_brain_pkg/data/keeper_head_pos_angle.txt";

void readHeadPos()
{
    goalkeeper_fin.open(goalkeeper_head_pos_file_path, std::ios::in);
    goalkeeper_fin >> neck_rotation_theta_angle >> neck_front_swing_theta_angle;
    std::cout << "goalkeeper_fin311:" << std::endl;

    goalkeeper_fin.close();
}

std::deque<cv::Point> ball_trajectory; // ���ڱ�������켣�Ķ���

bool IsBallMovingLeft()
{
    if (ball_trajectory.size() < 2)
    {
        return false;
    }

    cv::Point currentPos = ball_trajectory.back();
    cv::Point prevPos = ball_trajectory.at(ball_trajectory.size() - 2);

    if ((currentPos.y - prevPos.y > 5) && (currentPos.x - prevPos.x < 5))
    {
        return true;
    }

    return false;
}

bool IsBallMovingRight()
{
    if (ball_trajectory.size() < 2)
    {
        return false;
    }

    cv::Point currentPos = ball_trajectory.back();
    cv::Point prevPos = ball_trajectory.at(ball_trajectory.size() - 2);

    if ((currentPos.y - prevPos.y > 5) && (currentPos.x - prevPos.x > 5))
    {
        return true;
    }

    return false;
}

void CollectEnvData(const robot_brain_pkg::calculate_position_result::ConstPtr &position_res)
{
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_int_distribution<int> distribution(7, 8);

    // int randomValue = distribution(gen);

    // if no football
    if (position_res->football_xyxy.size() == 0)
    {
        if (keeper_flag == true)
        {
            std::cout << "keeper sleep" << std::endl;
            ros::Duration(9).sleep();
            keeper_flag = false;
            std::cout << "keeper ok" << std::endl;
            return;
        }
        std::cout << "no football" << std::endl;
        keeper_flag = false;
        return;
    }else
    {
        if (keeper_flag == true)
        {
            std::cout << "keeper sleep" << std::endl;
            ros::Duration(10).sleep();
            keeper_flag = false;
            std::cout << "keeper ok" << std::endl;
            return;
        }
    }

    if (count % 30 == 0)
    {
        state_machine.controlHead(neck_rotation_theta_angle, neck_front_swing_theta_angle, false, false);
    }

    int central_point_x = position_res->football_xyxy[0] + (position_res->football_xyxy[2] - position_res->football_xyxy[0]) / 2; // 640
    int central_point_y = position_res->football_xyxy[1] + (position_res->football_xyxy[3] - position_res->football_xyxy[1]) / 2; // 480

    // define solve football trade
    ball_trajectory.push_back(cv::Point(central_point_x, central_point_y));

    if (ball_trajectory.size() > 10)
    {
        ball_trajectory.pop_front();
    }

    // solve football trade
    if (central_point_x <= FOLLOW_FPOINT_EDGE_LEFT && central_point_y >= FOLLOW_FPOINT_EDGE_DOWN)
    {
        std::cout << "left area" << std::endl;
        // randomValue
        if (IsBallMovingLeft())
        {
            std::cout << "left moving" << std::endl;
            std_msgs::Int16 msg;
            msg.data = 7;
            // std::cout << "left goalkeeper" << msg << std::endl;
            pub_spcial.publish(msg);
            std::cout << "left success" << std::endl;
            ros::Duration(6).sleep();

            keeper_flag = true;
            return;
        }
        return;
    }
    // �����ұߣ�������
    else if (central_point_x >= FOLLOW_FPOINT_EDGE_RIGHT && central_point_y >= FOLLOW_FPOINT_EDGE_DOWN)
    {
        std::cout << "right area" << std::endl;
        if (IsBallMovingRight())
        {
            std::cout << "right moving" << std::endl;
            std_msgs::Int16 msg;
            msg.data = 8;
            // std::cout << "right goalkeeper:" << msg << std::endl;
            pub_spcial.publish(msg);
            std::cout << "right success" << std::endl;
            ros::Duration(6).sleep();

            keeper_flag = true;

            return;
        }
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crobot_brain");
    ros::NodeHandle nh;

    pub_walk = nh.advertise<robot_brain_pkg::cmd_walk>("/cmd_walk", 10);
    pub_spcial = nh.advertise<std_msgs::Int16>("/special_gait", 10);
    pub_head_control = nh.advertise<robot_brain_pkg::head_contol_by_brain>("/chatter_head_control", 10);
    // pub_parallelMove = nh.advertise<std_msgs::Int16>("/parallelMove",1);

    ros::Subscriber sub = nh.subscribe("chatter_calculate_position", 1, CollectEnvData);

    state_machine.cur_env_data = new EnvData();

    ros::Duration(3).sleep();

    ros::Rate rate(60);

    readHeadPos();
    state_machine.controlHead(neck_rotation_theta_angle, neck_front_swing_theta_angle, false, false);
    std::cout << "neck_rotation_theta_angle:" << neck_rotation_theta_angle << " neck_front_swing_theta_angle:" << neck_front_swing_theta_angle << std::endl;

    // ѭ���ȴ���Ϣ�ص�
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
