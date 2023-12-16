#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include <opencv2/opencv.hpp>
#include "worker_brain_pkg/calculate_position_result.h"
#include "worker_brain_pkg/cmd_walk.h"
#include "worker_brain_pkg/robot_head_pos.h"
#include "worker_brain_pkg/head_contol_by_brain.h"
#include "worker_brain_pkg/helper.h"

#define PI acos(-1)
#define FIND_FOOTBALL_STATE_FRAME_INTERVAL 13
#define FOUND_FOOTBALL_TRIGGER 6
#define LOST_FOOTBALL_TRIGGER 30
#define KICK_FOOTBALL_DISTANCE_TRIGGER 30
#define FOLLOW_FPOINT_EDGE_LEFT 300
#define FOLLOW_FPOINT_EDGE_RIGHT 320
#define FOLLOW_FPOINT_EDGE_UP 220
#define FOLLOW_FPOINT_EDGE_DOWN 260
#define FOLLOW_FOOTBALL_DEVIATE_TRIGGER_ANGLE 15
#define ROTATION_OFFSET_ANGLE 3
#define FRONT_OFFSET_ANGLE 3
#define MAX_WALK_ROTATION_ANGLE 15
#define RIGHT_KICKBALL 3
#define LEFT_KICKBALL 4
#define NO_ADJUST_BODY_DISTANCE 33

using namespace cv;
using namespace std;

// 全局变量
// ros 发布
ros::Publisher pub_walk;
ros::Publisher pub_spcial;
ros::Publisher pub_head_control;
ros::Publisher pub_parallelMove;

// 文件输入输出流
std::fstream fout;
std::fstream fin;

const char head_pos_file_path[] = "/home/nvidia/ikid_ws/src/worker_brain_pkg/data/head_pos_angle.txt";

//helper UDP相关
const char* serverIP = "192.168.1.247";
int serverPort = 12345;
// 在helper启动UDP发送器
UDPSender sender(serverIP, serverPort);

UdpData data;

struct ParametersSrvData {
    bool stop_special_gait_flag;
    bool stop_walk_flag;
    bool stop_turn_flag;
    bool walk_with_ball;
};

// 角度单位的head_pos
struct RobotHeadPosAngle {
    double neck_rotation_theta_angle;
    double neck_front_swing_theta_angle;
};

//定义当前环境数据
struct EnvData{
    // 目标检测像素坐标点
    std::vector<int> football_xyxy;
    std::vector<int> goal_xyxy;
    std::vector<int> net_xyxy;
    std::vector<int> robot_xyxy;
    std::vector<int> penalty_mark_xyxy;
    std::vector<int> center_circle_xyxy;
    // 球距离
    float distance; // 球计算距离
    float kf_distance; // after kf 球距离
    /*
        float64 sx  # 步长
        float64 sy   # 步宽
        float64 var_theta  # 转角增量
        bool walk_with_ball  # 是否带球行走/动态踢球
        bool stop_walk   # 停止行走标志位
    */
    worker_brain_pkg::cmd_walk cur_cmd_walk;
    /* 
        # 发送前转为弧度
        # 颈前摆：0-90度 上-下
        # 颈旋转：80-（-80）度 左-右
        float64 neck_rotation_theta  # 颈旋转关节角度
        float64 neck_front_swing_theta # 颈前摆关节角度
    */
    // 这里存的是弧度
    // worker_brain_pkg::robot_head_pos cur_robot_head_pos;
    // 这里存的是角度
    RobotHeadPosAngle robot_head_pos_angle;

    // 参数服务器数据
    ParametersSrvData parameters_srv_data;
    EnvData() {
        
    }
    EnvData(const worker_brain_pkg::calculate_position_result::ConstPtr& position_res, 
    const worker_brain_pkg::cmd_walk& cmd_walk, const RobotHeadPosAngle robot_head_pos_angle) {
        football_xyxy = position_res -> football_xyxy;
        goal_xyxy = position_res -> goal_xyxy;
        net_xyxy = position_res -> net_xyxy;
        net_xyxy = position_res -> net_xyxy;
        penalty_mark_xyxy = position_res -> penalty_mark_xyxy;
        center_circle_xyxy = position_res -> center_circle_xyxy;
        distance = position_res -> distance;
        kf_distance = position_res -> kf_distance;
        this -> cur_cmd_walk = cmd_walk;
        this -> robot_head_pos_angle = robot_head_pos_angle;
    }
};

// 定义状态枚举
enum class State {
    Initial,        // 开始状态
    FindFootball,   // 找球状态
    FollowFootball,   // 找球状态
    FindRobot,
    FollowRobot,
    Move,           // 移动状态
    Kick         
};

// 定义事件枚举
enum class Event {
    Start,          // 开始事件
    Walk,           // 行走事件
    Turn,           // 转向事件
    FindFootball,   // 找到球事件
    FindGoal,       // 找到门事件
    Kick,           // 踢球事件
    Resume,         // 恢复事件
    Stop            // 停止事件
};

// 状态机类
class NewHelperStateMachine {
public:
    EnvData *cur_env_data; // 当前环境数据
    State cur_state;// 当前状态
    State pre_state;// 上一个状态
    State pre_interrupt_state;// 上次中断时保存的状态
    long long frame;// 帧数

    // 网的连续帧数
    int net_sustain_frames_nums = 0;
    int net_lost_frames_nums = 0;

    // 门的连续帧数
    int goal_sustain_frames_nums = 0;
    int goal_lost_frames_nums = 0;

    // 球的连续帧数
    int football_sustain_frames_nums = 0;
    int football_lost_frames_nums = 0;

    // Robot的连续帧数
    int robot_sustain_frames_nums = 0;
    int robot_lost_frames_nums = 0;

    // 找球的连续帧数
    int find_football_state_sustain_frames_nums = 0;
    // 下一个找球状态的帧
    int next_find_football_state_frame = 0;

    // 找Robot的连续帧数
    int find_robot_state_sustain_frames_nums = 0;
    // 下一个找Robot状态的帧
    int next_find_robot_state_frame = 0;

    // 找wang的连续帧数
    int find_net_state_sustain_frames_nums = 0;

    // 下一个可以开始动的帧
    long long until_pause_frame = 0;

    // 跟球状态中，进入正在确认是否踢球的标志位
    bool confirming_kick = false;

    // 头目前正在执行的任务的标志位
    // 如果使用标志位判断头部舵机的工作则加，否则不使用
    const int FIND_FOOTBALL_TASK = 1;
    const int FIND_GOAL_TASK = 2;
    const int FIND_NET_TASK = 3;
    int head_cur_task = FIND_FOOTBALL_TASK;

    // 当前网是否已经正确
    bool is_net_vaild = false;

    //找网时的一些临时变量 
    float temp_neck_rotation_theta_angle = 0;
    float temp_neck_front_swing_theta_angle = 0;
    // 如果球网或者球门方向正确且处于中心附近
    bool net_flag = false;// 网是否对对准的标志位
    bool goal_flag = false;
    // 找网连续了几帧
    int find_net_frame = 0;
    // 下位机给的参数服务器的默认歩长歩宽，读取一次就行
    double default_walk_length = 0.08;
    double default_walk_width = 0.135;

    // 辅助定位已拐多少
    float cur_walk_rotation_angle = 0;

    // 当前摄像头跟随的目标
    int cur_object = 0;// 0:球; 1:机器人

    // football摄像头x_angle
    int football_angle = 180;
    // robot摄像头x_angle
    int robot_angle = 180;

    vector<float> football_3d_pos = {0,0,0};
    vector<float> robot_3d_pos = {0,0,0};

    // 角度
    const std::vector<std::vector<float>> angle_vec = 
    {{0, 20}, 
    {30, 20}, {60, 20}, 
    {60, 40}, {30, 40}, {0, 40}, {-30, 40}, {-60, 40},
    {-60, 60}, {-30, 60}, {0, 60}, {30, 60}, {60, 60},
    {30, 30}};
    int angle_vec_pos = 0;

    //状态机构造函数
    NewHelperStateMachine() {
        cur_env_data = new EnvData();
        cur_state = State::Initial;
        pre_state = State::Initial;
        frame = 0;
        football_sustain_frames_nums = 0;
        football_lost_frames_nums = 0;
        find_football_state_sustain_frames_nums = 0;
        next_find_football_state_frame = 0;
        confirming_kick = false;
        // 下位机给的参数服务器的默认歩长歩宽，读取一次就行（构造函数问题，下面有重复调用）
        ros::param::get("/pid_amend/walk_length", this -> default_walk_length);
        ros::param::get("/pid_amend/walk_width", this -> default_walk_width);
    }

    // 更新环境信息(walk没有更新)
    void updateEnvData(const worker_brain_pkg::calculate_position_result::ConstPtr& position_res) {
        frame++;
        // 更新球的连续帧数
        if(cur_env_data->football_xyxy.size() > 0){
            football_sustain_frames_nums++;
        }else {
            football_sustain_frames_nums = 0;
        }
        // 更新球的丢失帧数
        if(cur_env_data->football_xyxy.size() == 0){
            football_lost_frames_nums++;
        }else {
            football_lost_frames_nums = 0;
        }
        // 更新robot的连续帧数
        if(cur_env_data->robot_xyxy.size() > 0){
            robot_sustain_frames_nums++;
        }else {
            robot_sustain_frames_nums = 0;
        }
        // 更新robot的丢失帧数
        if(cur_env_data->robot_xyxy.size() == 0){
            robot_lost_frames_nums++;
        }else {
            robot_lost_frames_nums = 0;
        }
        // 更新网的连续帧数
        if(cur_env_data->net_xyxy.size() > 0){
            net_sustain_frames_nums++;
        }else {
            net_sustain_frames_nums = 0;
        }
        // 更新网的丢失帧数
        if(cur_env_data->net_xyxy.size() == 0){
            net_lost_frames_nums++;
        }else {
            net_lost_frames_nums = 0;
        }
        // 更新门的连续帧数
        if(cur_env_data->goal_xyxy.size() > 0){
            goal_sustain_frames_nums++;
        }else {
            goal_sustain_frames_nums = 0;
        }
        // 更新门的丢失帧数
        if(cur_env_data->goal_xyxy.size() == 0){
            goal_lost_frames_nums++;
        }else {
            goal_lost_frames_nums = 0;
        }

        // 读取参数服务器数据
        cur_env_data->parameters_srv_data = readParametersSrvData();
        // 目标检测+pnp的环境数据
        cur_env_data -> football_xyxy = position_res -> football_xyxy;
        cur_env_data -> goal_xyxy = position_res -> goal_xyxy;
        cur_env_data -> net_xyxy = position_res -> net_xyxy;
        cur_env_data -> robot_xyxy = position_res -> robot_xyxy;
        cur_env_data -> penalty_mark_xyxy = position_res -> penalty_mark_xyxy;
        cur_env_data -> center_circle_xyxy = position_res -> center_circle_xyxy;
        cur_env_data -> distance = position_res -> distance;
        cur_env_data -> kf_distance = position_res -> kf_distance;
        // 更新头部舵机信息
        cur_env_data->robot_head_pos_angle = readHeadPos();

        // about helper更新udp信息
        createUdpDate(data, *position_res);
        data.neck_rotation_theta_angle = cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle;
    }

    // 获取当前状态
    State getCurrentState() const {
        return cur_state;
    }

    // 读取头部位置的信息
    RobotHeadPosAngle readHeadPos() {
        RobotHeadPosAngle temp_robot_head_pos_angle;
        fin.open(head_pos_file_path, std::ios::in);
        fin >> temp_robot_head_pos_angle.neck_rotation_theta_angle >> temp_robot_head_pos_angle.neck_front_swing_theta_angle;
        fin.close();
        return temp_robot_head_pos_angle;
    }

    // 读取参数服务器信息
    ParametersSrvData readParametersSrvData() {
        ParametersSrvData temp_data;
        ros::param::get("/pid_amend/walk_length", this -> default_walk_length);
        ros::param::get("/pid_amend/walk_width", this -> default_walk_width);
        ros::param::get("stop_special_gait_flag",temp_data.stop_special_gait_flag);
        ros::param::get("stop_walk_flag",temp_data.stop_walk_flag);
        ros::param::get("stop_turn_flag",temp_data.stop_turn_flag);
        ros::param::get("walk_with_ball",temp_data.walk_with_ball);
        return temp_data;
    }

    // 角度转弧度
    double angleToRadian(double angle) {
        double radian = angle / 180 * PI;
        return radian;
    }

    /*
        发布控制头部消息
        neck_rotation_theta 左右量
        neck_front_swing_theta 上下量
        is_find_state 是否找球状态
        is_follow_state 是否跟球状态
    */
    void controlHead(double neck_rotation_theta_angle, double neck_front_swing_theta_angle, bool is_find_state, bool is_follow_state) {
        worker_brain_pkg::head_contol_by_brain head_contol_by_brain;
        head_contol_by_brain.neck_rotation_theta_angle = neck_rotation_theta_angle;
        head_contol_by_brain.neck_front_swing_theta_angle = neck_front_swing_theta_angle;
        head_contol_by_brain.is_find_state = is_find_state;
        head_contol_by_brain.is_follow_state = is_follow_state;
        pub_head_control.publish(head_contol_by_brain);
    }

    // 调用行走步态
    void runWalk(double sx, double sy, bool stop_walk, bool walk_with_ball, double var_theta) {
        worker_brain_pkg::cmd_walk walk;
        walk.sx = sx;
        walk.sy = sy;
        walk.stop_walk = stop_walk;
        walk.walk_with_ball = walk_with_ball;
        walk.var_theta = var_theta;
        pub_walk.publish(walk);
    }

    //调用踢球步态
    void kickFootball() {
        std_msgs::Int16 msg;
        // 右脚踢球
        if(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle < 0) {
            msg.data = RIGHT_KICKBALL;
        }else {// 左脚踢球
            msg.data = LEFT_KICKBALL;
        }
        std::cout << "pusblish kick:" << msg << std::endl;
        pub_spcial.publish(msg);
    }

    //调用踢球步态
    void kickFootballByLeg(int leg) {
        std_msgs::Int16 msg;
        msg.data = leg;
        std::cout << "pusblish kick:" << msg << std::endl;
        pub_spcial.publish(msg);
    }

    // 设置N帧暂停
    void pauseNFrame(long long N) {
        until_pause_frame = frame + N;
    }

    // 根据目标检测的位置调用摄像头跟目标
    void adjHeadFollowObject(std::vector<int> object_xyxy){
        //可以控制帧数
        if(object_xyxy.size() == 0) {
            return;
        }
        int central_point_x = object_xyxy[0] + (object_xyxy[2] - object_xyxy[0]) / 2; // 640
        int central_point_y = object_xyxy[1] + (object_xyxy[3] - object_xyxy[1]) / 2; // 480
        double rotation_offset_angle = 0;
        double front_offset_angle = 0;
        if(central_point_x < FOLLOW_FPOINT_EDGE_LEFT) {
            rotation_offset_angle = ROTATION_OFFSET_ANGLE;
        }else if(central_point_x >= FOLLOW_FPOINT_EDGE_RIGHT){
            rotation_offset_angle = -ROTATION_OFFSET_ANGLE;
        }
        if(central_point_y > FOLLOW_FPOINT_EDGE_DOWN) {
            front_offset_angle = FRONT_OFFSET_ANGLE;
        }else if(central_point_y <= FOLLOW_FPOINT_EDGE_UP) {
            front_offset_angle = -FRONT_OFFSET_ANGLE;
        }
        controlHead(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle + rotation_offset_angle, 
        cur_env_data->robot_head_pos_angle.neck_front_swing_theta_angle + front_offset_angle, false, false);
    }


    // 求目标距离的函数
    float cal_distance(std::vector<Point2d> image_points, std::vector<Point3d> model_points) {
        // 相机参数
        Mat camera_matrix = (Mat_<float>(3, 3) << 689.318, 0, 299.737,
        0, 690.730 , 215.668,
        0, 0, 1);
        // 相机畸变系数
        Mat dist_coeffs = (Mat_<float>(5, 1) << 0, 0, 0, 0, 0);
        /*
            PNP算法求解
        */
        // 旋转向量
        Mat rotation_vector;
        // 平移向量
        Mat translation_vector;
        solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector, 0, SOLVEPNP_EPNP);
        Mat Rvec;
        Mat_<float> Tvec;
        rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
        translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式 
        Mat_<float> rotMat(3, 3);
        Rodrigues(Rvec, rotMat);
        // 旋转向量转成旋转矩阵
        Mat P_oc;
        P_oc = -rotMat.inv() * Tvec;
        float distance = P_oc.at<float>(2,0);
        return distance;
    }

    std::vector<float> cal_object_pos(int object_calss) {//class robot = 1, football = 0
        // 构造image_point2d
        vector<Point2d> image_points;
        // 构造model_point3d
        std::vector<Point3d> model_points;
        if(object_calss == 0) {// football
            // 计算球点
            int football_point1_x = cur_env_data->football_xyxy[0];
            int football_point1_y = cur_env_data->football_xyxy[1];
            int football_point2_x = cur_env_data->football_xyxy[2];
            int football_point2_y = cur_env_data->football_xyxy[3];
            int football_point3_x = football_point2_x;
            int football_point3_y = football_point1_y;
            int football_point4_x = football_point1_x + (football_point2_x - football_point1_x) / 2;
            int football_point4_y = football_point1_y + (football_point2_y - football_point1_y) / 2;
            int football_point5_x = cur_env_data->football_xyxy[0] + (cur_env_data->football_xyxy[2]-cur_env_data->football_xyxy[0])/4;
            int football_point5_y = football_point4_y;
            int football_point6_x = football_point4_x;
            int football_point6_y = football_point1_y + (football_point2_y - football_point1_y) *7/8;
            int football_point7_x = football_point5_x;
            int football_point7_y = football_point1_y - (football_point2_y - football_point1_y) / 4;
            int football_point8_x = football_point4_x;
            int football_point8_y = football_point7_y;
            int football_point9_x = football_point2_x / 2;
            int football_point9_y = football_point1_y / 2;
            int football_point10_x= football_point2_x / 2;
            int football_point10_y= football_point4_y;
            int football_point11_x= football_point2_x / 2;
            int football_point11_y= football_point2_y - (football_point2_y - football_point1_y) / 4;
            int football_point12_x= football_point4_x;
            int football_point12_y= football_point2_y / 2;
            int football_point13_x= football_point2_x / 2;
            int football_point13_y= football_point11_y;
            int football_point14_x = football_point1_x;
            int football_point14_y = football_point2_y;

            // 构造image_point2d
            // vector<Point2d> image_points;
            image_points.push_back(Point2d(football_point1_x, football_point1_y));
            image_points.push_back(Point2d(football_point2_x, football_point2_y));
            image_points.push_back(Point2d(football_point3_x, football_point3_y));
            image_points.push_back(Point2d(football_point4_x, football_point4_y));
            image_points.push_back(Point2d(football_point5_x, football_point5_y));
            image_points.push_back(Point2d(football_point6_x, football_point6_y));
            // image_points.push_back(Point2d(football_point7_x, football_point7_y));
            // image_points.push_back(Point2d(football_point8_x, football_point8_y));
            // image_points.push_back(Point2d(football_point9_x, football_point9_y));
            // image_points.push_back(Point2d(football_point10_x, football_point10_y));
            // image_points.push_back(Point2d(football_point11_x, football_point11_y));
            // image_points.push_back(Point2d(football_point12_x, football_point12_y));
            // image_points.push_back(Point2d(football_point13_x, football_point13_y));

            // 3D 特征点世界坐标，与像素坐标对应，单位是cm, 构造image_point3d
            // std::vector<Point3d> model_points;
            model_points.push_back(Point3d(-6.95f, +6.95f, 0)); 
            model_points.push_back(Point3d(+6.95f, -6.95f, 0));
            model_points.push_back(Point3d(+6.95f, +6.95f, 0));
            model_points.push_back(Point3d(0, 0, +6.95f));
            model_points.push_back(Point3d(-3.4750f, 0, +6.018877f));
            model_points.push_back(Point3d(0, +5.2125f, +4.596993f));
            // model_points.push_back(Point3d(-3.475f, +3.475f, +4.914392f));
            // model_points.push_back(Point3d(0, +3.475f, +6.018877f));
            // model_points.push_back(Point3d(+3.475f, +3.475f, +4.914392f));
            // model_points.push_back(Point3d(+3.475f, 0, +6.018877f));
            // model_points.push_back(Point3d(+3.475f, -3.475f, +4.914392f));
            // model_points.push_back(Point3d(0, -3.475f, +4.914392f));
            // model_points.push_back(Point3d(-3.475f, -3.475f, +4.914392f));
            // model_points.push_back(Point3d(-6.95f, -6.95f, 0));
        }else if(object_calss == 1) {// robot
            // 计算robot点,run_height=48
            int robot_point1_x = max(0, (cur_env_data->robot_xyxy[0] + cur_env_data->robot_xyxy[2])/2 - (cur_env_data->robot_xyxy[3] - cur_env_data->robot_xyxy[1])/2);
            int robot_point1_y = cur_env_data->robot_xyxy[1];
            int robot_point2_x = min(640, (cur_env_data->robot_xyxy[0] + cur_env_data->robot_xyxy[2])/2 + (cur_env_data->robot_xyxy[3] - cur_env_data->robot_xyxy[1])/2);
            int robot_point2_y = cur_env_data->robot_xyxy[3];
            int robot_point3_x = robot_point2_x;
            int robot_point3_y = robot_point1_y;
            int robot_point4_x = robot_point1_x + (robot_point2_x - robot_point1_x) / 2;
            int robot_point4_y = robot_point1_y + (robot_point2_y - robot_point1_y) / 2;
            int robot_point5_x = cur_env_data->robot_xyxy[0] + (cur_env_data->robot_xyxy[2]-cur_env_data->robot_xyxy[0])/4;
            int robot_point5_y = robot_point4_y;
            int robot_point6_x = robot_point4_x;
            int robot_point6_y = robot_point1_y + (robot_point2_y - robot_point1_y) *7/8;
            int robot_point7_x = robot_point5_x;
            int robot_point7_y = robot_point1_y - (robot_point2_y - robot_point1_y) / 4;
            int robot_point8_x = robot_point4_x;
            int robot_point8_y = robot_point7_y;
            int robot_point9_x = robot_point2_x / 2;
            int robot_point9_y = robot_point1_y / 2;
            int robot_point10_x= robot_point2_x / 2;
            int robot_point10_y= robot_point4_y;
            int robot_point11_x= robot_point2_x / 2;
            int robot_point11_y= robot_point2_y - (robot_point2_y - robot_point1_y) / 4;
            int robot_point12_x= robot_point4_x;
            int robot_point12_y= robot_point2_y / 2;
            int robot_point13_x= robot_point2_x / 2;
            int robot_point13_y= robot_point11_y;
            int robot_point14_x = robot_point1_x;
            int robot_point14_y = robot_point2_y;

            // 构造image_point2d
            // vector<Point2d> image_points;
            image_points.push_back(Point2d(robot_point1_x, robot_point1_y));
            image_points.push_back(Point2d(robot_point2_x, robot_point2_y));
            image_points.push_back(Point2d(robot_point3_x, robot_point3_y));
            image_points.push_back(Point2d(robot_point4_x, robot_point4_y));
            image_points.push_back(Point2d(robot_point5_x, robot_point5_y));
            image_points.push_back(Point2d(robot_point6_x, robot_point6_y));
            // image_points.push_back(Point2d(football_point7_x, football_point7_y));
            // image_points.push_back(Point2d(football_point8_x, football_point8_y));
            // image_points.push_back(Point2d(football_point9_x, football_point9_y));
            // image_points.push_back(Point2d(football_point10_x, football_point10_y));
            // image_points.push_back(Point2d(football_point11_x, football_point11_y));
            // image_points.push_back(Point2d(football_point12_x, football_point12_y));
            // image_points.push_back(Point2d(football_point13_x, football_point13_y));

            // 3D 特征点世界坐标，与像素坐标对应，单位是cm, 构造image_point3d
            // std::vector<Point3d> model_points;
            model_points.push_back(Point3d(-6.95f*6.906, +6.95f*6.906, 0)); 
            model_points.push_back(Point3d(+6.95f*6.906, -6.95f*6.906, 0));
            model_points.push_back(Point3d(+6.95f*6.906, +6.95f*6.906, 0));
            model_points.push_back(Point3d(0, 0, +6.95f*6.906));
            model_points.push_back(Point3d(-3.4750f*6.906, 0, +6.018877f*6.906));
            model_points.push_back(Point3d(0, +5.2125f*6.906, +4.596993f*6.906));
            // model_points.push_back(Point3d(-3.475f, +3.475f, +4.914392f));
            // model_points.push_back(Point3d(0, +3.475f, +6.018877f));
            // model_points.push_back(Point3d(+3.475f, +3.475f, +4.914392f));
            // model_points.push_back(Point3d(+3.475f, 0, +6.018877f));
            // model_points.push_back(Point3d(+3.475f, -3.475f, +4.914392f));
            // model_points.push_back(Point3d(0, -3.475f, +4.914392f));
            // model_points.push_back(Point3d(-3.475f, -3.475f, +4.914392f));
            // model_points.push_back(Point3d(-6.95f, -6.95f, 0));
        }
        // cal object distance
        float distance = cal_distance(image_points, model_points);
        // cal object pos
        vector<float> res(3, 0);
        if(object_calss == 0) {
            res[0] = 0;
            res[1] = 6.95;
            res[2] = sqrt(pow(distance, 2) - pow((48-6.95), 2));
        }else if(object_calss == 1){
            float cd_distance = sqrt(pow(distance, 2) - pow(24, 2));
            res[0] = cd_distance * cos(angleToRadian(football_angle - robot_angle));
            res[1] = 24;
            res[2] = cd_distance * sin(angleToRadian(football_angle - robot_angle));
        }
        return res;
    }

    // 构造辅助请求包
    void createUdpDate(UdpData& data ,worker_brain_pkg::calculate_position_result cal_pos_res) {
        if(cal_pos_res.football_xyxy.size() == 0) {
            for(int i = 0; i < 4; i++) {
                data.football_xyxy[i] = 0;
            }
        }else {
            for(int i = 0; i < cal_pos_res.football_xyxy.size(); i++) {
                data.football_xyxy[i] = cal_pos_res.football_xyxy[i];
            }
        }

        if(cal_pos_res.goal_xyxy.size() == 0) {
            for(int i = 0; i < 4; i++) {
                data.goal_xyxy[i] = 0;
            }
        }else {
            for(int i = 0; i < cal_pos_res.goal_xyxy.size(); i++) {
                data.goal_xyxy[i] = cal_pos_res.goal_xyxy[i];
            }
        }

        if(cal_pos_res.net_xyxy.size() == 0) {
            for(int i = 0; i < 4; i++) {
                data.net_xyxy[i] = 0;
            }
        }else {
            for(int i = 0; i < cal_pos_res.net_xyxy.size(); i++) {
                data.net_xyxy[i] = cal_pos_res.net_xyxy[i];
            }
        }

        if(cal_pos_res.robot_xyxy.size() == 0) {
            for(int i = 0; i < 4; i++) {
                data.robot_xyxy[i] = 0;
            }
        }else {
            for(int i = 0; i < cal_pos_res.robot_xyxy.size(); i++) {
                data.robot_xyxy[i] = cal_pos_res.robot_xyxy[i];
            }
        }

        if(cal_pos_res.penalty_mark_xyxy.size() == 0) {
            for(int i = 0; i < 4; i++) {
                data.penalty_mark_xyxy[i] = 0;
            }
        }else {
            for(int i = 0; i < cal_pos_res.penalty_mark_xyxy.size(); i++) {
                data.penalty_mark_xyxy[i] = cal_pos_res.penalty_mark_xyxy[i];
            }
        }
        
        if(cal_pos_res.center_circle_xyxy.size() == 0) {
            for(int i = 0; i < 4; i++) {
                data.center_circle_xyxy[i] = 0;
            }
        }else {
            for(int i = 0; i < cal_pos_res.center_circle_xyxy.size(); i++) {
                data.center_circle_xyxy[i] = cal_pos_res.center_circle_xyxy[i];
            }
        }
    }

    // 循环找球状态相关参数初始化
    void findFootballParamInit() {
        find_football_state_sustain_frames_nums = 0;
        next_find_football_state_frame = 0;
        angle_vec_pos == 0;
    }
    void FindRobotParamInit() {
        find_robot_state_sustain_frames_nums = 0;
        next_find_robot_state_frame = 0;
        angle_vec_pos == 0;
    }

    // getNextStateByEnvCurState 由环境和当前状态得到下一个状态，并执行下一个状态
    State getNextStateByEnvCurState() {
        switch (cur_state) {
            case State::Initial:
                // 状态是初始化，则进入找球状态
                std::cout << "[" << frame << "]初始化结束进入找球状态" << std::endl;
                return State::FindFootball;
                break;
            case State::FindFootball:
                if(pre_state != State::FindFootball) {
                    findFootballParamInit();
                    next_find_football_state_frame = frame + FIND_FOOTBALL_STATE_FRAME_INTERVAL;
                    std::cout << "[" << frame << "]找球状态开始，相关参数已初始化" << std::endl;
                    return State::FindFootball;
                }
                // 如果找球过程中发现了球且大于FOUND_FOOTBALL_TRIGGER帧
                if (cur_env_data -> football_xyxy.size() != 0 && football_sustain_frames_nums >= FOUND_FOOTBALL_TRIGGER) {
                    // 头部相关参数归位，停止继续转动摄像头
                    std::cout << "[" << frame << "]连续发现球，退出找球状态，改变状态为跟球" << std::endl;
                    return State::FollowFootball;
                }
                // 没有找到球，当前帧是下一个转动摄像头的帧,转动摄像头
                if(frame >= next_find_football_state_frame) {
                    controlHead(angle_vec[angle_vec_pos][0], angle_vec[angle_vec_pos][1], false, false);
                    angle_vec_pos++;
                    // 循环数组，并向右原地转圈
                    if(angle_vec_pos == angle_vec.size()){
                        angle_vec_pos = 0;
                        // 暂时删掉了向右转圈
                        // runWalk(0, default_walk_width, false, false, angleToRadian(-30));
                    }
                    next_find_football_state_frame = frame + FIND_FOOTBALL_STATE_FRAME_INTERVAL;
                    std::cout << "[" << frame << "]找球模式下没有找到球，继续找球" << std::endl;
                    return State::FindFootball;
                }
                return State::FindFootball;
                break;
            case State::FollowFootball:
            {
                int central_point_x, central_point_y;
                if(cur_env_data->football_xyxy.size() != 0){
                    central_point_x = cur_env_data->football_xyxy[0] + (cur_env_data->football_xyxy[2] - cur_env_data->football_xyxy[0]) / 2; // 640
                    central_point_y = cur_env_data->football_xyxy[1] + (cur_env_data->football_xyxy[3] - cur_env_data->football_xyxy[1]) / 2; // 480
                }else {
                    return State::FindFootball;
                }
                //if football in mid
                if(central_point_x > FOLLOW_FPOINT_EDGE_LEFT && central_point_x < FOLLOW_FPOINT_EDGE_RIGHT && 
                central_point_y < FOLLOW_FPOINT_EDGE_DOWN && central_point_y > FOLLOW_FPOINT_EDGE_UP) {
                    //... 
                    football_angle = cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle;
                    football_3d_pos = cal_object_pos(0);
                    return State::FindRobot;
                }else {
                    adjHeadFollowObject(cur_env_data->football_xyxy);
                }
                break;
            }
            case State::FindRobot:
                if(pre_state != State::FindRobot) {
                    FindRobotParamInit();
                    next_find_robot_state_frame = frame + FIND_FOOTBALL_STATE_FRAME_INTERVAL;
                    std::cout << "[" << frame << "]找球状态开始，相关参数已初始化" << std::endl;
                    return State::FindRobot;
                }
                // 如果找球过程中发现了球且大于FOUND_FOOTBALL_TRIGGER帧
                if (cur_env_data -> robot_xyxy.size() != 0 && robot_sustain_frames_nums >= FOUND_FOOTBALL_TRIGGER) {
                    // 头部相关参数归位，停止继续转动摄像头
                    std::cout << "[" << frame << "]连续发现球，退出找球状态，改变状态为跟球" << std::endl;
                    return State::FollowRobot;
                }
                // 没有找到球，当前帧是下一个转动摄像头的帧,转动摄像头
                if(frame >= next_find_robot_state_frame) {
                    controlHead(angle_vec[angle_vec_pos][0], angle_vec[angle_vec_pos][1], false, false);
                    angle_vec_pos++;
                    // 循环数组，并向右原地转圈
                    if(angle_vec_pos == angle_vec.size()){
                        angle_vec_pos = 0;
                        // 暂时删掉了向右转圈
                        // runWalk(0, default_walk_width, false, false, angleToRadian(-30));
                    }
                    next_find_robot_state_frame = frame + FIND_FOOTBALL_STATE_FRAME_INTERVAL;
                    std::cout << "[" << frame << "]找球模式下没有找到球，继续找球" << std::endl;
                    return State::FindRobot;
                }
                return State::FindRobot;
                break;
            case State::FollowRobot:
            {
                int central_point_x, central_point_y;
                if(cur_env_data->robot_xyxy.size() != 0){
                    central_point_x = cur_env_data->robot_xyxy[0] + (cur_env_data->robot_xyxy[2] - cur_env_data->robot_xyxy[0]) / 2; // 640
                    central_point_y = cur_env_data->robot_xyxy[1] + (cur_env_data->robot_xyxy[3] - cur_env_data->robot_xyxy[1]) / 2; // 480
                }else {
                    return State::FindFootball;
                }
                //if robot in mid
                if(central_point_x > FOLLOW_FPOINT_EDGE_LEFT && central_point_x < FOLLOW_FPOINT_EDGE_RIGHT && 
                central_point_y < FOLLOW_FPOINT_EDGE_DOWN && central_point_y > FOLLOW_FPOINT_EDGE_UP) {
                    //... 
                    robot_angle = cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle;
                    robot_3d_pos = cal_object_pos(1);
                    return State::FindRobot;
                }else {
                    adjHeadFollowObject(cur_env_data->robot_xyxy);
                }
                break;
            }
            case State::Move:
                {
                    float football_robot_distance = sqrt(pow(robot_3d_pos[0] - football_3d_pos[0], 2) + pow(robot_3d_pos[2] - football_3d_pos[2], 2));
                    if(football_robot_distance < 10) {
                        return State::Kick;
                    }else {
                        data.state = 1;
                        data.var_theta = football_angle - robot_angle;
                        // udp发送数据给worker
                        sender.SendData(data);
                        pauseNFrame(100);
                        return State::FindFootball;
                    }
                }
                break;
            case State::Kick:
                data.state = 2;
                if(robot_3d_pos[0] < 0) {
                    data.kick_leg = 4;//left
                }else {
                    data.kick_leg = 3;//right
                }
                // udp发送数据给worker
                sender.SendData(data);
                pauseNFrame(100);
                return State::FindFootball;
                break;
            default:
                return State::Initial;
        }
    }

};

#endif