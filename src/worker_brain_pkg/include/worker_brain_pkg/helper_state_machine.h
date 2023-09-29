#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Int16.h"
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
    FollowFootball,   // 跟ball状态
    Move,           // 移动状态
    FindRobot,      // 找机器人状态
    FollowRobot,
    Sender,         // 发送信息状态
    Stopped         // 停止状态
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
class HelperStateMachine {
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

    // 角度
    const std::vector<std::vector<float>> angle_vec = 
    {{0, 20}, 
    {30, 20}, {60, 20}, 
    {60, 40}, {30, 40}, {0, 40}, {-30, 40}, {-60, 40},
    {-60, 60}, {-30, 60}, {0, 60}, {30, 60}, {60, 60},
    {30, 30}};
    int angle_vec_pos = 0;

    //状态机构造函数
    HelperStateMachine() {
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

    // 第一版辅助定位worker代码
    // int walk_rotation_flag = 0;// 0代表直走，1代表左转，-1代表右转。
    // void execute_action_by_cur_data(UdpData &data) {
    //     std::cout << "1" << std::endl;
    //     frame++;
    //     // 条件限制
    //     if(data.football_xyxy[2] == 0 || data.robot_xyxy[2] == 0) return;

    //     //当前帧不是暂停帧才可以行动
    //     if(frame <= until_pause_frame) {
    //         // if(cur_env_data -> parameters_srv_data.stop_walk_flag == false) {
    //         //     runWalk(0, default_walk_width, true, false, 0);
    //         // }
    //         return;
    //     }
    //     // std::cout << data.football_xyxy.size() << " " << data.robot_xyxy.size() << std::endl;
    //     // std::cout << data.football_xyxy[0] << " " << data.robot_xyxy[0] << std::endl;
    //     std::cout << data.football_xyxy[0] << " " << data.football_xyxy[2] << std::endl;
    //     std::cout << "2" << std::endl;
    //     if(data.football_xyxy[0] >= data.robot_xyxy[2]) {
    //         //如果已经是左转了
    //         if(walk_rotation_flag == 1) {
    //             pauseNFrame(15);
    //             return;
    //         }
    //         std::cout << "3" << std::endl;
    //         // 机器人左拐
    //         runWalk(default_walk_length, default_walk_width, false, false, angleToRadian(MAX_WALK_ROTATION_ANGLE));
    //         cur_walk_rotation_angle -= MAX_WALK_ROTATION_ANGLE;
    //         walk_rotation_flag = 1;
    //         std::cout << "球在机器人左边改为右拐" << std::endl;
    //     }else if(data.football_xyxy[2] <= data.robot_xyxy[0]){
    //         //如果已经是右转了
    //         if(walk_rotation_flag == -1) {
    //             pauseNFrame(15);
    //             return;
    //         }
    //         std::cout << "4" << std::endl;
    //         // 机器人右拐
    //         runWalk(default_walk_length, default_walk_width, false, false, angleToRadian(-MAX_WALK_ROTATION_ANGLE));
    //         cur_walk_rotation_angle += MAX_WALK_ROTATION_ANGLE;
    //         walk_rotation_flag = -1;
    //         std::cout << "球在机器人右边改为右拐" << std::endl;
    //     }else {
    //         // 改回直走
    //         std::cout << "5" << std::endl;
    //         runWalk(default_walk_length, default_walk_width, false, false, angleToRadian(-cur_walk_rotation_angle));
    //         cur_walk_rotation_angle = 0;
    //         walk_rotation_flag = 0;
    //         std::cout << "改回直走" << std::endl;
    //     }
    //     pauseNFrame(60);
    //     std::cout << "结束一轮辅助定位" << std::endl;
    // }

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

    // 保存中断状态
    void save_interrupt_state(State state) {
        pre_interrupt_state = state;
    }

    // 循环找球状态相关参数初始化
    void findFootballParamInit() {
        find_football_state_sustain_frames_nums = 0;
        next_find_football_state_frame = 0;
        angle_vec_pos == 0;
    }

    // 循环找Robot状态相关参数初始化
    void findRobotParamInit() {
        find_robot_state_sustain_frames_nums = 0;
        next_find_robot_state_frame = 0;
        angle_vec_pos == 0;
    }

    // 循环找网状态相关参数初始化
    void findNetParamInit() {
        find_net_state_sustain_frames_nums = 0;
        next_find_football_state_frame = 0;
        angle_vec_pos = 0;
        // 摄像头设置为找门
        head_cur_task = FIND_NET_TASK;
    }

    // 读取头部位置的信息
    RobotHeadPosAngle readHeadPos() {
        RobotHeadPosAngle temp_robot_head_pos_angle;
        fin.open(head_pos_file_path, std::ios::in);
        fin >> temp_robot_head_pos_angle.neck_rotation_theta_angle >> temp_robot_head_pos_angle.neck_front_swing_theta_angle;
        fin.close();
        return temp_robot_head_pos_angle;
    }

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

    // 设置N帧暂停
    void pauseNFrame(long long N) {
        until_pause_frame = frame + N;
    }

    // 根据球的位置调用摄像头跟球
    void adjHeadFollowFootBall(std::vector<int> football_xyxy){
        //可以控制帧数
        if(football_xyxy.size() == 0) {
            return;
        }
        int central_point_x = football_xyxy[0] + (football_xyxy[2] - football_xyxy[0]) / 2; // 640
        int central_point_y = football_xyxy[1] + (football_xyxy[3] - football_xyxy[1]) / 2; // 480
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

    // 根据机器人的位置调用摄像头跟机器人
    void adjHeadFollowRobot(std::vector<int> robot_xyxy){
        //可以控制帧数
        if(robot_xyxy.size() == 0) {
            return;
        }
        int central_point_x = robot_xyxy[0] + (robot_xyxy[2] - robot_xyxy[0]) / 2; // 640
        int central_point_y = robot_xyxy[1] + (robot_xyxy[3] - robot_xyxy[1]) / 2; // 480
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

    // getNextStateByEnvCurState 由环境和当前状态得到下一个状态，并执行下一个状态
    State getNextStateByEnvCurState() {
        //根据目标转动摄像头
        if(cur_object == 0) {
            if(cur_env_data->football_xyxy.size() > 0){
                std::cout << cur_env_data->football_xyxy[0] << std::endl;
                adjHeadFollowFootBall(cur_env_data->football_xyxy);
            }
        }else if(cur_object == 1) {
            if(cur_env_data->robot_xyxy.size() > 0){
                adjHeadFollowRobot(cur_env_data->robot_xyxy);
            }
        }
        //当前帧不是暂停帧才可以行动
        if(frame <= until_pause_frame) {
            if(cur_env_data -> parameters_srv_data.stop_walk_flag == false)
                runWalk(0, default_walk_width, true, false, 0);
            return cur_state;
        }

        switch (cur_state) {
            case State::Initial:
                // 状态是初始化，则进入找球状态
                std::cout << "[" << frame << "]初始化结束进入找球状态" << std::endl;
                return State::FindFootball;
                break;

            //找球模式要改成每帧都判断，通过frame求余或者计数求余来循环找球，所有状态都要视觉跟球
            case State::FindFootball:
                if(cur_env_data -> parameters_srv_data.stop_walk_flag == false) {
                    runWalk(0, default_walk_width, true, false, 0);
                }
                // 刚刚进入找球模式，初始化找球参数,FIND_FOOTBALL_FRAME_INTERVAL帧后动循环摄像头
                if(pre_state != State::FindFootball) {
                    findFootballParamInit();
                    next_find_football_state_frame = frame + FIND_FOOTBALL_STATE_FRAME_INTERVAL;
                    std::cout << "[" << frame << "]找球状态开始，相关参数已初始化" << std::endl;
                    return State::FindFootball;
                }
                // 如果找球过程中发现了球且大于FOUND_FOOTBALL_TRIGGER帧
                if (cur_env_data -> football_xyxy.size() != 0 && football_sustain_frames_nums >= FOUND_FOOTBALL_TRIGGER) {
                    // 头部相关参数归位，停止继续转动摄像头
                    std::cout << "[" << frame << "]连续发现球，退出找球状态，改变状态为FollowFootball" << std::endl;
                    return State::FollowFootball;
                }
                // 没有找到球，当前帧是下一个转动摄像头的帧,转动摄像头
                if(frame == next_find_football_state_frame) {
                    controlHead(angle_vec[angle_vec_pos][0], angle_vec[angle_vec_pos][1], false, false);
                    angle_vec_pos++;
                    // 循环数组
                    if(angle_vec_pos == angle_vec.size()){
                        angle_vec_pos = 0;
                        // runWalk(0, default_walk_width, false, false, angleToRadian(-30));
                    }
                    next_find_football_state_frame = frame + FIND_FOOTBALL_STATE_FRAME_INTERVAL;
                    std::cout << "[" << frame << "]找球模式下没有找到球，继续找球" << std::endl;
                    return State::FindFootball;
                }
                return State::FindFootball;
                break;

            case State::FollowFootball:
                //如果在跟过程中球丢失
                if(football_lost_frames_nums >= LOST_FOOTBALL_TRIGGER) {
                    confirming_kick = false;
                    std::cout << "[" << frame << "]跟球过程中球丢失,回到找球状态" << std::endl;
                    return State::FindFootball;
                }
                // 如果球的连续帧大于FOUND_FOOTBALL_TRIGGER,开始移动
                if(football_sustain_frames_nums > FOUND_FOOTBALL_TRIGGER) {
                    if(confirming_kick == false) {
                        pauseNFrame(30);
                        confirming_kick = true;
                    }else {
                        confirming_kick = false;
                        std::cout << "[" << frame << "]跟球成功，进入移动状态" << std::endl;
                        return State::Move;
                    }
                }
                return State::FollowFootball;
                break;

            case State::Move:
                //如果在跟过程中球丢失
                if(football_lost_frames_nums >= LOST_FOOTBALL_TRIGGER) {
                    confirming_kick = false;
                    std::cout << "[" << frame << "]跟球过程中球丢失,回到找球状态" << std::endl;
                    return State::FindFootball;
                }
                // 如果摄像头偏移大于10度则左右平移
                if(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle > 10 || 
                    cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle < -10) {
                    confirming_kick = false;
                    // 摄像头x轴如果是+,说明物体在左，则身体要往左
                    if(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle > 10) {
                        //向左平移
                        // "/parallelMove"
                        // 0 左 1 右
                        //给1.5的行动时间
                        std_msgs::Int16 msg;
                        msg.data = 0;//左
                        pub_parallelMove.publish(msg);
                        std::cout << "向左平移三步" << std::endl;
                        ros::Duration(1.5).sleep();
                    }else {
                        //向右平移
                        // "/parallelMove"
                        // 0 左 1 右
                        //给1.5的行动时间
                        std_msgs::Int16 msg;
                        msg.data = 1;
                        pub_parallelMove.publish(msg);
                        std::cout << "向右平移三步" << std::endl;
                        ros::Duration(2.5).sleep();
                    }
                    //暂停75帧给下位机Move移动身体的时间
                    pauseNFrame(75);
                    std::cout << "[" << frame << "]移动完成，暂停75帧确认后继续进入移动状态" << std::endl;
                    return State::Move;
                }else {//如果摄像头偏转角度小于10度，进入找机器人阶段
                    // Move finish
                    std::cout << "[" << frame << "]移动完成进入找机器人状态" << std::endl;
                    return State::FindRobot;
                }
                break;

            case State::FindRobot:
                cur_object = 1;
                if(cur_env_data -> parameters_srv_data.stop_walk_flag == false) {
                    runWalk(0, default_walk_width, true, false, 0);
                }
                // 刚刚进入找球模式，初始化找球参数,FIND_FOOTBALL_FRAME_INTERVAL帧后动循环摄像头
                if(pre_state != State::FindRobot) {
                    findFootballParamInit();
                    next_find_football_state_frame = frame + FIND_FOOTBALL_STATE_FRAME_INTERVAL;
                    std::cout << "[" << frame << "]找robot状态开始，相关参数已初始化" << std::endl;
                    return State::FindRobot;
                }
                // 如果找球过程中发现了球且大于FOUND_FOOTBALL_TRIGGER帧
                if (cur_env_data -> robot_xyxy.size() != 0 && football_sustain_frames_nums >= FOUND_FOOTBALL_TRIGGER) {
                    // 头部相关参数归位，停止继续转动摄像头
                    std::cout << "[" << frame << "]连续发现robot，退出找robot状态，改变状态为sender" << std::endl;
                    return State::Sender;
                }
                // 没有找到球，当前帧是下一个转动摄像头的帧,转动摄像头
                if(frame == next_find_football_state_frame) {
                    controlHead(angle_vec[angle_vec_pos][0], angle_vec[angle_vec_pos][1], false, false);
                    angle_vec_pos++;
                    // 循环数组
                    if(angle_vec_pos == angle_vec.size()){
                        angle_vec_pos = 0;
                        // runWalk(0, default_walk_width, false, false, angleToRadian(-30));
                    }
                    next_find_football_state_frame = frame + FIND_FOOTBALL_STATE_FRAME_INTERVAL;
                    std::cout << "[" << frame << "]找robot模式下没有找到球，继续找球" << std::endl;
                    return State::FindRobot;
                }
                return State::FindRobot;
                break;

            case State::Sender:
                // 定义在了全局变量中
                // UdpData data;
                // createUdpDate(data, position_res);

                // data.football_xyxy = cal_pos_res.football_xyxy;
                // data.goal_xyxy = cal_pos_res.goal_xyxy;
                // data.net_xyxy = cal_pos_res.net_xyxy;
                // data.robot_xyxy = cal_pos_res.robot_xyxy;
                // data.penalty_mark_xyxy = cal_pos_res.penalty_mark_xyxy;
                // data.distance = cal_pos_res.distance;
                // data.kf_distance = cal_pos_res.kf_distance;
                // data.robot_distance = 0;
                // udp发送数据给worker
                sender.SendData(data);
                std::cout << "Sent data UdpData: UdpData.distance = " << data.distance << std::endl;
                return State::Sender;

            default:
                // ROS_INFO("逻辑边界，机器人初始化");
                std::cout << "[" << frame << "]逻辑边界，机器人初始化" << std::endl;
                return State::Initial;
                break;


            // case State::FollowObject:
            //     //如果在跟过程中球丢失
            //     if(football_lost_frames_nums >= LOST_FOOTBALL_TRIGGER) {
            //         confirming_kick = false;
            //         std::cout << "[" << frame << "]跟球过程中球丢失,回到找球状态" << std::endl;
            //         return State::FindFootball;
            //     }

            //     // if(cur_env_data->football_xyxy.size() > 0){
            //     //     adjHeadFollowFootBall(cur_env_data->football_xyxy);
            //     // }

            //     //如果球很远，才要调整身体位置，否则直接踢球NO_ADJUST_BODY_DISTANCE
            //     if(abs(cur_env_data -> kf_distance) >= NO_ADJUST_BODY_DISTANCE && cur_env_data -> distance != 0) {
            //         // 踢球和行走中的身体角度判断
            //         // 如果摄像头偏了，給2s调整身体
            //         if(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle > FOLLOW_FOOTBALL_DEVIATE_TRIGGER_ANGLE || 
            //         cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle < -FOLLOW_FOOTBALL_DEVIATE_TRIGGER_ANGLE) {
            //             confirming_kick = false;
            //             // 摄像头x轴如果是-,说明物体在右，则身体要往右，也就是walk偏转角度要为负
            //             if(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle > MAX_WALK_ROTATION_ANGLE) {
            //                 runWalk(0, default_walk_width, false, false, angleToRadian(MAX_WALK_ROTATION_ANGLE));
            //             }else {
            //                 runWalk(0, default_walk_width, false, false, angleToRadian(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle));
            //             }
            //             // 摄像头的x轴调整到0
            //             controlHead(0, cur_env_data->robot_head_pos_angle.neck_front_swing_theta_angle, false, false);
            //             std::cout << "[" << frame << "]头偏过大，暂停调整身体;distance:" << abs(cur_env_data -> kf_distance) << std::endl;
            //             //暂停60帧给下位机移动身体和头部的时间
            //             pauseNFrame(60);
            //             return State::FollowObject;
            //         }
            //     }
            //     // 踢球前的距离判断
            //     //如果到了踢球的距离,!=0防止没有找到球传空数据的情况
            //     // 暂停30帧确认球的距离和位置
            //     if(abs(cur_env_data -> kf_distance) < KICK_FOOTBALL_DISTANCE_TRIGGER && cur_env_data -> distance != 0 && 
            //     football_sustain_frames_nums >= FOUND_FOOTBALL_TRIGGER) {
            //         if(confirming_kick == false) {
            //             if(cur_env_data -> parameters_srv_data.stop_walk_flag == false)
            //                 runWalk(0, default_walk_width, true, false, 0);
            //                 pauseNFrame(30);
            //                 confirming_kick = true;
            //                 std::cout << "[" << frame << "]似乎球距离已经可以踢，暂停30帧确认一下" << std::endl;
            //                 return State::FollowObject;
            //         } else {
            //             //如果球门和网在正确范围内，则踢球
            //             if(goal_flag == true || net_flag == true) {
            //                 confirming_kick = false;
            //                 goal_flag = false;
            //                 net_flag = false;
            //                 std::cout << "[" << frame << "]暂停30帧确认后，距离和目标确认无误，进入踢球状态" << std::endl;
            //                 return State::KickFootball;
            //             }else {
            //                 std::cout << "[" << frame << "]暂停30帧确认后，网和球门位置不对，进入找网状态" << std::endl;
            //                 return State::FindNet;
            //             }
            //         }
            //     }
            //     //摄像头没偏，也没丢失球，则继续walk跟球，并实时调整摄像头位置
            //     //（可以加一个参数服务器限制）
            //     // if(abs(cur_env_data->kf_distance) <= 100) {
            //     //     runWalk(default_walk_length/2, default_walk_width, false, false, 0);
            //     // }else {
            //     runWalk(default_walk_length, default_walk_width, false, false, 0);
            //     // }
            //     // adjHeadFollowFootBall(cur_env_data->football_xyxy);
            //     return State::FollowObject;
            //     break;

            // // 找网模式
            // case State::FindNet:
            //     runWalk(0, default_walk_width, true, false, 0);
            //     //初次进入找网，需要几帧去更新环境信息
            //     if(find_net_frame == 0) {
            //         std::cout << "[" << frame << "]first become findnet state" << std::endl;
            //         // 保存头位置
            //         temp_neck_rotation_theta_angle = cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle;
            //         temp_neck_front_swing_theta_angle = cur_env_data->robot_head_pos_angle.neck_front_swing_theta_angle;
            //         // 球网方向判断（注意环境变量不会更新了）
            //         controlHead(0, 0, false, false);
            //         find_net_frame++;
            //         return State::FindNet;
            //         break;
            //     }
            //     // 更新环境变量
            //     if(find_net_frame < 30) {
            //         // 开始设置网和球门的信息in these 30 frames
            //         // we can find 2 frame is ok in 30 frames
            //         if(cur_env_data->net_xyxy.size() > 0){
            //             int net_central_point_x = cur_env_data->net_xyxy[0] + (cur_env_data->net_xyxy[2] - cur_env_data->net_xyxy[0]) / 2; // 640
            //             int net_central_point_y = cur_env_data->net_xyxy[1] + (cur_env_data->net_xyxy[3] - cur_env_data->net_xyxy[1]) / 2; // 480
            //             // 只要目标不偏移在二维图像边缘(偏中心)
            //             if(net_sustain_frames_nums>=2 && net_central_point_x<560 && net_central_point_x>80) {
            //                 net_flag = true;
            //             }
            //         }
            //         if(cur_env_data->goal_xyxy.size() > 0){
            //             int goal_central_point_x = cur_env_data->goal_xyxy[0] + (cur_env_data->goal_xyxy[2] - cur_env_data->goal_xyxy[0]) / 2; // 640
            //             int goal_central_point_y = cur_env_data->goal_xyxy[1] + (cur_env_data->goal_xyxy[3] - cur_env_data->goal_xyxy[1]) / 2; // 480
            //             // 只要目标不偏移在二维图像边缘(偏中心)
            //             if(goal_sustain_frames_nums>=2 && goal_central_point_x<560 && goal_central_point_x>80) {
            //                 goal_flag = true;
            //             }
            //         }
            //         find_net_frame++;
            //         return State::FindNet;
            //         break;
            //     }
            //     find_net_frame = 0;

            //     //如果没找到网和门，持续找
            //     if(goal_flag==false && net_flag==false) {
            //         std::cout << "[" << frame << "]dont find net" << std::endl;
            //         //向左平移
            //         // "/parallelMove"
            //         // 0 左 1 右
            //         //给1.5的行动时间
            //         std_msgs::Int16 msg;
            //         msg.data = 0;//左
            //         pub_parallelMove.publish(msg);
            //         std::cout << "向左平移三步" << std::endl;
            //         ros::Duration(2.5).sleep();
            //         //向右转圈
            //         runWalk(0, default_walk_width, false, false, angleToRadian(-30));
            //         std::cout << "向右转30度" << std::endl;
            //         pauseNFrame(60);
            //         return State::FindNet;
            //     }else {
            //         //将看球的头部还原归位
            //         controlHead(temp_neck_rotation_theta_angle, temp_neck_front_swing_theta_angle, false, false);
            //         std::cout << "[" << frame << "]found net" << std::endl;
            //         pauseNFrame(15);
            //         return State::FollowObject;
            //     }

            // case State::KickFootball:
            //     // 停止运动
            //     if(cur_env_data -> parameters_srv_data.stop_walk_flag == false)
            //         runWalk(0, default_walk_width, true, false, 0);
            //     // 调用踢球
            //     kickFootball();
            //     std::cout << "[" << frame << "]调用踢球步态，停止运动，进入特殊步态状态" << std::endl;
            //     return State::SpecialGait;

            // case State::SpecialGait:
            //     if(!cur_env_data->parameters_srv_data.stop_special_gait_flag) {
            //         std::cout << "[" << frame << "]参数服务器还在特殊步态状态，保持特殊步态状态" << std::endl;
            //         return State::SpecialGait;
            //     }else {
            //         std::cout << "[" << frame << "]参数服务器结束特殊步态状态，进入跟球状态" << std::endl;
            //         return State::FollowObject;
            //     }
            //     break;
        }
    }
};

#endif