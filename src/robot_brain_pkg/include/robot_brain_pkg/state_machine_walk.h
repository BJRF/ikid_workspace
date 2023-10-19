#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "robot_brain_pkg/calculate_position_result.h"
#include "robot_brain_pkg/cmd_walk.h"
#include "robot_brain_pkg/robot_head_pos.h"
#include "robot_brain_pkg/head_contol_by_brain.h"

// #include "robot_brain_pkg/tool.h"

#define PI acos(-1)
#define FIND_FOOTBALL_STATE_FRAME_INTERVAL 8
#define FOUND_FOOTBALL_TRIGGER 4
#define LOST_FOOTBALL_TRIGGER 30
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
#define NO_ADJUST_BODY_DISTANCE 60
// #define KICK_FOOTBALL_DISTANCE_TRIGGER 30

float KICK_FOOTBALL_DISTANCE_TRIGGER;

// 全局变量
// ros 发布
ros::Publisher pub_walk;
ros::Publisher pub_spcial;
ros::Publisher pub_head_control;
ros::Publisher pub_parallelMove;

// 文件输入输出流
std::fstream fout;
std::fstream fin;

const char head_pos_file_path[] = "/home/nvidia/ikid_ws/src/robot_brain_pkg/data/head_pos_angle.txt";

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
    robot_brain_pkg::cmd_walk cur_cmd_walk;
    /* 
        # 发送前转为弧度
        # 颈前摆：0-90度 上-下
        # 颈旋转：80-（-80）度 左-右
        float64 neck_rotation_theta  # 颈旋转关节角度
        float64 neck_front_swing_theta # 颈前摆关节角度
    */
    // 这里存的是弧度
    // robot_brain_pkg::robot_head_pos cur_robot_head_pos;
    // 这里存的是角度
    RobotHeadPosAngle robot_head_pos_angle;

    // 参数服务器数据
    ParametersSrvData parameters_srv_data;
    EnvData() {
        
    }
    EnvData(const robot_brain_pkg::calculate_position_result::ConstPtr& position_res, 
    const robot_brain_pkg::cmd_walk& cmd_walk, const RobotHeadPosAngle robot_head_pos_angle) {
        football_xyxy = position_res -> football_xyxy;
        goal_xyxy = position_res -> goal_xyxy;
        robot_xyxy = position_res -> robot_xyxy;
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
    AdjPosInPlace,  // 在原地调整位置
    Walk,           // 行走状态
    KickFootball,   // 踢球状态
    FindGoal,       // 找门状态
    FindNet,        // 找网状态
    FollowObject,   // 跟随状态（身体跟随，非头部视觉）
    SpecialGait,    // 特殊步态状态
    Paused,         // 暂停状态
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
class StateMachine {
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

    // 找球的连续帧数
    int find_football_state_sustain_frames_nums = 0;
    // 找wang的连续帧数
    int find_net_state_sustain_frames_nums = 0;

    // 下一个找球状态的帧
    long long next_find_football_state_frame = 0;

    // 下一个可以开始动的帧
    long long until_pause_frame = 0;

    // 下一个可以开始juece的帧
    long long until_pause_frame_action = 0;

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

    // 角度
    // const std::vector<std::vector<float>> angle_vec = 
    // {{0, 15}, 
    // {20, 15}, {40, 15}, {60, 15}, 
    // {70, 35}, {50, 35}, {30, 35}, {0, 35}, {-30, 35}, {-50, 35}, {-70, 35},
    // {-60, 60}, {-60, 35}, {-60, 15}, {-60, 0}, {-60, 20}, {-60, 40}, {-60, 60},
    // {-30, 75}, {0, 75},
    // {0, 50}};
    const std::vector<std::vector<float>> angle_vec = 
    {{-60, 60}, {-30, 60}, {0, 60}, {30, 60}, {60, 60},
    {60, 40}, {30, 40}, {0, 40}, {-30, 40}, {-60, 40},
    {-60, 20}, {-30, 20}, {0, 20}, {30, 20}, {60, 20}, 
    {60, 0}, {30, 0}, {0, 0}, {-30, 0}, {-60, 0},
    {30, 30}};
    int angle_vec_pos = 0;

    // 裁判盒信息
    // GameState 游戏状态
    // #define STATE_INITIAL 0.
    // #define STATE_READY 1
    // #define STATE_SET 2
    // #define STATE_PLAYING 3
    // #define STATE_FINISHED 4
    // RoboPenalty 机器人惩罚
    // #define PENALTY_NONE 0
    // #define HL_BALL_MANIPULATION 30
    // #define HL_PHYSICAL_CONTACT 31
    // #define HL_ILLEGAL_ATTACK 32
    // #define HL_ILLEGAL_DEFENSE 33
    // #define HL_PICKUP_OR_INCAPABLE 34
    // #define HL_SERVICE 35
    int GameState = 0;
    int RoboPenalty = 0;

    // 是否在准备阶段入场的标志位
    bool ready_flag = false;
    // ready锁
    bool ready_lock = false;

    // 入场走几秒
    int ready_walktime = 0;
    // 入场旋转多少度
    int ready_angle = 0;
    // 入场旋转几次
    int ready_turn_count = 0;

    // 该机器人是否是前锋
    bool is_forward = true;

    // 开始比赛后累计转了多少度
    float turn_sum = 0;

    //状态机构造函数
    StateMachine() {
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
        readConfig();
        readReadyWalktimeAngle();
        readIsForward();
    }

    // 更新环境信息(walk没有更新)
    void updateEnvData(const robot_brain_pkg::calculate_position_result::ConstPtr& position_res) {
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
        // 更新裁判盒命令
        ros::param::get("game_controller_gamestate", GameState);
        ros::param::get("game_controller_robopenalty", RoboPenalty);
    }

    // // 读取配置文件
    void readConfig() {
        const char kick_football_distance_trigger_file_path[] = "/home/nvidia/ikid_ws/src/robot_brain_pkg/data/kick_football_distance_trigger.txt";
        fin.open(kick_football_distance_trigger_file_path, std::ios::in);
        fin >> KICK_FOOTBALL_DISTANCE_TRIGGER;
        fin.close();
        std::cout << "读取到配置文件KICK_FOOTBALL_DISTANCE_TRIGGER:" << KICK_FOOTBALL_DISTANCE_TRIGGER << std::endl;
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

    // 读取ready态入场时间+旋转角度（连续4次旋转angle度）
    void readReadyWalktimeAngle() {
        fin.open("/home/nvidia/ikid_ws/src/robot_brain_pkg/data/ready_walktime_angle.txt", std::ios::in);
        fin >> ready_walktime >> ready_angle >> ready_turn_count;
        std::cout << "readReadyWalktimeAngle" << ready_walktime << " ready_angle" << ready_angle << " ready_turn_count" << ready_turn_count << std::endl;
        fin.close();
    }

    // 读取该机器人是否是前锋
    void readIsForward() {
        fin.open("/home/nvidia/ikid_ws/src/robot_brain_pkg/data/is_forward.txt", std::ios::in);
        fin >> is_forward;
        std::cout << "readIsForward" << is_forward << std::endl;
        fin.close();
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
        robot_brain_pkg::head_contol_by_brain head_contol_by_brain;
        head_contol_by_brain.neck_rotation_theta_angle = neck_rotation_theta_angle;
        head_contol_by_brain.neck_front_swing_theta_angle = neck_front_swing_theta_angle;
        head_contol_by_brain.is_find_state = is_find_state;
        head_contol_by_brain.is_follow_state = is_follow_state;
        pub_head_control.publish(head_contol_by_brain);
    }

    // 调用行走步态
    void runWalk(double sx, double sy, bool stop_walk, bool walk_with_ball, double var_theta) {
        robot_brain_pkg::cmd_walk walk;
        walk.sx = sx;
        walk.sy = sy;
        walk.stop_walk = stop_walk;
        walk.walk_with_ball = walk_with_ball;
        walk.var_theta = var_theta;
        turn_sum += var_theta;
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

    // 设置N帧暂停
    void pauseNFrameAndAction(long long N) {
        until_pause_frame_action = frame + N;
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
        // std::cout << "[" << frame << "]adj head left&right:"<<rotation_offset_angle 
        // << "up&down:" << front_offset_angle<<std::endl;
        // std::cout << cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle << std::endl;
        // std::cout << cur_env_data->robot_head_pos_angle.neck_front_swing_theta_angle << std::endl;
        if(frame % 3 == 0) { 
            controlHead(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle + rotation_offset_angle, 
            cur_env_data->robot_head_pos_angle.neck_front_swing_theta_angle + front_offset_angle, false, false);
        }
    }

    // getNextStateByEnvCurState 由环境和当前状态得到下一个状态，并执行下一个状态
    State getNextStateByEnvCurState() {
        //根据目标球转动摄像头
        if(cur_env_data->football_xyxy.size() > 0 && cur_state != State::FindNet){
            adjHeadFollowFootBall(cur_env_data->football_xyxy);
        }
        // else if(cur_env_data -> net_xyxy.size() > 0 && head_cur_task == FIND_NET_TASK) {
        //     adjHeadFollowFootBall(cur_env_data->net_xyxy);
        // }
        // return State::Initial;

        // 判断裁判盒指令
        // std::cout << GameState << " " << RoboPenalty << std::endl;
        if(GameState == 0 || GameState == 2 || GameState == 4 || RoboPenalty != 0) {
            runWalk(0, default_walk_width, true, false, 0);
            return cur_state;
        }

        // 裁判盒ready进场
        if(GameState == 1 && ready_flag == false) {
            std::cout << "[" << frame << "]裁判盒正在ready状态" << std::endl;
            //判断进场锁是否有被使用，防止重复下发下位机指令
            if(ready_lock == false) {
                // 加锁
                ready_lock = true;

                std::cout << "[" << frame << "]机器人开始进场" << std::endl;
                // 向前ready_walktime秒
                runWalk(default_walk_length, default_walk_width, false, false, 0);
                ros::Duration(ready_walktime).sleep();
                // 停顿0.5s
                runWalk(0, default_walk_width, true, false, 0);
                ros::Duration(0.5).sleep();
                // 转ready_angle*ready_turn_count度
                for(int i = 0; i < ready_turn_count; i++) {
                    runWalk(0, default_walk_width, false, false, angleToRadian(ready_angle));
                    ros::Duration(2).sleep();
                }
                runWalk(0, default_walk_width, true, false, 0);
                // 解锁
                ready_lock = false;
                // 进场结束
                ready_flag = true;
            }
            return State::Initial;
        }else if(GameState == 1 && ready_flag == true) { // 如果裁判盒指令是ready且机器人已经入场完成
            std::cout << "[" << frame << "]机器人正在进场" << std::endl;
            return State::Initial;
        }

        // std::cout << frame << std::endl;
        // std::cout << until_pause_frame_action << std::endl;
        if(frame <= until_pause_frame_action) {
            return cur_state;
        }

        //当前帧不是暂停帧才可以行动
        if(frame <= until_pause_frame) {
            if(cur_env_data -> parameters_srv_data.stop_walk_flag == false) {
                runWalk(0, default_walk_width, true, false, 0);
            }
            return cur_state;
        }

        /*
        前锋后卫逻辑
        决策1.0：
            前锋：
                1. 向其他所以机器人发送自己的信息
                2. 如果有后卫到近距离is_close_distance == true，则停止运动
            后卫：
                1. 如果前锋数量小于2，后卫发现则变为前锋
                2. 接受前锋信息确认前锋存活（保活）
        */
        /*
        决策2.0：
            上场机器人入场时从前往后布置
            前锋标志位：启动时文件读取设置，前锋标志位 == false时，看到球小于200cm&&大于6帧则冲，然后前锋标志位设为true。
        */
        // 如果后卫看到小于200cm&&大于5帧则变为前锋
        if(is_forward == false && abs(cur_env_data -> kf_distance) < 150 && football_sustain_frames_nums > 5) {
            is_forward = true;
        }     

        switch (cur_state) {
            case State::Initial:
                //读取配置文件信息
                // readConfig();
                // 状态是初始化，则进入找球状态
                std::cout << "[" << frame << "]初始化结束进入找球状态" << std::endl;
                turn_sum = 0;
                return State::FindFootball;
                break;

            //找球模式要改成每帧都判断，通过frame求余或者计数求余来循环找球，所有状态都要视觉跟球
            case State::FindFootball:
                // if(cur_env_data -> parameters_srv_data.stop_walk_flag == false) {
                //     std::cout << "test" << std::endl;
                //     runWalk(0, default_walk_width, true, false, 0);
                // }
                // 刚刚进入找球模式，初始化找球参数,FIND_FOOTBALL_FRAME_INTERVAL帧后动循环摄像头
                if(pre_state != State::FindFootball) {
                    findFootballParamInit();
                    next_find_football_state_frame = frame + FIND_FOOTBALL_STATE_FRAME_INTERVAL;
                    runWalk(0, default_walk_width, true, false, 0);
                    std::cout << "[" << frame << "]找球状态开始，相关参数已初始化" << std::endl;
                    return State::FindFootball;
                }
                // 如果找球过程中发现了球且大于FOUND_FOOTBALL_TRIGGER帧
                if (cur_env_data -> football_xyxy.size() != 0 && football_sustain_frames_nums >= FOUND_FOOTBALL_TRIGGER) {
                    if(is_forward == false) {// 如果是后卫则不进跟球状态
                        std::cout << "[" << frame << "]后卫连续发现球，但是未到踢球距离，摄像头继续跟着球" << std::endl;
                        return State::FindFootball;
                    }
                    // 头部相关参数归位，停止继续转动摄像头
                    std::cout << "[" << frame << "]连续发现球，退出找球状态，改变状态为跟球" << std::endl;
                    return State::FollowObject;
                }
                // 没有找到球，当前帧是下一个转动摄像头的帧,转动摄像头
                if(frame >= next_find_football_state_frame) {
                    runWalk(0, default_walk_width, true, false, 0);
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

            case State::AdjPosInPlace:
                return State::AdjPosInPlace;
                break;

            case State::FollowObject:
                //如果在跟过程中球丢失
                if(football_lost_frames_nums >= LOST_FOOTBALL_TRIGGER) {
                    confirming_kick = false;
                    std::cout << "[" << frame << "]跟球过程中球丢失,回到找球状态" << std::endl;
                    return State::FindFootball;
                }

                // if(cur_env_data->football_xyxy.size() > 0){
                //     adjHeadFollowFootBall(cur_env_data->football_xyxy);
                // }

                //如果球很远，才要调整身体位置，否则直接踢球NO_ADJUST_BODY_DISTANCE
                if(abs(cur_env_data -> kf_distance) >= 40 && cur_env_data -> distance != 0) {
                    // 踢球和行走中的身体角度判断
                    // 如果摄像头偏了，給2s调整身体
                    if(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle > FOLLOW_FOOTBALL_DEVIATE_TRIGGER_ANGLE || 
                    cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle < -FOLLOW_FOOTBALL_DEVIATE_TRIGGER_ANGLE) {
                        confirming_kick = false;
                        // 摄像头x轴如果是+,说明物体在左，则身体要往左，也就是walk偏转角度要为+
                        if(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle > MAX_WALK_ROTATION_ANGLE) {
                            runWalk(0, default_walk_width, false, false, angleToRadian(MAX_WALK_ROTATION_ANGLE + 5));
                        }else if(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle <= MAX_WALK_ROTATION_ANGLE && cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle > 0){
                            runWalk(0, default_walk_width, false, false, angleToRadian(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle + 5));
                        }else if(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle < -MAX_WALK_ROTATION_ANGLE) {
                            runWalk(0, default_walk_width, false, false, angleToRadian(-MAX_WALK_ROTATION_ANGLE - 5));
                        }else {
                            runWalk(0, default_walk_width, false, false, angleToRadian(cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle - 5));
                        }
                        // 摄像头x轴如果是-,说明物体在右，则身体要往右，也就是walk偏转角度要为-
                        
                        // -----------------------------待调整---------------------------------------------
                        // 摄像头的x轴调整到0
                        // controlHead(0, cur_env_data->robot_head_pos_angle.neck_front_swing_theta_angle, false, false);
                        std::cout << "[" << frame << "]头偏过大，暂停调整身体;distance:" << abs(cur_env_data -> kf_distance) << std::endl;
                        //暂停60帧给下位机移动身体和头部的时间
                        pauseNFrame(20);
                        return State::FollowObject;
                    }
                }


                // //--------------------
                // // save head
                // temp_neck_rotation_theta_angle = cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle;
                // temp_neck_front_swing_theta_angle = cur_env_data->robot_head_pos_angle.neck_front_swing_theta_angle;
                // // 球网方向判断
                // controlHead(0, 0, false, false);
                // ros::Duration(0.25).sleep();
                
                // if(cur_env_data->net_xyxy.size() > 0){
                //     int net_central_point_x = cur_env_data->net_xyxy[0] + (cur_env_data->net_xyxy[2] - cur_env_data->net_xyxy[0]) / 2; // 640
                //     int net_central_point_y = cur_env_data->net_xyxy[1] + (cur_env_data->net_xyxy[3] - cur_env_data->net_xyxy[1]) / 2; // 480
                //     // 只要目标不偏移在二维图像边缘(偏中心)
                //     if(net_sustain_frames_nums>=3 && net_central_point_x<480 && net_central_point_x>160) {
                //         net_flag = true;
                //     }
                // }
                // if(cur_env_data->goal_xyxy.size() > 0){
                //     int goal_central_point_x = cur_env_data->goal_xyxy[0] + (cur_env_data->goal_xyxy[2] - cur_env_data->goal_xyxy[0]) / 2; // 640
                //     int goal_central_point_y = cur_env_data->goal_xyxy[1] + (cur_env_data->goal_xyxy[3] - cur_env_data->goal_xyxy[1]) / 2; // 480
                //     // 只要目标不偏移在二维图像边缘(偏中心)
                //     if(goal_sustain_frames_nums>=3 && goal_central_point_x<480 && goal_central_point_y>160) {
                //         goal_flag = true;
                //     }
                // }
                // if(goal_flag==false && net_flag==false) {
                //     //向左平移
                //     // ------------
                //     // ------------
                //     // ------------
                //     // ------------
                //     // ------------
                //     // "/parallelMove"
                //     // 0 左 1 右
                //     // ------------
                //     // ------------
                //     //给1.5的行动时间
                //     std_msgs::Int16 msg;
                //     msg.data = 0;//左
                //     pub_parallelMove.publish(msg);
                //     ros::Duration(0.5).sleep();
                //     pub_parallelMove.publish(msg);
                //     ros::Duration(0.5).sleep();
                //     pub_parallelMove.publish(msg);
                //     ros::Duration(0.5).sleep();
                //     //向右转圈
                //     runWalk(0, default_walk_width, false, false, angleToRadian(-30));
                //     //将看球的头部还原归位
                //     controlHead(temp_neck_rotation_theta_angle, temp_neck_front_swing_theta_angle, false, false);
                //     ros::Duration(0.25).sleep();
                //     return State::FollowObject;
                // }
                // //--------------------

                // 踢球前的距离判断
                //如果到了踢球的距离,!=0防止没有找到球传空数据的情况
                // 暂停30帧确认球的距离和位置
                if(abs(cur_env_data -> kf_distance) < KICK_FOOTBALL_DISTANCE_TRIGGER && cur_env_data -> distance != 0 && 
                football_sustain_frames_nums >= FOUND_FOOTBALL_TRIGGER) {
                    if(confirming_kick == false) {
                        if(cur_env_data -> parameters_srv_data.stop_walk_flag == false)
                            runWalk(0, default_walk_width, true, false, 0);
                            pauseNFrame(30);
                            confirming_kick = true;
                            std::cout << "[" << frame << "]似乎球距离已经可以踢，暂停30帧确认一下" << std::endl;
                            return State::FollowObject;
                    } else {
                        //如果球门和网在正确范围内，则踢球
                        if(goal_flag == true || net_flag == true) {
                            confirming_kick = false;
                            goal_flag = false;
                            net_flag = false;
                            std::cout << "[" << frame << "]暂停30帧确认后，距离和目标确认无误，进入踢球状态" << std::endl;
                            return State::KickFootball;
                        }else {
                            std::cout << "[" << frame << "]暂停30帧确认后，网和球门位置不对，进入找网状态" << std::endl;
                            return State::FindNet;
                        }
                    }
                }
                //摄像头没偏，也没丢失球，则继续walk跟球，并实时调整摄像头位置
                //（可以加一个参数服务器限制）
                // if(abs(cur_env_data->kf_distance) <= 100) {
                //     runWalk(default_walk_length/2, default_walk_width, false, false, 0);
                // }else {
                runWalk(default_walk_length, default_walk_width, false, false, 0);
                // }
                // adjHeadFollowFootBall(cur_env_data->football_xyxy);
                return State::FollowObject;
                break;

            // 找网模式
            case State::FindNet:
                runWalk(0, default_walk_width, true, false, 0);
                //初次进入找网，需要几帧去更新环境信息
                if(find_net_frame == 0) {
                    std::cout << "[" << frame << "]first become findnet state" << std::endl;
                    // 保存头位置
                    temp_neck_rotation_theta_angle = cur_env_data->robot_head_pos_angle.neck_rotation_theta_angle;
                    temp_neck_front_swing_theta_angle = cur_env_data->robot_head_pos_angle.neck_front_swing_theta_angle;
                    // 球网方向判断（注意环境变量不会更新了）
                    controlHead(0, 0, false, false);
                    find_net_frame++;
                    return State::FindNet;
                    break;
                }

                // net_sustain_frames_nums

                //当前位置找网和门15帧
                if(find_net_frame < 30) {
                    // 开始设置网和球门的信息in these 30 framess
                    // we can find 2 frame is ok in 30 frames
                    if(cur_env_data->net_xyxy.size() > 0 && net_sustain_frames_nums >= 3){
                        int net_central_point_x = cur_env_data->net_xyxy[0] + (cur_env_data->net_xyxy[2] - cur_env_data->net_xyxy[0]) / 2; // 640
                        int net_central_point_y = cur_env_data->net_xyxy[1] + (cur_env_data->net_xyxy[3] - cur_env_data->net_xyxy[1]) / 2; // 480
                        // 只要目标不偏移在二维图像边缘(偏中心)
                        // if(net_sustain_frames_nums>=2 && net_central_point_x<560 && net_central_point_x>80) {
                        if(cur_env_data->net_xyxy[0] < 320 || cur_env_data->net_xyxy[2] > 320) {// 如果门不偏左或者偏右则算找到了门
                            net_flag = true;
                        }
                        // }
                    }
                    if(cur_env_data->goal_xyxy.size() > 0 && goal_sustain_frames_nums >= 3){
                        int goal_central_point_x = cur_env_data->goal_xyxy[0] + (cur_env_data->goal_xyxy[2] - cur_env_data->goal_xyxy[0]) / 2; // 640
                        int goal_central_point_y = cur_env_data->goal_xyxy[1] + (cur_env_data->goal_xyxy[3] - cur_env_data->goal_xyxy[1]) / 2; // 480
                        // 只要目标不偏移在二维图像边缘(偏中心)
                        // if(goal_sustain_frames_nums>=2 && goal_central_point_x<560 && goal_central_point_x>80) {
                        if(cur_env_data->goal_xyxy[0] < 320 || cur_env_data->goal_xyxy[2] > 320) {// 如果门不偏左或者偏右则算找到了门
                            goal_flag = true;
                        }
                        // }
                    }
                    find_net_frame++;
                    return State::FindNet;
                    break;
                }
                //找球门的帧置零
                find_net_frame = 0;

                //如果没找到网和门，持续找
                if(goal_flag==false && net_flag==false) {
                    std::cout << "[" << frame << "]dont find net" << std::endl;
                    // if(cur_env_data->goal_xyxy.size() == 0 && cur_env_data->net_xyxy.size() == 0) {
                    //     runWalk(default_walk_length, default_walk_width, false, false, 0);
                    //     pauseNFrameAndAction(60);
                    //     // ros::Duration(2.5).sleep();
                    //     return State::FindFootball;
                    // }
                    if(turn_sum >= 0) {//如果累计朝向左边，则左平移+右转
                        //向左平移 
                        // "/parallelMove"
                        // 0 左 1 右
                        //给1.5的行动时间
                        std_msgs::Int16 msg;
                        msg.data = 0;//左
                        pub_parallelMove.publish(msg);
                        std::cout << "向左平移三步" << std::endl;
                        ros::Duration(2.5).sleep();
                        //向右转圈
                        runWalk(0, default_walk_width, false, false, angleToRadian(-30));
                        //找球门不改变累计转向，因为要一直往一个方向转，防止在0附近徘徊
                        turn_sum += 30;
                        std::cout << "向右转30度" << std::endl;
                        pauseNFrame(60);
                    }else {// 否则右平移+左转
                        //向右平移 
                        // 0 左 1 右
                        //给1.5的行动时间
                        std_msgs::Int16 msg;
                        msg.data = 1;//右
                        pub_parallelMove.publish(msg);
                        std::cout << "向右平移三步" << std::endl;
                        ros::Duration(2.5).sleep();
                        //向右转圈
                        runWalk(0, default_walk_width, false, false, angleToRadian(30));
                        //找球门不改变累计转向，因为要一直往一个方向转，防止在0附近徘徊
                        turn_sum -= 30;
                        std::cout << "向左转30度" << std::endl;
                        pauseNFrame(60);
                    }
                    return State::FindNet;
                }else {
                    //找到后累计转向归0
                    turn_sum = 0;
                    //将看球的头部还原归位
                    controlHead(temp_neck_rotation_theta_angle, temp_neck_front_swing_theta_angle, false, false);
                    std::cout << "[" << frame << "]found net" << std::endl;
                    pauseNFrame(15);
                    return State::FindFootball;
                }
                break;


                // ------10.08暂时废弃版本------
                // //如果有找到球门或者球网
                // if(goal_flag == true || net_flag == true) {
                //     // 球网或者球门在右
                //     if((cur_env_data->net_xyxy.size() != 0 && cur_env_data->net_xyxy[0] > 320) || (cur_env_data->goal_xyxy.size() != 0 && cur_env_data->goal_xyxy[0] > 320)) {
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
                //     }else if((cur_env_data->net_xyxy.size() != 0 && cur_env_data->net_xyxy[2] <= 320) || (cur_env_data->goal_xyxy.size() != 0 && cur_env_data->goal_xyxy[2] <= 320)){
                //         //向左平移 
                //         // "/parallelMove"
                //         // 0 左 1 右
                //         //给1.5的行动时间
                //         std_msgs::Int16 msg;
                //         msg.data = 1;//右
                //         pub_parallelMove.publish(msg);
                //         std::cout << "向右平移三步" << std::endl;
                //         ros::Duration(2.5).sleep();
                //         //向左转圈
                //         runWalk(0, default_walk_width, false, false, angleToRadian(30));
                //         std::cout << "向左转30度" << std::endl;
                //         pauseNFrame(60);
                //         return State::FindNet;
                //     }else {
                //         //球网和球门在中心了
                //         //将看球的头部还原归位
                //         controlHead(temp_neck_rotation_theta_angle, temp_neck_front_swing_theta_angle, false, false);
                //         std::cout << "[" << frame << "]found net" << std::endl;
                //         pauseNFrame(15);
                //         return State::FindFootball;
                //     }
                // }else {
                //     std::cout << "[" << frame << "]没有找到球网和球门" << std::endl;
                //     return 
                // }



                // --------------9.08----------
                // 舵机找网模式未使用，需改进
                // runWalk(0, default_walk_width, true, false, 0);
                // // 刚刚进入找网模式，初始化找球参数,FIND_FOOTBALL_FRAME_INTERVAL帧后动循环摄像头
                // if(pre_state != State::FindNet) {
                //     findNetParamInit();
                //     next_find_football_state_frame = frame + FIND_FOOTBALL_STATE_FRAME_INTERVAL;
                //     std::cout << "[" << frame << "]找网状态开始，相关参数已初始化" << std::endl;
                //     return State::FindNet;
                // }
                // // 如果找网过程中发现了网且大于FOUND_FOOTBALL_TRIGGER帧
                // if (cur_env_data -> net_xyxy.size() != 0 && net_sustain_frames_nums >= FOUND_FOOTBALL_TRIGGER) {
                //     // 头部相关参数归位，停止继续转动摄像头
                //     std::cout << "[" << frame << "]连续发现网，开始调整位置对准网" << std::endl;

                //     // return State::FollowObject;
                // }
                // // 没有找到网，当前帧是下一个转动摄像头的帧,转动摄像头
                // if(frame == next_find_football_state_frame) {
                //     controlHead(angle_vec[angle_vec_pos][0], angle_vec[angle_vec_pos][1], false, false);
                //     angle_vec_pos++;
                //     // 循环数组，并向右原地转圈
                //     if(angle_vec_pos == angle_vec.size()){
                //         angle_vec_pos = 0;
                //         runWalk(0, default_walk_width, false, false, angleToRadian(-30));
                //     }
                //     next_find_football_state_frame = frame + FIND_FOOTBALL_STATE_FRAME_INTERVAL;
                //     std::cout << "[" << frame << "]找球模式下没有找到球，继续找球" << std::endl;
                //     return State::FindFootball;
                // }
                // return State::FindFootball;
                // break;

            case State::KickFootball:
                // 停止运动
                if(cur_env_data -> parameters_srv_data.stop_walk_flag == false)
                    runWalk(0, default_walk_width, true, false, 0);
                // 调用踢球
                // kickFootball();
                runWalk(default_walk_length, default_walk_width, false, false, 0);
                pauseNFrameAndAction(60);
                return State::FindFootball;
                // std::cout << "[" << frame << "]调用踢球步态，停止运动，进入特殊步态状态" << std::endl;
                // return State::SpecialGait;

            case State::SpecialGait:
                if(!cur_env_data->parameters_srv_data.stop_special_gait_flag) {
                    std::cout << "[" << frame << "]参数服务器还在特殊步态状态，保持特殊步态状态" << std::endl;
                    return State::SpecialGait;
                }else {
                    std::cout << "[" << frame << "]参数服务器结束特殊步态状态，进入跟球状态" << std::endl;
                    return State::FollowObject;
                }
                break;

            default:
                // ROS_INFO("逻辑边界，机器人初始化");
                std::cout << "[" << frame << "]逻辑边界，机器人初始化" << std::endl;
                return State::Initial;
                break;
        }
    }

    // 暂时废弃，使用一个新节点控制头部
    // 运行头部的线程
    // void runFindFootball() {
    //     // 更新状态为Running
    //     // currentState = State::Running;
    //     // 创建线程
    //     thread = std::thread([this]() {
    //         robot_brain_pkg::robot_head_pos head_pos;
    //         float neck_rotation_theta_angle = 0;
    //         float neck_front_swing_theta_angle = 0;
    //         head_pos.neck_rotation_theta = angleToRadian(neck_rotation_theta_angle);
    //         head_pos.neck_front_swing_theta = angleToRadian(neck_front_swing_theta_angle);
    //         std::vector<std::vector<float>> angle_vec = {{0, 0},{-80, 0}, {-80, 45}, {0, 45}, {80, 45}, {80, 90}, {0, 90}, {-80, 90}, {0, 45}};
    //         int angle_vec_pos = 1;
    //         // 线程
    //         while (cur_state == State::FindFootball) {
    //             // 在FindFootball状态下，不断找球
    //             if(angle_vec_pos == angle_vec.size()) {
    //                 angle_vec_pos = 0;
    //             }
    //             pub_head_control.publish(head_pos);
    //             head_pos.neck_rotation_theta = angleToRadian(angle_vec[angle_vec_pos][0]);
    //             head_pos.neck_front_swing_theta = angleToRadian(angle_vec[angle_vec_pos][1]);
    //             std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //         }
    //     });
    // }

    // 暂时废弃，使用一个新节点控制头部
    // 回收找球的线程
    // void stopFindFootball() {
        // // 如果当前状态不是Running，直接返回
        // if (currentState != State::Running) {
        //     return;
        // }

        // 更新状态为Stopped
        // currentState = State::Stopped;

        // 等待线程结束
        // thread.join();
    // }

private:
    // 当前状态
    // std::atomic<State> currentState;
    // 线程对象
    // std::thread thread;

    /* 
        暂时废弃
        根据当前状态和事件获取下一个状态
    */
    // State getNextState(State currentState, Event event) {
    //     switch (currentState) {
    //         case State::Initial:
    //             if (event == Event::Start) {
    //                 runFindFootball();
    //                 return State::FindFootball;
    //             }
    //             break;
    //         case State::FindFootball:
    //             if (event == Event::Walk) {
    //                 stopFindFootball();
    //                 return State::Walk;
    //             } 
    //             // else if (event == Event::Stop) {
    //             //     return State::Stopped;
    //             // }
    //             break;
    //         case State::Walk:
    //             if (event == Event::Kick) {
    //                 kickFootball();
    //                 return State::FindFootball;
    //             } 
    //             // else if (event == Event::Stop) {
    //             //     return State::Stopped;
    //             // }
    //             break;
    //         case State::Stopped:
    //             if (event == Event::Start) {
    //                 return State::Initial;
    //             }
    //             break;
    //         default:
    //             break;
    //     }
    //     return State::Initial;
    // }

    /* 
        暂时废弃
        getEventByEnvState 由环境和状态得到事件
    */
    // Event getEventByEnvState(EnvData *cur_env_data, State cur_state) {
        
    //     return Event::Start;
    // }
    
    /* 
        暂时废弃，直接通过环境和状态得到下一个状态
        处理事件
    */
    // void handleEvent(Event event) {
    //     // 根据当前状态和事件查找下一个状态
    //     State nextState = getNextState(cur_state, event);

    //     // 下一个状态不是当前的状态，更新当前状态
    //     if (nextState != cur_state) {
    //         cur_state = nextState;
    //         // execute();
    //     }
    // }

    // void execute() {
        
    // }
};

#endif