#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <iostream>
#include <vector>
#include <thread>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "robot_brain_pkg/calculate_position_result.h"
#include "robot_brain_pkg/cmd_walk.h"
#include "robot_brain_pkg/robot_head_pos.h"
#include "robot_brain_pkg/head_contol_by_brain.h"

#define PI acos(-1)

// 全局变量
// ros 发布
ros::Publisher pub_walk;
ros::Publisher pub_spcial;
ros::Publisher pub_head_control;

// 文件输入输出流
std::fstream fout;
std::fstream fin;

const char head_pos_file_path[] = "/home/hjf/project/ikid_workspace/src/robot_brain_pkg/data/head_pos_angle.txt";

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

    // 球的连续帧
    int football_sustain_frames_nums = 0;
    // 参数服务器数据
    ParametersSrvData parameters_srv_data;
    EnvData() {
        
    }
    EnvData(const robot_brain_pkg::calculate_position_result::ConstPtr& position_res, 
    const robot_brain_pkg::cmd_walk& cmd_walk, const RobotHeadPosAngle robo_head_pos_angle) {
        football_xyxy = position_res -> football_xyxy;
        goal_xyxy = position_res -> goal_xyxy;
        net_xyxy = position_res -> net_xyxy;
        net_xyxy = position_res -> net_xyxy;
        penalty_mark_xyxy = position_res -> penalty_mark_xyxy;
        center_circle_xyxy = position_res -> center_circle_xyxy;
        distance = position_res -> distance;
        kf_distance = position_res -> kf_distance;
        this -> cur_cmd_walk = cmd_walk;
        this -> robot_head_pos_angle = robo_head_pos_angle;
    }
};

// 定义状态枚举
enum class State {
    Initial,        // 开始状态
    FindFootball,   // 找球状态
    Walk,           // 行走状态(直走)
    KickFootball,   // 踢球状态
    FindGoal,       // 找门状态
    FollowObject,   // 跟随状态(视觉)
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

    //状态机构造函数
    StateMachine() {
        cur_env_data = new EnvData();
        cur_state = State::Initial;
    }
    
    // getNextStateByEnvCurState 由环境和当前状态得到下一个状态，并执行下一个状态
    State getNextStateByEnvCurState() {
        // 参数服务器环境能动才判断状态
        if(cur_env_data -> parameters_srv_data.stop_special_gait_flag || cur_env_data -> parameters_srv_data.stop_turn_flag || 
        cur_env_data -> parameters_srv_data.stop_walk_flag || cur_env_data -> parameters_srv_data.walk_with_ball) {
            ROS_INFO("参数服务器不允许动");
            return State::Paused;
        }
        switch (cur_state) {
            case State::Initial:
                // 状态是初始化，则进入找球状态
                controlHead(0, 0, true, false);
                return State::FindFootball;
                break;
            case State::FindFootball:
                // 相机里没有发现球,继续找
                if(cur_env_data -> football_xyxy.size() == 0){
                    controlHead(0, 0, true, false);
                    return State::FindFootball;
                }
                // 如果相机里发现了球且连续超过3帧
                else if (cur_env_data -> football_xyxy.size() != 0 && cur_env_data -> football_sustain_frames_nums >= 3) {
                    // 距离小于50,进入踢球状态
                    if(abs(cur_env_data -> kf_distance) < 50) {
                        return State::KickFootball;
                    }
                    // 大于50则跟球
                    else {
                        return State::FollowObject;
                    }
                }
                // 继续找球 
                else {
                    controlHead(0, 0, true, false);
                    return State::FindFootball;
                }
                break;
            case State::KickFootball:
                if(pre_state == State::KickFootball) {
                    // 查参数服务器，直到能动才继续行动(查看环境信息的参数服务器信息)
                    if(!cur_env_data -> parameters_srv_data.stop_special_gait_flag) {
                        return State::FollowObject;
                    }
                    return State::KickFootball;
                }else {
                    kickFootball();
                    // 停止运动
                    runWalk(0, 0, true, false, 0);
                    return State::KickFootball;
                }
            case State::FollowObject:
                runWalk(0.08, 0.07*2, false, false, 0);
                return State::FollowObject;
                break;
            case State::Paused:
                break;
            default:
                break;
        }
        return State::Initial;
    }

    // 获取当前状态
    State getCurrentState() const {
        return cur_state;
    }

    // 读取头部位置的信息
    RobotHeadPosAngle readHeadPos() {
        RobotHeadPosAngle temp_robot_head_pos_angle;
        fin.open("/home/wp/ikid_ws/imubuffer.txt", std::ios::in);
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
        ros::param::get("stop_special_gait_flag",temp_data.stop_special_gait_flag);
        ros::param::get("stop_walk_flag",temp_data.stop_walk_flag);
        ros::param::get("stop_turn_flag",temp_data.stop_turn_flag);
        ros::param::get("walk_with_ball",temp_data.walk_with_ball);
        return temp_data;
    }

    // 更新环境信息(walk没有更新)
    void updateEnvData(const robot_brain_pkg::calculate_position_result::ConstPtr& position_res) {
        // 更新球的连续帧数
        if(cur_env_data->football_xyxy.size() > 0){
            cur_env_data -> football_sustain_frames_nums++;
        }else {
            cur_env_data -> football_sustain_frames_nums = 0;
        }
        // 读取参数服务器数据
        cur_env_data->parameters_srv_data = readParametersSrvData();
        // 目标检测+pnp的环境数据
        cur_env_data -> football_xyxy = position_res -> football_xyxy;
        cur_env_data -> goal_xyxy = position_res -> goal_xyxy;
        cur_env_data -> net_xyxy = position_res -> net_xyxy;
        cur_env_data -> net_xyxy = position_res -> net_xyxy;
        cur_env_data -> penalty_mark_xyxy = position_res -> penalty_mark_xyxy;
        cur_env_data -> center_circle_xyxy = position_res -> center_circle_xyxy;
        cur_env_data -> distance = position_res -> distance;
        cur_env_data -> kf_distance = position_res -> kf_distance;
        // 更新头部舵机信息
        cur_env_data->robot_head_pos_angle = readHeadPos();
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
    void controlHead(double neck_rotation_theta, double neck_front_swing_theta, bool is_find_state, bool is_follow_state) {
        robot_brain_pkg::head_contol_by_brain head_contol_by_brain;
        head_contol_by_brain.neck_rotation_theta = neck_rotation_theta;
        head_contol_by_brain.neck_front_swing_theta = neck_front_swing_theta;
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
        std::cout << "pusblish walk:" << walk << std::endl;
        pub_walk.publish(walk);
    }

    //调用踢球步态
    void kickFootball() {
        std_msgs::Int16 msg;
        msg.data = 3;
        std::cout << "pusblish kick:" << msg << std::endl;
        pub_spcial.publish(msg);
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