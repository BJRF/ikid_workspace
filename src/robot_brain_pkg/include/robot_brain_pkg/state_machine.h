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

//定义当前环境数据
struct EnvData
    {
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
        robot_brain_pkg::robot_head_pos cur_robot_head_pos;
        EnvData() {
            
        }
        EnvData(const robot_brain_pkg::calculate_position_result::ConstPtr& position_res, 
        const robot_brain_pkg::cmd_walk& cmd_walk, const robot_brain_pkg::robot_head_pos& robot_head_pos) {
            football_xyxy = position_res -> football_xyxy;
            goal_xyxy = position_res -> goal_xyxy;
            net_xyxy = position_res -> net_xyxy;
            net_xyxy = position_res -> net_xyxy;
            penalty_mark_xyxy = position_res -> penalty_mark_xyxy;
            center_circle_xyxy = position_res -> center_circle_xyxy;
            distance = position_res -> distance;
            kf_distance = position_res -> kf_distance;
            this -> cur_cmd_walk = cmd_walk;
            this -> cur_robot_head_pos = robot_head_pos;
        }
    };

// 定义状态枚举
enum class State {
    Initial,        // 开始状态
    FindFootball,   // 找球状态
    Walk,           // 行走状态(直走)
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
    EnvData *cur_env_data;
    State cur_state;

    //状态机构造函数
    StateMachine() {
        cur_env_data = new EnvData();
        cur_state = State::Initial;
    }
    
    // getNextStateByEnvCurState 由环境和当前状态得到下一个状态，并执行下一个状态
    State getNextStateByEnvCurState(State state, EnvData env) {
        std::cout << "state" << int(state) << std::endl;
        // controlHead(20, 20, true, false);
        switch (state) {
            case State::Initial:
                controlHead(20, 20, true, false);
                // cur_state = State::FindFootball;
                return State::FindFootball;
                break;
            case State::FindFootball:
                if (env.football_xyxy.size() != 0) {
                    if(env.kf_distance > -50) {
                        std::cout << "env.kf_distance" << env.kf_distance;
                        // kickFootball();
                    }
                    // stopFindFootball();
                    runWalk(0.08, 0.07*2, false, false, 0);
                    return State::Walk;
                } else {
                    return State::FindFootball;
                }
                // else if (event == Event::Stop) {
                //     return State::Stopped;
                // }
                break;
            case State::Walk:
                if (env.football_xyxy.size() != 0) {
                    if(env.kf_distance > -50) {
                        std::cout << "env.kf_distance" << env.kf_distance;
                        // kickFootball();
                    }
                }
                return State::Walk;
                // if (event == Event::Kick) {
                //     kickFootball();
                //     return State::FindFootball;
                // } 
                // else if (event == Event::Stop) {
                //     return State::Stopped;
                // }
                break;
            case State::Stopped:
                // if (event == Event::Start) {
                //     return State::Initial;
                // }
                break;
            default:
                break;
        }
        return State::Walk;
    }

    // 获取当前状态
    State getCurrentState() const {
        return cur_state;
    }

    void updateEnvData(const robot_brain_pkg::calculate_position_result::ConstPtr& position_res) {
        cur_env_data -> football_xyxy = position_res -> football_xyxy;
        cur_env_data -> goal_xyxy = position_res -> goal_xyxy;
        cur_env_data -> net_xyxy = position_res -> net_xyxy;
        cur_env_data -> net_xyxy = position_res -> net_xyxy;
        cur_env_data -> penalty_mark_xyxy = position_res -> penalty_mark_xyxy;
        cur_env_data -> center_circle_xyxy = position_res -> center_circle_xyxy;
        cur_env_data -> distance = position_res -> distance;
        cur_env_data -> kf_distance = position_res -> kf_distance;
    }

    // 角度转弧度
    double angleToRadian(double angle) {
        double radian = angle / 180 * PI;
        return radian;
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
    void stopFindFootball() {
        // // 如果当前状态不是Running，直接返回
        // if (currentState != State::Running) {
        //     return;
        // }

        // 更新状态为Stopped
        // currentState = State::Stopped;

        // 等待线程结束
        thread.join();
    }

    // 控制头部
    void controlHead(double neck_rotation_theta, double neck_front_swing_theta, bool is_find_state, bool is_follow_state) {
        robot_brain_pkg::head_contol_by_brain head_contol_by_brain;
        head_contol_by_brain.neck_rotation_theta = neck_rotation_theta;
        head_contol_by_brain.neck_front_swing_theta = neck_front_swing_theta;
        head_contol_by_brain.is_find_state = is_find_state;
        head_contol_by_brain.is_follow_state = is_follow_state;
        std::cout << "pusblish head_contol_by_brain:" << head_contol_by_brain << std::endl;
        std::cout << "111111111111111111111111111111:" << head_contol_by_brain << std::endl;
        pub_head_control.publish(head_contol_by_brain);
    }

    // 行走
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

private:
    // 当前状态，默认为Initial
    // State currentState = State::Initial;

    // 当前状态
    // std::atomic<State> currentState;
    // 线程对象
    std::thread thread;

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
    Event getEventByEnvState(EnvData *cur_env_data, State cur_state) {
        
        return Event::Start;
    }
    
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

// int main() {
//     StateMachine fsm;

//     // 处理开始事件
//     fsm.handleEvent(Event::Start);
//     std::cout << "当前状态: " << static_cast<int>(fsm.getCurrentState()) << std::endl; // 输出: 当前状态: 1

//     // 处理暂停事件
//     fsm.handleEvent(Event::Pause);
//     std::cout << "当前状态: " << static_cast<int>(fsm.getCurrentState()) << std::endl; // 输出: 当前状态: 2

//     // 处理恢复事件
//     fsm.handleEvent(Event::Resume);
//     std::cout << "当前状态: " << static_cast<int>(fsm.getCurrentState()) << std::endl; // 输出: 当前状态: 1

//     // 处理停止事件
//     fsm.handleEvent(Event::Stop);
//     std::cout << "当前状态: " << static_cast<int>(fsm.getCurrentState()) << std::endl; // 输出: 当前状态: 3

//     return 0;
// }

#endif