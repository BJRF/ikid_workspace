#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <iostream>
#include <vector>
#include "robot_brain_pkg/calculate_position_result.h"
#include "robot_brain_pkg/cmd_walk.h"
#include "robot_brain_pkg/robot_head_pos.h"

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
    Start,
    Idle,       // 空闲状态
    Running,    // 运行状态
    Paused,     // 暂停状态
    Stopped     // 停止状态
};

// 定义事件枚举
enum class Event {
    Start,      // 开始事件
    Pause,      // 暂停事件
    Resume,     // 恢复事件
    Stop        // 停止事件
};

// 状态机类
class StateMachine {
public:
    EnvData *cur_env_data;
    State cur_state;

    //状态机构造函数
    StateMachine() {
        cur_env_data = new EnvData();
        cur_state = State::Start;
    }

    Event getEventByEnvState(EnvData *cur_env_data, State cur_state) {

        return Event::Pause;
    }
    
    // 处理事件
    void handleEvent(Event event) {
        // 根据当前状态和事件查找下一个状态
        State nextState = getNextState(currentState, event);

        // 如果找到了下一个状态，更新当前状态
        if (nextState != State::Idle) {
            currentState = nextState;
        }
    }

    // 获取当前状态
    State getCurrentState() const {
        return currentState;
    }

    void execute() {

    }

    void updateEnvData(const robot_brain_pkg::calculate_position_result::ConstPtr& position_res, 
    const robot_brain_pkg::cmd_walk& cmd_walk, const robot_brain_pkg::robot_head_pos& robot_head_pos) {
        cur_env_data -> football_xyxy = position_res -> football_xyxy;
        cur_env_data -> goal_xyxy = position_res -> goal_xyxy;
        cur_env_data -> net_xyxy = position_res -> net_xyxy;
        cur_env_data -> net_xyxy = position_res -> net_xyxy;
        cur_env_data -> penalty_mark_xyxy = position_res -> penalty_mark_xyxy;
        cur_env_data -> center_circle_xyxy = position_res -> center_circle_xyxy;
        cur_env_data -> distance = position_res -> distance;
        cur_env_data -> kf_distance = position_res -> kf_distance;
        cur_env_data -> cur_cmd_walk = cmd_walk;
        cur_env_data -> cur_robot_head_pos = robot_head_pos;
    }

private:
    // 当前状态，默认为Idle
    State currentState = State::Idle;

    // 根据当前状态和事件获取下一个状态
    State getNextState(State currentState, Event event) {
        switch (currentState) {
            case State::Idle:
                if (event == Event::Start) {
                    return State::Running;
                }
                break;
            case State::Running:
                if (event == Event::Pause) {
                    return State::Paused;
                } else if (event == Event::Stop) {
                    return State::Stopped;
                }
                break;
            case State::Paused:
                if (event == Event::Resume) {
                    return State::Running;
                } else if (event == Event::Stop) {
                    return State::Stopped;
                }
                break;
            case State::Stopped:
                if (event == Event::Start) {
                    return State::Running;
                }
                break;
            default:
                break;
        }
        return State::Idle;
    }
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