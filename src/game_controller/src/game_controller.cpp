#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cerrno>
#include <fcntl.h>
#include <thread> // 包含多线程支持
#include "ros/ros.h"

#include "RoboCupGameControlData.h"

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

// 不同机器人修改信息
#define team_input 14   // 队伍名 *********************要修改*********
#define player_input 4 // 机器人编号 *********************要修改*********

struct MyStruct
{
    uint8_t GameState;   // 游戏状态
    uint8_t RoboPenalty; // 机器人是否被惩罚
};

void updateParamsToServer(const MyStruct &myStruct);

// 线程函数，用于发送数据
void sendGameData(int returnSock, const sockaddr_in &returnAddr)
{
    const int BUFFER_SIZE = sizeof(RoboCupGameControlReturnData); // 发送缓冲区大小

    while (true)
    {
        RoboCupGameControlReturnData returnData;
        returnData.team = team_input;     //队名
        returnData.player = player_input; //操控机器人编号
        returnData.message = 2;
        // 根据需要填充其他字段

        if (sendto(returnSock, &returnData, sizeof(returnData), 0, (const sockaddr *)&returnAddr, sizeof(returnAddr)) < 0)
        {
            perror("发送数据失败");
        }
        else
        {
            std::cout << "发送数据给裁判盒" << std::endl;
        }

        // 在这里可以加入适当的延迟，以控制发送频率
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 休眠1秒
    }
}

// 处理接收到的数据
void processGameData(const RoboCupGameControlData &gameControlData, MyStruct &myStruct)
{

    // 检查数据包的来源IP地址和端口
    // char *sourceIP = inet_ntoa(recvAddr.sin_addr);
    // int sourcePort = ntohs(recvAddr.sin_port);
    // std::cout << "数据来自的源：" << sourceIP << ":" << sourcePort << std::endl;

    // 打印接收到的 RoboCupGameControlData 结构体的信息
    int teamIndex; //获取自己队伍信息，对方队伍为1
    int teamIndex0 = 0; //获取自己队伍信息，对方队伍为1
    int teamIndex1 = 1; //获取自己队伍信息，对方队伍为1

    const TeamInfo &teamInfo0 = gameControlData.teams[teamIndex0];

    const TeamInfo &teamInfo1 = gameControlData.teams[teamIndex1];

    if(static_cast<int>(teamInfo0.teamNumber) == team_input)
    {
        teamIndex = teamIndex0;
    }

    if(static_cast<int>(teamInfo1.teamNumber) == team_input)
    {
        teamIndex = teamIndex1;
    }
    
    const TeamInfo &teamInfo = gameControlData.teams[teamIndex];

    std::cout << "------------------------------- " << std::endl;
    std::cout << "------------------------------- " << std::endl;
    std::cout << "Packet Number: " << static_cast<int>(gameControlData.packetNumber) << std::endl;
    std::cout << "Game Type: " << static_cast<int>(gameControlData.gameType) << std::endl;
    std::cout << "State: " << static_cast<int>(gameControlData.state) << std::endl;
    std::cout << "First Half: " << static_cast<int>(gameControlData.firstHalf) << std::endl;
    std::cout << "kick Off Team: " << static_cast<int>(gameControlData.kickOffTeam) << std::endl;
    std::cout << "Secondary State: " << static_cast<int>(gameControlData.secondaryState) << std::endl;
    std::cout << "Secondary State Info: " << gameControlData.secondaryStateInfo << std::endl
              << std::endl;

    // int teamIndex = 0; //获取自己队伍信息，对方队伍为1

    // if((static_cast<int>(gameControlData.firstHalf)) == 0)
    // {
    //     teamIndex = 1;
    // }
    // const TeamInfo &teamInfo = gameControlData.teams[teamIndex];
    std::cout << "Team Number: " << static_cast<int>(teamInfo.teamNumber) << std::endl;
    std::cout << "Score: " << static_cast<int>(teamInfo.score) << std::endl;
    std::cout << "Penalty Shot: " << static_cast<int>(teamInfo.penaltyShot) << std::endl;
    std::cout << "Single Shots: " << static_cast<int>(teamInfo.singleShots) << std::endl;

    // for (int RoboIndex = 0; RoboIndex < 4; ++RoboIndex) //用于遍历四个机器人信息

    const RobotInfo &robotInfo = teamInfo.players[player_input - 1]; //只获取player_input输入机器人的信息
    std::cout << "    Robort " << player_input << std::endl;
    std::cout << "        Penalty: " << static_cast<int>(robotInfo.penalty) << std::endl;
    std::cout << "        Seconds Till Unpenalised: " << static_cast<int>(robotInfo.secsTillUnpenalised) << std::endl;
    std::cout << "        Number of Warnings: " << static_cast<int>(robotInfo.numberOfWarnings) << std::endl;
    std::cout << "        Yellow Card Count: " << static_cast<int>(robotInfo.yellowCardCount) << std::endl;
    std::cout << "        Red Card Count: " << static_cast<int>(robotInfo.redCardCount) << std::endl;
    std::cout << "        Goal Keeper: " << (robotInfo.goalKeeper ? "Yes" : "No") << std::endl;

    std::cout << "--------------------- " << std::endl;

    myStruct.GameState = static_cast<int>(gameControlData.state);
    myStruct.RoboPenalty = static_cast<int>(robotInfo.penalty);

    // 将MyStruct发送到参数服务器
    updateParamsToServer(myStruct);
}

// 上传参数到参数服务器
void updateParamsToServer(const MyStruct &myStruct)
{
    ros::param::set("game_controller_gamestate", myStruct.GameState);
    ros::param::set("game_controller_robopenalty", myStruct.RoboPenalty);
}

// 线程函数，用于接收数据
int receiveGameData(int recvSock, RoboCupGameControlData &gameControlData, MyStruct &myStruct, struct sockaddr_in &recvAddr)
{
    const int BUFFER_SIZE = sizeof(RoboCupGameControlData); // 接收缓冲区大小
    int bytesReceived = 0;
    int noDataCount = 0; // 计数器，用于记录连续未收到数据的次数

    while (true)
    {
        socklen_t recvAddr_sz = sizeof(recvAddr);
        fcntl(recvSock, F_SETFL, O_NONBLOCK);
        bytesReceived = recvfrom(recvSock, &gameControlData, BUFFER_SIZE, 0, (struct sockaddr *)&recvAddr, &recvAddr_sz);

        // 在这里可以加入适当的延迟，以控制接收频率
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 休眠10毫秒

        if (bytesReceived > 0)
        {
            // 如果成功接收到数据，则处理数据
            processGameData(gameControlData, myStruct);

            // 重置计数器
            noDataCount = 0;
        }
        else if (bytesReceived == -1)
        {
            // 如果未成功接收到数据，增加计数器
            noDataCount++;

            // 如果连续未收到数据达到一定次数，进行初始化并发送到参数服务器
            if (noDataCount >= 100)
            {
                myStruct.GameState = 0;
                myStruct.RoboPenalty = 0;

                // 将MyStruct发送到参数服务器
                updateParamsToServer(myStruct);

                // 重置计数器
                noDataCount = 0;
            }
        }
    }

    return bytesReceived;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "game_controller");
    ros::NodeHandle nh;

    int returnSock, recvSock;
    struct sockaddr_in returnAddr, recvAddr;

    // 创建 UDP sockets 进行数据发送/接收
    returnSock = socket(AF_INET, SOCK_DGRAM, 0);
    recvSock = socket(AF_INET, SOCK_DGRAM, 0);

    if (returnSock < 0 || recvSock < 0)
    {
        perror("套接字创建失败");
        return 1;
    }
    
    std::string robo_ip;
    // char* robo_ip;
    ros::param::get("/pid_amend/local_ip_addr",robo_ip);

    // 设置发送数据给 game_controller 的地址
    returnAddr.sin_family = AF_INET;
    returnAddr.sin_addr.s_addr = inet_addr(robo_ip.c_str()); // 替换为 机器人 的IP地址
    returnAddr.sin_port = htons(game_controller_RETURN_PORT);

    // 设置接收game_controller返回数据的地址
    recvAddr.sin_family = AF_INET;
    recvAddr.sin_addr.s_addr = htonl(INADDR_ANY); // 监听所有网络接口的数据
    recvAddr.sin_port = htons(game_controller_DATA_PORT);

    // 将接收套接字绑定到指定的端口
    if (bind(recvSock, (struct sockaddr *)&recvAddr, sizeof(recvAddr)) < 0)
    {
        perror("绑定失败");
        return 1;
    }

    // 创建发送数据的线程
    std::thread senderThread(sendGameData, returnSock, returnAddr);

    // 创建接收数据的线程
    RoboCupGameControlData gameControlData; // 用于存储接收到的数据
    MyStruct myStruct;                      // 用于存储机器人状态信息

    //循环接收数据
    while (true)
    {
        receiveGameData(recvSock, gameControlData, myStruct, recvAddr);
    }

    // 等待线程结束
    senderThread.join();

    close(returnSock);
    close(recvSock);

    return 0;
}
