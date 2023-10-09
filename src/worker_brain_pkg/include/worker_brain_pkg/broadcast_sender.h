#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <chrono>
#include <vector>

// 定义要传输的数据结构
struct RobotBroadcastData {
    /*
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
    int id;// 机器人id
    bool is_close_distance;// 该机器人是否到靠近的距离
    bool is_forward;// 是否前锋（否则为后卫）
    bool is_live;// 是否存活
};

struct BroadcastData {
    RobotBroadcastData Robot1;
    RobotBroadcastData Robot2;
    RobotBroadcastData Robot3;
    RobotBroadcastData Robot4;
};

class BroadcastSender {
public:
    BroadcastSender() {
        // 创建UDP socket
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd == -1) {
            perror("Error creating socket");
            exit(1);
        }

        // 设置广播选项
        int broadcast = 1;
        if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1) {
            perror("Error setting broadcast option");
            exit(1);
        }

        memset(&clientAddr, 0, sizeof(clientAddr));
        clientAddr.sin_family = AF_INET;
    }

    void sendData(const BroadcastData& data, const char* broadcastAddress, int broadcastPort) {
        clientAddr.sin_addr.s_addr = inet_addr(broadcastAddress);
        clientAddr.sin_port = htons(broadcastPort);

        if (sendto(sockfd, &data, sizeof(data), 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr)) == -1) {
            perror("Error sending broadcast");
            exit(1);
        }
    }

    ~BroadcastSender() {
        close(sockfd);
    }

private:
    int sockfd;
    struct sockaddr_in clientAddr;
};

int main() {
    const char* broadcastAddress = "255.255.255.255";
    int broadcastPort = 12345;
    int receivePort = 12345;

    // 创建传感器数据发送者
    BroadcastSender sender;

    // 准备要发送的传感器数据
    BroadcastData dataToSend;
    dataToSend.temperature = 25.0f;
    dataToSend.humidity = 50.0f;

    // 发送传感器数据广播
    while(1) {
        dataToSend.humidity++;
        sender.sendData(dataToSend, broadcastAddress, broadcastPort);
        std::cout << "send data" << std::endl;
    }


    return 0;
}
