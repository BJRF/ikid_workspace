#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <chrono>
#include <vector>

// 发送数据结构
struct UdpData {
    int football_xyxy[4];
    int goal_xyxy[4];
    int net_xyxy[4];
    int robot_xyxy[4];
    int penalty_mark_xyxy[4];
    int center_circle_xyxy[4];
    // std::vector<int> football_xyxy;
    // std::vector<int> goal_xyxy;
    // std::vector<int> net_xyxy;
    // std::vector<int> robot_xyxy;
    // std::vector<int> penalty_mark_xyxy;
    // std::vector<int> center_circle_xyxy;
    float distance; //原始PNP计算距离
    float kf_distance; //卡尔曼滤波后的计算距离
    float robot_distance; //机器人距离
};

class UDPSender {
public:
    UDPSender(const char* serverIP, int serverPort) : serverIP(serverIP), serverPort(serverPort) {
        // 创建套接字
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd == -1) {
            perror("Failed to create socket");
            exit(1);
        }

        // 设置服务器地址信息
        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(serverPort);
        inet_pton(AF_INET, serverIP, &(serverAddr.sin_addr));
    }

    void SendData(const UdpData& data) {
        // 发送数据
        if (sendto(sockfd, &data, sizeof(data), 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
            perror("Failed to send data");
            exit(1);
        }
    }

    void Close() {
        // 关闭套接字
        close(sockfd);
    }

private:
    const char* serverIP;
    int serverPort;
    int sockfd;
    struct sockaddr_in serverAddr;
};

// int main() {
//     const char* serverIP = "127.0.0.1";
//     int serverPort = 12345;

//     // 在客户端启动发送器
//     UDPSender sender(serverIP, serverPort);
//     int i = 0;
//     while (true) {
//         // 准备要发送的自定义数据
//         MyData dataToSend;
//         dataToSend.id = i++;
//         dataToSend.value = 3.14;

//         // 发送数据到服务器
//         sender.SendData(dataToSend);
//         std::cout << "Sent data: id=" << dataToSend.id << ", value=" << dataToSend.value << std::endl;
//         // std::this_thread::sleep_for(std::chrono::seconds(1));
//     }

//     return 0;
// }
