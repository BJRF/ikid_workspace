#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <chrono>
#include <vector>

// 接收数据结构
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
    double neck_rotation_theta_angle;// 颈部旋转关节
    int state;//0-data,1-run,2-kick
    double var_theta;//if run
    int kick_leg;//if kick
};

// UdpData data;

class UDPReceiver {
public:
    UDPReceiver(int port) : port(port) {
        // 创建套接字
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd == -1) {
            perror("Failed to create socket");
            exit(1);
        }

        // 定义要设置的接收缓冲区大小（以字节为单位）
        int bufferSize = sizeof(UdpData) * 5; 
        // 设置接收缓冲区大小
        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &bufferSize, sizeof(bufferSize)) == -1) {
            perror("Failed to set receive buffer size");
        }

        // 设置本地地址信息
        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(port);
        serverAddr.sin_addr.s_addr = INADDR_ANY;

        // 绑定套接字到本地地址
        if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
            perror("Failed to bind socket");
            exit(1);
        }
    }

    void ReceiveData(UdpData &data) {
        // UdpData data;
        // 接收数据
        socklen_t addrLen = sizeof(clientAddr);
        // std::cout << sizeof(UdpData) << " " << sizeof(data) << std::endl;
        if (recvfrom(sockfd, &data, sizeof(data), 0, (struct sockaddr*)&clientAddr, &addrLen) == -1) {
            perror("Failed to receive data");
            exit(1);
        }
        std::cout << data.distance << std::endl;
        // return data;
    }

    void Close() {
        // 关闭套接字
        close(sockfd);
    }

private:
    int port;
    int sockfd;
    struct sockaddr_in serverAddr;
    struct sockaddr_in clientAddr;
};

// int main() {
//     int serverPort = 12345;

//     // 在服务器端启动接收器
//     UDPReceiver receiver(serverPort);

//     while (true) {
//         // 接收来自客户端的数据
//         MyData receivedData = receiver.ReceiveData();
//         std::cout << "Received data: id=" << receivedData.id << ", value=" << receivedData.value << std::endl;
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//     }

//     return 0;
// }
