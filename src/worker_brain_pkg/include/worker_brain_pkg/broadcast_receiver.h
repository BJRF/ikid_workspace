#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <chrono>
#include <vector>

// 定义要传输的数据结构
struct BroadcastData {
    float temperature;
    float humidity;
};

class BroadcastReceiver {
public:
    BroadcastReceiver(const char* bindAddress, int bindPort)
        : bindAddress(bindAddress), bindPort(bindPort) {
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

        // 定义要设置的接收缓冲区大小（以字节为单位）
        int bufferSize = sizeof(BroadcastData) * 1; 
        // 设置接收缓冲区大小
        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &bufferSize, sizeof(bufferSize)) == -1) {
            perror("Failed to set receive buffer size");
        }

        // 绑定到指定端口
        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        serverAddr.sin_port = htons(bindPort);
        if (bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
            perror("Error binding to port");
            exit(1);
        }
    }

    void receiveSensorData(BroadcastData& data) {
        socklen_t clientAddrLen = sizeof(clientAddr);
        if (recvfrom(sockfd, &data, sizeof(data), 0, (struct sockaddr *)&clientAddr, &clientAddrLen) == -1) {
            perror("Error receiving broadcast");
            exit(1);
        }
    }

    ~BroadcastReceiver() {
        close(sockfd);
    }

private:
    int sockfd;
    const char* bindAddress;
    int bindPort;
    struct sockaddr_in serverAddr, clientAddr;
};

int main() {
    const char* broadcastAddress = "255.255.255.255";
    int broadcastPort = 12345;
    int receivePort = 12345;


    // 创建传感器数据接收者
    BroadcastReceiver receiver("0.0.0.0", receivePort);

    // 接收传感器数据广播
    BroadcastData receivedData;
    while(1) {
        receiver.receiveSensorData(receivedData);
        std::cout << "Received sensor data - Temperature: " << receivedData.temperature
              << " Humidity: " << receivedData.humidity << std::endl;
              std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}

