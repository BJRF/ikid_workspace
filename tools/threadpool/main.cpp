#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include "lock.h"
#include "threadpool.h"
#include <signal.h>
#include "http_conn.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/epoll.h>
#include <errno.h>
#include <unistd.h>
#include <exception>

#define MAX_FD 65535  //最大的文件描述符数量
#define MAX_EVENT_NUMBER 10000  //一次可以监听的最大数量

//做信号处理，sig表示对什么信号进行处理，handler要做的操作
void addsig(int sig, void(handler)(int)) {
    struct sigaction sa;
    //清空sa,头文件<string.h>
    memset(&sa, '\0', sizeof(sa));
    //设置临时阻塞的信号集，这里设置全阻塞
    sigfillset(&sa.sa_mask);
    //int sigaction(int signum, const struct sigaction *act, struct sigaction *oldact);
    //signum参数指出要捕获的信号类型，act参数指定新的信号处理方式，oldact参数输出先前信号的处理方式
    sigaction(sig, &sa, NULL);
}

//添加文件描述符到epoll中
extern void addfd(int eopollfd, int fd, bool one_shot);

//从epoll中修改文件描述符
extern void modfd(int epollfd, int fd, int ev);

//从epoll中删除文件描述符
extern void removefd(int epollfd, int fd);

int main(int argc, char* argv[]) {
    if(argc <= 1) {
        printf("输入格式有误");
        exit(-1);  
    }
    //获取端口号
    //atoi将字符串转成整形,在stdlib.h中
    int port = atoi(argv[1]);
    /*
    SIGPIPE信号来自于在读端关闭时，写端继续写时：
    例如情况：假如客户端与服务器通信时客户端关闭连接，服务器可能
    还在写数据给客户端，导致客户端产生SIGPIPE信号，并发送数据
    给服务器，这是服务器已经关闭，导致服务器也产生SIGPIPE，最
    后导致程序崩溃。
    */
    //对SIGPIE信号做处理,处理为SIG_IGN忽略。
    addsig(SIGPIPE, SIG_IGN);
    //创建线程池，初始化线程池
    threadpool<http_conn> *pool = NULL;
    try {
        pool = new threadpool<http_conn>;
    }catch(...) {
        exit(-1);
    }
    //创建一个数组用于保存所有客户端信息
    http_conn* users = new http_conn[MAX_FD];
    /*
    int socket(int domain, int type, int protocol);
    返回一个文件描述符
    domain：通信协议族
    例如这里的AF_INET是ipv4
    type：流套接字
    例如这里的SOCK_STREAM代表使用TCP
    protocol：用于制定某个协议的特定类型，即type类型中的某个类型
    通常某协议中的只有一种特定类型，这样protocol参数能设置成0
    */
    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    /*
    int setsockopt(int sockfd, int level, int optname,const void *optval, socklen_t optlen)
    sockfd：socket监听的文件描述符
    level：1.SOL_SOCKET 通用套接字选项。2.IPPROTO_IP ip层选项。3.IPPROTO_TCP TCP层选项。
    optname:设置的选项
    例如SO_REUSEADDR -- 打开或关闭地址复用功能
    optval：指针，指向存放选项值的缓冲区
    socklen_t:optval缓冲区的长度
    */
    //设置端口复用
    int reuse = 1;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    //结构体sockaddr_in头文件#include <arpa/inet.h>
    /*
    struct sockaddr_in {
    short sin_family;           //Address family：一般来说AF_INET（地址族）PF_INET（协议族）
    unsigned short sin_port;    //Port number：(必须要采用网络数据格式,普通数字可以用htons()函数转换成网络数据格式的数字)
    struct in_addr sin_addr;    //IP address in network byte order：（Internet address）
    unsigned char sin_zero[8];  //Same size as struct sockaddr：没有实际意义,只是为了　跟SOCKADDR结构在内存中对齐
    */
    struct sockaddr_in address;
    //定义协议族
    address.sin_family = AF_INET;
    //定义IP地址
    /*
    INADDR_ANY其实就是0.0.0.0，在这里是表示可以监听来自任何IP的地址。
    */
    address.sin_addr.s_addr = INADDR_ANY;
    /*
    htons是将整型变量从主机字节顺序转变成网络字节顺序,
    就是整数在地址空间存储方式变为高位字节存放在内存的低地址处。
    */
    address.sin_port = htons(port);
    //绑定到本地地址，这里一般强转(struct sockaddr*)&address
    bind(listenfd, (struct sockaddr*)&address, sizeof(address));
    /*
    _N:请求排队的最大长度。当有多个客户端和服务器相连时，
    这个值表示可以使用的处于等待的队列长度
    */
    //监听
    listen(listenfd, 5);

    /*  epoll_event结构体详解
    typedef union epoll_data {
    void *ptr;
    int fd;
    __uint32_t u32;
    __uint64_t u64;
    } epoll_data_t;

    struct epoll_event {
    __uint32_t events; // Epoll events 
    epoll_data_t data; // User data variable 
    };
    */

    //创建epoll对象，事件数组，添加
    epoll_event events[MAX_EVENT_NUMBER];
    int epollfd = epoll_create(1);
    //将监听的文件描述符添加到epoll对象中去
    addfd(epollfd, listenfd, false);
    http_conn::m_epollfd = epollfd;
    while(true) {
        int num = epoll_wait(epollfd, events, MAX_EVENT_NUMBER, -1);
        //errno 需要调用errno.h
        //errno == EINTR是代表是被中断了
        if(num < 0 && errno != EINTR) {
            printf("epoll failure!");
            break;
        }
        //循环遍历事件数组
        for(int i = 0; i < num; i++) {
            int sockfd = events[i].data.fd;
            if(sockfd == listenfd) {
                //有客户端连接进来
                struct sockaddr_in client_address;
                socklen_t client_addrlen = sizeof(client_address);
                int connfd = accept(listenfd, (struct sockaddr*)&client_address, &client_addrlen);
                if(http_conn::m_user_count >= MAX_FD) {
                    //目前连接数满了
                    close(connfd);
                    continue;
                }
                //将新客户的数据初始化，放到数组中
                users[connfd].init(connfd, client_address);
            }else if(events[i].events & (EPOLLRDHUP | EPOLLHUP | EPOLLERR)) {
                //对方异常断开或者错误等事件
                users[sockfd].close_conn();
            }else if(events[i].events & EPOLLIN) {
                if(users[sockfd].read()) {
                    //一次性读完所有数据
                    pool -> append(users + sockfd);
                }
                else {
                    users[sockfd].close_conn();
                }
            }else if(events[i].events & EPOLLOUT) {
                //一次性写完所有数据
                if(!users[sockfd].write()) {
                    users[sockfd].close_conn();
                }
            }
        }
        

    }
    close(epollfd);
    close(listenfd);
    delete[] users;
    delete pool;
    
    return 0;
}