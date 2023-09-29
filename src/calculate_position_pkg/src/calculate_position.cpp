#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "calculate_position_pkg/image_points.h"
#include "calculate_position_pkg/calculate_position_result.h"
#include "calculate_position/kalman_filter.h"
#include "calculate_position/helper.h"

using namespace std;
using namespace cv;

ros::Publisher pub_cal_pos_res;

//PNP实验IO文件
char path1[] = "/home/hjf/project/ikid_workspace/result_data/distance_result/distance_result_50_200";
char path2[] = "/home/hjf/project/ikid_workspace/result_data/distance_result/kf_distance_result_50_200";

//卡尔曼滤波初值
double pre_distance = 100;
double pre_kf_distance = 100;

// MyKalmanFilter kf;
MyKalmanFilter *kf = new MyKalmanFilter();

//helper UDP相关
const char* serverIP = "192.168.1.247";
int serverPort = 12345;
// 在helper启动UDP发送器
UDPSender sender(serverIP, serverPort);

/* 
	reateBaseCalculatePositionMsg
	根据订阅到的信息构造基础的CalculatePositionMsg
*/
void CreateBaseCalculatePositionMsg(calculate_position_pkg::calculate_position_result &cal_pos_res, const calculate_position_pkg::image_points::ConstPtr& msg) {
	cal_pos_res.football_xyxy = msg->football_xyxy;
	cal_pos_res.goal_xyxy = msg->goal_xyxy;
	cal_pos_res.net_xyxy = msg->net_xyxy;
	cal_pos_res.robot_xyxy = msg->robot_xyxy;
	cal_pos_res.penalty_mark_xyxy = msg->penalty_mark_xyxy;
	cal_pos_res.center_circle_xyxy = msg->center_circle_xyxy;
}

// 构造辅助请求包
void createUdpDate(UdpData& data ,calculate_position_pkg::calculate_position_result cal_pos_res) {
	if(cal_pos_res.football_xyxy.size() == 0) {
		for(int i = 0; i < 4; i++) {
			data.football_xyxy[i] = 0;
		}
	}else {
		for(int i = 0; i < cal_pos_res.football_xyxy.size(); i++) {
			data.football_xyxy[i] = cal_pos_res.football_xyxy[i];
		}
	}

	if(cal_pos_res.goal_xyxy.size() == 0) {
		for(int i = 0; i < 4; i++) {
			data.goal_xyxy[i] = 0;
		}
	}else {
		for(int i = 0; i < cal_pos_res.goal_xyxy.size(); i++) {
			data.goal_xyxy[i] = cal_pos_res.goal_xyxy[i];
		}
	}

	if(cal_pos_res.net_xyxy.size() == 0) {
		for(int i = 0; i < 4; i++) {
			data.net_xyxy[i] = 0;
		}
	}else {
		for(int i = 0; i < cal_pos_res.net_xyxy.size(); i++) {
			data.net_xyxy[i] = cal_pos_res.net_xyxy[i];
		}
	}

	if(cal_pos_res.robot_xyxy.size() == 0) {
		for(int i = 0; i < 4; i++) {
			data.robot_xyxy[i] = 0;
		}
	}else {
		for(int i = 0; i < cal_pos_res.robot_xyxy.size(); i++) {
			data.robot_xyxy[i] = cal_pos_res.robot_xyxy[i];
		}
	}

	if(cal_pos_res.penalty_mark_xyxy.size() == 0) {
		for(int i = 0; i < 4; i++) {
			data.penalty_mark_xyxy[i] = 0;
		}
	}else {
		for(int i = 0; i < cal_pos_res.penalty_mark_xyxy.size(); i++) {
			data.penalty_mark_xyxy[i] = cal_pos_res.penalty_mark_xyxy[i];
		}
	}
	
	if(cal_pos_res.center_circle_xyxy.size() == 0) {
		for(int i = 0; i < 4; i++) {
			data.center_circle_xyxy[i] = 0;
		}
	}else {
		for(int i = 0; i < cal_pos_res.center_circle_xyxy.size(); i++) {
			data.center_circle_xyxy[i] = cal_pos_res.center_circle_xyxy[i];
		}
	}
}

// 求目标距离的函数
float cal_distance(std::vector<Point2d> image_points, std::vector<Point3d> model_points) {
	/*
		构造相机参数
	*/

	// // 相机内参矩阵和畸变系数均由相机标定结果得出
	// // 相机内参矩阵
	// Mat camera_matrix = (Mat_<float>(3, 3) << 1545.8, 0, 0, 0, 1545.5, 0, 975.0269, 579.3380, 1);
	// // 相机畸变系数
	// Mat dist_coeffs = (Mat_<float>(5, 1) << 0.0879, -0.131,0, 0, 0);

	//copy相机参数
	Mat camera_matrix = (Mat_<float>(3, 3) << 659.9293277147924, 0, 145.8791713723572,
    0, 635.3941888799933, 120.2096985290085,
        0, 0, 1);
    // copy相机畸变系数
    Mat dist_coeffs = (Mat_<float>(5, 1) << -0.5885200737681696, 0.6747491058456546, 0.006768694852797847, 
        0.02067272313155804, -0.3616453058722507);

	/*
		PNP算法求解
	*/

	// 旋转向量
	Mat rotation_vector;
	// 平移向量
	Mat translation_vector;
	
	// 线性回归pnp求解
	// solvePnP(model_points, image_points, camera_matrix, dist_coeffs, \
	// 	rotation_vector, translation_vector, 0, CV_ITERATIVE);

	// Epnp求解
	// 默认CV_ITERATIVE方法，可尝试修改为EPNP（CV_EPNP）,P3P（CV_P3P）
	solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector, 0, 1);

	Mat Rvec;
	Mat_<float> Tvec;
	
	rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
	translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式 
	
	Mat_<float> rotMat(3, 3);
	Rodrigues(Rvec, rotMat);
	// 旋转向量转成旋转矩阵
	// cout << "rotMat" << endl << rotMat << endl << endl;

	Mat P_oc;
	P_oc = -rotMat.inv() * Tvec;
	// 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是cm

	// 线性缩放
	// P_oc.at<float>(2,0) *= 48;
	// P_oc.at<float>(2,0) *= 17.3;
	
	/*
		提取距离
	*/

	// Z轴
	float distance = P_oc.at<float>(2,0);
	return distance;
}

/* 
	CalculatePnp
	PNP计算
	目前只计算球
*/
void CalculatePnp(const calculate_position_pkg::image_points::ConstPtr& msg) {
	// 限制
	if(msg -> football_xyxy.size() == 0) {
		// cout << "这帧有其他信息但是没有球" << endl;
		calculate_position_pkg::calculate_position_result cal_pos_res;
		CreateBaseCalculatePositionMsg(cal_pos_res, msg);
		cal_pos_res.distance = pre_distance;
		cal_pos_res.kf_distance = pre_kf_distance;
		pub_cal_pos_res.publish(cal_pos_res);
		return;
	}
	// 将接收的消息打印出来
	// ROS_INFO("收到football坐标: [x1:%d, y1:%d, x2:%d, y2:%d]", 
	// msg->football_xyxy[0], msg->football_xyxy[1], msg->football_xyxy[2], msg->football_xyxy[3]);
	
	// 计算球点
	int football_point1_x = msg->football_xyxy[0];
	int football_point1_y = msg->football_xyxy[1];
	int football_point2_x = msg->football_xyxy[2];
	int football_point2_y = msg->football_xyxy[3];
	int football_point3_x = football_point2_x;
	int football_point3_y = football_point1_y;
	int football_point4_x = football_point1_x + (football_point2_x - football_point1_x) / 2;
	int football_point4_y = football_point1_y + (football_point2_y - football_point1_y) / 2;

	/*
		构造球的2d和3d点
	*/
	// 构造image_point2d
	vector<Point2d> image_points;
	image_points.push_back(Point2d(football_point1_x, football_point1_y));
	image_points.push_back(Point2d(football_point2_x, football_point2_y));
	image_points.push_back(Point2d(football_point3_x, football_point3_y));
	image_points.push_back(Point2d(football_point4_x, football_point4_y));

	// 3D 特征点世界坐标，与像素坐标对应，单位是cm
	// 构造image_point3d
	// 注意世界坐标和像素坐标要一一对应
	std::vector<Point3d> model_points;
	model_points.push_back(Point3d(-7.5f, -7.5f, 0)); 
	model_points.push_back(Point3d(+7.5f, +7.5f, 0));
	model_points.push_back(Point3d(+7.5f, -7.5f, 0));
	// 原始点
	// model_points.push_back(Point3d(+7.5, +7.5, 0));
	//中心点
	model_points.push_back(Point3d(0, 0, +7.5f));

	/*
		构造相机参数
	*/

	// // 相机内参矩阵和畸变系数均由相机标定结果得出
	// // 相机内参矩阵
	// Mat camera_matrix = (Mat_<float>(3, 3) << 1545.8, 0, 0, 0, 1545.5, 0, 975.0269, 579.3380, 1);
	// // 相机畸变系数
	// Mat dist_coeffs = (Mat_<float>(5, 1) << 0.0879, -0.131,0, 0, 0);

	//copy相机参数
	Mat camera_matrix = (Mat_<float>(3, 3) << 659.9293277147924, 0, 145.8791713723572,
    0, 635.3941888799933, 120.2096985290085,
        0, 0, 1);
    // copy相机畸变系数
    Mat dist_coeffs = (Mat_<float>(5, 1) << -0.5885200737681696, 0.6747491058456546, 0.006768694852797847, 
        0.02067272313155804, -0.3616453058722507);

	/*
		PNP算法求解
	*/

	// 旋转向量
	Mat rotation_vector;
	// 平移向量
	Mat translation_vector;
	
	// 线性回归pnp求解
	// solvePnP(model_points, image_points, camera_matrix, dist_coeffs, \
	// 	rotation_vector, translation_vector, 0, CV_ITERATIVE);

	// Epnp求解
	// 默认CV_ITERATIVE方法，可尝试修改为EPNP（CV_EPNP）,P3P（CV_P3P）
	solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector, 0, 1);

	Mat Rvec;
	Mat_<float> Tvec;
	
	rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
	translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式 
	
	Mat_<float> rotMat(3, 3);
	Rodrigues(Rvec, rotMat);
	// 旋转向量转成旋转矩阵
	// cout << "rotMat" << endl << rotMat << endl << endl;

	Mat P_oc;
	P_oc = -rotMat.inv() * Tvec;
	// 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是cm

	// 线性缩放
	// P_oc.at<float>(2,0) *= 48;
	// P_oc.at<float>(2,0) *= 17.3;
	
	/*
		提取距离
	*/

	// Z轴
	float distance = P_oc.at<float>(2,0);
	// cout << "P_oc" << endl << P_oc << endl;
	
	//人工滤波
	// if(distance < 0 || distance > 500) return;
		
	float kf_distance = kf -> get_my_kalman_filter_result(distance);
	cout << "distance: " << distance << endl;
	cout << "kf_distance: " << kf_distance << endl;

	pre_distance = distance;
	pre_kf_distance = kf_distance;


	/* 
		实验结果收集
	*/
	// {
	// 	//写入未处理的distance
	// 	FILE* fp = NULL;
	// 	char ch[200];
	// 	// char filename[] = "/home/hjf/project/ikid_workspace/result_data/distance_result/distance_result_100";
	// 	// fp = fopen(filename, "a");
	// 	fp = fopen(path1, "a");
	// 	if(fp == NULL)
	// 	{
	// 		exit(0);
	// 	}
	// 	sprintf(ch, "%lf ", distance);
	// 	fputs(ch, fp);
	// 	fclose(fp);
	// }

	// {
	// 	//写入卡尔曼滤波处理后的distance
	// 	FILE* fp = NULL;
	// 	char ch[200];
	// 	// char filename[] = "/home/hjf/project/ikid_workspace/result_data/distance_result/kf_distance_result_100";
	// 	// fp = fopen(filename, "a");
	// 	fp = fopen(path2, "a");
	// 	if(fp == NULL)
	// 	{
	// 		exit(0);
	// 	}
	// 	sprintf(ch, "%lf ", kf_distance);
	// 	fputs(ch, fp);
	// 	fclose(fp);
	// }
	// 构造发布msg
	calculate_position_pkg::calculate_position_result cal_pos_res;
	cal_pos_res.distance = distance;
	cal_pos_res.kf_distance = kf_distance;
	CreateBaseCalculatePositionMsg(cal_pos_res, msg);
	// std::cout << cal_pos_res << std::endl;
	pub_cal_pos_res.publish(cal_pos_res);
	
	// 辅助定位相关
	// 构造数据
	// UdpData data;
	// createUdpDate(data, cal_pos_res);
	// // data.football_xyxy = cal_pos_res.football_xyxy;
	// // data.goal_xyxy = cal_pos_res.goal_xyxy;
	// // data.net_xyxy = cal_pos_res.net_xyxy;
	// // data.robot_xyxy = cal_pos_res.robot_xyxy;
	// // data.penalty_mark_xyxy = cal_pos_res.penalty_mark_xyxy;
	// // data.distance = cal_pos_res.distance;
	// // data.kf_distance = cal_pos_res.kf_distance;
	// // data.robot_distance = 0;
	// // udp发送数据给worker
	// sender.SendData(data);
	// std::cout << "Sent data UdpData: UdpData.distance = " << data.distance << std::endl;
}

void cleartxt(char* str)
{
	FILE* fp;
	char ch[200];
	char* filename = str;
	fp = fopen(filename, "w");
	if (fp == NULL)
	{
		exit(0);
	}
	fclose(fp);
	return;
}

int main(int argc, char **argv)
{
	// char path[] = "/home/hjf/project/ikid_ws/tools/pnp_data";
	// cleartxt(path);
	// // char path2[] = "/home/hjf/project/ikid_workspace/result_data/distance_result/distance_result_100";
	// cleartxt(path1);
	// // char path3[] = "/home/hjf/project/ikid_workspace/result_data/distance_result/kf_distance_result_100";
	// cleartxt(path2);

    //定义Person对象
    calculate_position_pkg::image_points p;
 
    //初始化ros节点
    ros::init(argc, argv, "calculate_position");
 
    //创建节点句柄
    ros::NodeHandle nh;

	pub_cal_pos_res = nh.advertise<calculate_position_pkg::calculate_position_result>("chatter_calculate_position",10);
    
    //创建Subscribe，订阅名为chatter的话题，注册回调函数chatterCallBack
    ros::Subscriber sub = nh.subscribe("chatter_image_points", 100, CalculatePnp);
 
    //循环等待消息回调
    ros::spin();
	// delete kf;
    return 0;
	// waitKey(0);
}

