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

char path1[] = "/home/hjf/project/ikid_workspace/result_data/distance_result/distance_result_50_900";
char path2[] = "/home/hjf/project/ikid_workspace/result_data/distance_result/kf_distance_result_50_900";

double pre_distance = 100;
double pre_kf_distance = 100;

// MyKalmanFilter kf;
MyKalmanFilter *kf = new MyKalmanFilter();

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
	//将接收的消息打印出来
    // ROS_INFO("收到坐标: [x1:%d, y1:%d, x2:%d, y2:%d, x3:%d, y3:%d, x4:%d, y4:%d, ]", 
	// msg->x1, msg->y1, msg->x2, msg->y2, msg->x3, msg->y3, msg->x4, msg->y4);
	ROS_INFO("收到football坐标: [x1:%d, y1:%d, x2:%d, y2:%d]", 
	msg->football_xyxy[0], msg->football_xyxy[1], msg->football_xyxy[2], msg->football_xyxy[3]);
	
	// 计算球点
	int football_point1_x = msg->football_xyxy[0];
	int football_point1_y = msg->football_xyxy[1];
	int football_point2_x = msg->football_xyxy[2];
	int football_point2_y = msg->football_xyxy[3];
	int football_point3_x = football_point2_x;
	int football_point3_y = football_point1_y;
	int football_point4_x = football_point1_x + (football_point2_x - football_point1_x) / 2;
	int football_point4_y = football_point1_y + (football_point2_y - football_point1_y) / 2;
	int football_point5_x = msg->football_xyxy[0] + (msg->football_xyxy[2]-msg->football_xyxy[0])/4;
	int football_point5_y = football_point4_y;
	int football_point6_x = football_point4_x;
	int football_point6_y = football_point1_y + (football_point2_y - football_point1_y) *7/8;
	int football_point7_x = football_point5_x;
	int football_point7_y = football_point1_y - (football_point2_y - football_point1_y) / 4;
	int football_point8_x = football_point4_x;
	int football_point8_y = football_point7_y;
	int football_point9_x = football_point2_x / 2;
	int football_point9_y = football_point1_y / 2;
	int football_point10_x= football_point2_x / 2;
	int football_point10_y= football_point4_y;
	int football_point11_x= football_point2_x / 2;
	int football_point11_y= football_point2_y - (football_point2_y - football_point1_y) / 4;
	int football_point12_x= football_point4_x;
	int football_point12_y= football_point2_y / 2;
	int football_point13_x= football_point2_x / 2;
	int football_point13_y= football_point11_y;
	int football_point14_x = football_point1_x;
	int football_point14_y = football_point2_y;
	// cout << "x1:" << football_point1_x << " y1:" << football_point1_y << endl;
	// cout << "x2:" << football_point2_x << " y2:" << football_point2_y << endl;
	// cout << "x3:" << football_point3_x << " y3:" << football_point3_y << endl;
	// cout << "x4:" << football_point4_x << " y4:" << football_point4_y << endl;

	// 构造image_point2d
	vector<Point2d> image_points;
	image_points.push_back(Point2d(football_point1_x, football_point1_y));
	image_points.push_back(Point2d(football_point2_x, football_point2_y));
	image_points.push_back(Point2d(football_point3_x, football_point3_y));
	image_points.push_back(Point2d(football_point4_x, football_point4_y));
	image_points.push_back(Point2d(football_point5_x, football_point5_y));
	image_points.push_back(Point2d(football_point6_x, football_point6_y));
	// image_points.push_back(Point2d(football_point7_x, football_point7_y));
	// image_points.push_back(Point2d(football_point8_x, football_point8_y));
	// image_points.push_back(Point2d(football_point9_x, football_point9_y));
	// image_points.push_back(Point2d(football_point10_x, football_point10_y));
	// image_points.push_back(Point2d(football_point11_x, football_point11_y));
	// image_points.push_back(Point2d(football_point12_x, football_point12_y));
	// image_points.push_back(Point2d(football_point13_x, football_point13_y));
	
	// 原始点
	// model_points.push_back(Point3d(+7.5, +7.5, 0));

	// 3D 特征点世界坐标，与像素坐标对应，单位是cm
	// 构造image_point3d
	// 注意世界坐标和像素坐标要一一对应
	std::vector<Point3d> model_points;
	model_points.push_back(Point3d(-6.95f, +6.95f, 0)); 
	model_points.push_back(Point3d(+6.95f, -6.95f, 0));
	model_points.push_back(Point3d(+6.95f, +6.95f, 0));
	//中心点
	model_points.push_back(Point3d(0, 0, +6.95f));

	
	model_points.push_back(Point3d(-3.4750f, 0, +6.018877f));
	model_points.push_back(Point3d(0, +5.2125f, +4.596993f));
	// model_points.push_back(Point3d(-3.475f, +3.475f, +4.914392f));
	// model_points.push_back(Point3d(0, +3.475f, +6.018877f));
	// model_points.push_back(Point3d(+3.475f, +3.475f, +4.914392f));
	// model_points.push_back(Point3d(+3.475f, 0, +6.018877f));
	// model_points.push_back(Point3d(+3.475f, -3.475f, +4.914392f));
	// model_points.push_back(Point3d(0, -3.475f, +4.914392f));
	// model_points.push_back(Point3d(-3.475f, -3.475f, +4.914392f));
	// model_points.push_back(Point3d(-6.95f, -6.95f, 0));


	// // 相机内参矩阵和畸变系数均由相机标定结果得出
	// // 相机内参矩阵
	// Mat camera_matrix = (Mat_<float>(3, 3) << 1545.8, 0, 0, 0, 1545.5, 0, 975.0269, 579.3380, 1);
	// // 相机畸变系数
	// Mat dist_coeffs = (Mat_<float>(5, 1) << 0.0879, -0.131,0, 0, 0);

	// //copy相机参数
	// Mat camera_matrix = (Mat_<float>(3, 3) << 689.31882651, 0, 299.73752887,
    // 0, 690.73014404 , 215.66889367,
    //     0, 0, 1);
	Mat camera_matrix = (Mat_<float>(3, 3) << 689.318, 0, 299.737,
    0, 690.730 , 215.668,
        0, 0, 1);
    // // copy相机畸变系数
    Mat dist_coeffs = (Mat_<float>(5, 1) << 0, 0, 0, 
        0, 0);
    
	// // copy相机参数
	// Mat camera_matrix = (Mat_<float>(3, 3) << 685.8779 , 0 ,306,
    // 0 , 687.9383 , 218,
	// 0 , 0 , 1);
    // // copy相机畸变系数
	// Mat dist_coeffs = (Mat_<float>(5, 1) << 0.101833374721345 , -0.116193491164472 , -0.454393331554385,
	// 0.001136924435672 , -0.004731239124819);
	
	
	
	
	// cout << "Camera Matrix " << endl << camera_matrix << endl << endl;
	// 旋转向量
	Mat rotation_vector;
	// 平移向量
	Mat translation_vector;
	
	// 线性回归pnp求解
	// solvePnP(model_points, image_points, camera_matrix, dist_coeffs, \
	// 	rotation_vector, translation_vector, 0, CV_ITERATIVE);

	// Epnp求解
	solvePnP(model_points, image_points, camera_matrix, dist_coeffs, \
		rotation_vector, translation_vector, 0, SOLVEPNP_UPNP);
	// SOLVEPNP_EPNP 60 60+-
	// SOLVEPNP_P3P 60 55+-
	// SOLVEPNP_ITERATIVE 60 0 <x <10
	// SOLVEPNP_DLS 60 10+-
	// SOLVEPNP_UPNP 60 40 < x < 70
	// 默认CV_ITERATIVE方法，可尝试修改为EPNP（CV_EPNP）,P3P（CV_P3P）

	// cout << "Rotation Vector " << endl << rotation_vector << endl << endl;
	// cout << "Translation Vector" << endl << translation_vector << endl << endl;

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
	
	// Z轴
	float distance = P_oc.at<float>(2,0);
	// cout << "P_oc" << endl << P_oc << endl;
	
	//人工滤波
	// if(distance < 0 || distance > 500) return;
		
	float kf_distance = kf -> get_my_kalman_filter_result(distance);
	if(distance > 1000 || distance < -1000) return;
	cout << "distance: " << distance << endl;
	cout << "kf_distance: " << kf_distance << endl;

	pre_distance = distance;
	pre_kf_distance = kf_distance;

	{
		//写入未处理的distance
		FILE* fp = NULL;
		char ch[200];
		// char filename[] = "/home/hjf/project/ikid_workspace/result_data/distance_result/distance_result_100";
		// fp = fopen(filename, "a");
		fp = fopen(path1, "a");
		if(fp == NULL)
		{
			exit(0);
		}
		sprintf(ch, "%lf ", distance);
		fputs(ch, fp);
		fclose(fp);
	}

	{
		//写入卡尔曼滤波处理后的distance
		FILE* fp = NULL;
		char ch[200];
		// char filename[] = "/home/hjf/project/ikid_workspace/result_data/distance_result/kf_distance_result_100";
		// fp = fopen(filename, "a");
		fp = fopen(path2, "a");
		if(fp == NULL)
		{
			exit(0);
		}
		sprintf(ch, "%lf ", kf_distance);
		fputs(ch, fp);
		fclose(fp);
	}
	// 构造发布msg
	calculate_position_pkg::calculate_position_result cal_pos_res;
	cal_pos_res.distance = distance;
	cal_pos_res.kf_distance = kf_distance;
	CreateBaseCalculatePositionMsg(cal_pos_res, msg);
	// std::cout << cal_pos_res << std::endl;
	pub_cal_pos_res.publish(cal_pos_res);

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

	cleartxt(path1);
	cleartxt(path2);

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
