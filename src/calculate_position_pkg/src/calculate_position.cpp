#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "calculate_position_pkg/image_points.h"
#include "calculate_position/kalman_filter.h"

using namespace std;
using namespace cv;

// MyKalmanFilter kf;
MyKalmanFilter *kf = new MyKalmanFilter();

void CalculatePnp(const calculate_position_pkg::image_points::ConstPtr& msg) {
	//将接收的消息打印出来
    ROS_INFO("收到坐标: [x1:%d, y1:%d, x2:%d, y2:%d, x3:%d, y3:%d, x4:%d, y4:%d, ]", 
	msg->x1, msg->y1, msg->x2, msg->y2, msg->x3, msg->y3, msg->x4, msg->y4);
	
	vector<Point2d> image_points;

	image_points.push_back(Point2d(msg->x1, msg->y1));
	image_points.push_back(Point2d(msg->x2, msg->y2));
	image_points.push_back(Point2d(msg->x3, msg->y3));
	image_points.push_back(Point2d(msg->x4, msg->y4));

	// 3D 特征点世界坐标，与像素坐标对应，单位是cm
	std::vector<Point3d> model_points;
	model_points.push_back(Point3d(-7.5f, -7.5f, 0)); 
	model_points.push_back(Point3d(+7.5f, -7.5f, 0));
	model_points.push_back(Point3d(-7.5f, +7.5f, 0));
	// 原始点
	// model_points.push_back(Point3d(+7.5, +7.5, 0));
	//中心点
	model_points.push_back(Point3d(0, 0, +7.5f));
	//　注意世界坐标和像素坐标要一一对应

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
		rotation_vector, translation_vector, 0, CV_EPNP);
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
	cout << "P_oc" << endl << P_oc << endl;
	
	//人工滤波
	// if(distance < 0 || distance > 500) return;
		
	float kf_distance = kf -> get_my_kalman_filter_result(distance);
	cout << "distance: " << distance << endl;
	cout << "kf_distance: " << kf_distance << endl;
	{
		//写入未处理的distance
		FILE* fp = NULL;
		char ch[200];
		char filename[] = "/home/hjf/project/ikid_ws/result_data/distance_result/distance_result_100";
		fp = fopen(filename, "a");
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
		char filename[] = "/home/hjf/project/ikid_ws/result_data/distance_result/kf_distance_result_100";
		fp = fopen(filename, "a");
		if(fp == NULL)
		{
			exit(0);
		}
		sprintf(ch, "%lf ", kf_distance);
		fputs(ch, fp);
		fclose(fp);
	}
}

void clearTxt()
{
	FILE* fp;
	char ch[200];
	char filename[] = "/home/hjf/project/ikid_ws/tools/pnp_data";
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

	clearTxt();

    //定义Person对象
    calculate_position_pkg::image_points p;
 
    //初始化ros节点
    ros::init(argc, argv, "calculate_position");
 
    //创建节点句柄
    ros::NodeHandle nh;
    
    //创建Subscribe，订阅名为chatter的话题，注册回调函数chatterCallBack
    ros::Subscriber sub = nh.subscribe("chatter_image_points", 100, CalculatePnp);
 
    //循环等待消息回调
    ros::spin();
	// delete kf;
    return 0;
	// waitKey(0);
}

