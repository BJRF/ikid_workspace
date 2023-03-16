#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


using namespace std;
using namespace cv;


void calibrateCameraFromFolder(const string& folder_path, const Size& board_size, const float& square_size) {
    vector<String> file_names;


    // 获取指定文件夹下的所有图片文件
    glob(folder_path + "/*.jpg", file_names, false);
    glob(folder_path + "/*.png", file_names, true);


    vector<vector<Point2f>> image_points;
    vector<vector<Point3f>> object_points;
    vector<Size> image_sizes;


    // 构造标定板上每个角点的坐标
    vector<Point3f> object_corner_points;
    for (int i = 0; i < board_size.height; i++) {
        for (int j = 0; j < board_size.width; j++) {
            object_corner_points.push_back(Point3f(j * square_size, i * square_size, 0.0));
        }
    }


    // 遍历所有图片文件，提取角点坐标
    for (const auto& file_name : file_names) {
        Mat image = imread(file_name, IMREAD_GRAYSCALE);
        if (image.empty()) {
            cerr << "无法读取文件：" << file_name << endl;
            continue;
        }


        // 检测角点
        vector<Point2f> image_corner_points;
        bool found = findChessboardCorners(image, board_size, image_corner_points);
        if (found) {
            // 收集角点坐标和图片大小
            image_points.push_back(image_corner_points);
            object_points.push_back(object_corner_points);
            image_sizes.push_back(image.size());
        }
    }


    // 执行相机标定
    Mat camera_matrix, distortion_coefficients;
    vector<Mat> rvecs, tvecs;
    double rms = calibrateCamera(object_points, image_points, image_sizes, camera_matrix, distortion_coefficients, rvecs, tvecs);


    // 输出标定结果
    cout << "相机内参矩阵：" << endl << camera_matrix << endl;
    cout << "畸变系数：" << endl << distortion_coefficients << endl;
    cout << "标定误差：" << rms << endl;
}


int main(int argc, char* argv[]) {
    if (argc != 2) {
        cerr << "使用方法：" << argv[0] << " 文件夹路径" << endl;
        return 1;
    }


    // 指定标定板的大小和每个格子的实际尺寸
    Size board_size(9, 6);
    float square_size = 2.5;


    // 执行相机标定
    calibrateCameraFromFolder(argv[1], board_size, square_size);


    return 0;
}
