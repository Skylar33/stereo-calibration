#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
//#include "popt_pp.h"
#include <sys/stat.h>

using namespace std;
using namespace cv;

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > image_points;
vector< Point2f > corners;
vector< vector< Point2f > > left_img_points;

Mat img, gray;
Size im_size;

bool doesExist (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

void setup_calibration(int board_width, int board_height, int num_imgs,
    float square_size, string imgs_directory, string imgs_filename,
    string extension) {
    Size board_size = Size(board_width, board_height);

    // 尝试从 0 或 1 开始读取，增加兼容性
    for (int k = 0; k <= num_imgs; k++) {
        string img_file_path = imgs_directory + imgs_filename + to_string(k) + "." + extension;

        if (!doesExist(img_file_path)) {
            // 只有当 k=0 且不存在时才静默跳过，其他编号不存在则提醒
            if (k > 0 && k < 5) cout << "跳过不存在的文件: " << img_file_path << endl;
            continue;
        }

        img = imread(img_file_path, IMREAD_COLOR);
        if (img.empty()) {
            cout << "无法加载图像数据: " << img_file_path << endl;
            continue;
        }

        cv::cvtColor(img, gray, COLOR_BGR2GRAY);

        bool found = cv::findChessboardCorners(gray, board_size, corners,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

        if (found) {
            cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));

            vector< Point3f > obj;
            for (int i = 0; i < board_height; i++)
                for (int j = 0; j < board_width; j++)
                    obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

            image_points.push_back(corners);
            object_points.push_back(obj);
            cout << "图像 " << k << " [成功]: 找到角点" << endl;
        }
        else {
            // 关键调试：如果文件存在但没找到角点，打印提示
            cout << "图像 " << k << " [失败]: 无法识别棋盘格，请检查角点数(w*h)是否设对" << endl;
        }
    }
}

double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
                                 const vector< vector< Point2f > >& imagePoints,
                                 const vector< Mat >& rvecs, const vector< Mat >& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs) {
  vector< Point2f > imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  vector< float > perViewErrors;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i) {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }
  return std::sqrt(totalErr/totalPoints);
}

int main(int argc, char const **argv)
{
  //int board_width, board_height, num_imgs;
  //float square_size;
  //char* imgs_directory;
  //char* imgs_filename;
  //char* out_file;
  //char* extension;

  //static struct poptOption options[] = {
  //  { "board_width",'w',POPT_ARG_INT,&board_width,0,"Checkerboard width","NUM" },
  //  { "board_height",'h',POPT_ARG_INT,&board_height,0,"Checkerboard height","NUM" },
  //  { "num_imgs",'n',POPT_ARG_INT,&num_imgs,0,"Number of checkerboard images","NUM" },
  //  { "square_size",'s',POPT_ARG_FLOAT,&square_size,0,"Size of checkerboard square","NUM" },
  //  { "imgs_directory",'d',POPT_ARG_STRING,&imgs_directory,0,"Directory containing images","STR" },
  //  { "imgs_filename",'i',POPT_ARG_STRING,&imgs_filename,0,"Image filename","STR" },
  //  { "extension",'e',POPT_ARG_STRING,&extension,0,"Image extension","STR" },
  //  { "out_file",'o',POPT_ARG_STRING,&out_file,0,"Output calibration filename (YML)","STR" },
  //  POPT_AUTOHELP
  //  { NULL, 0, 0, NULL, 0, NULL, NULL }
  //};

  //POpt popt(NULL, argc, argv, options, 0);
  //int c;
  //while((c = popt.getNextOpt()) >= 0) {}


    // --- 手动配置参数（去 Popt 化修改） ---
    int board_width = 11;                       // 棋盘格横向角点数
    int board_height = 8;                      // 棋盘格纵向角点数
    int num_imgs = 20;                         // 图像最大索引数
    float square_size = 0.040;                 // 格子大小（单位：米）
    const char* imgs_directory = "E:/1.Calibration/saved_images_0206/saved_images_0206/";  // 图像存放目录
    const char* imgs_filename = "BB";         // 图像文件名前缀 (如 left1.jpg 则填 "left")
    const char* extension = "bmp";             // 扩展名
    const char* out_file = "E:/1.Calibration/out_0206/intrinsic_B.yml";    // 输出文件名
    // ----------------------------1 woxi--------


  setup_calibration(board_width, board_height, num_imgs, square_size,
                   imgs_directory, imgs_filename, extension);

  // --- 核心修改：增加安全检查 ---
  if (image_points.empty()) {
      cout << "错误：未能在目录中找到任何有效的棋盘格角点！" << endl;
      cout << "请检查：1. 图片路径是否真的存在 2. 棋盘格行列数是否设对（应为内角点数）" << endl;
      return -1; // 终止运行，防止下一步 calibrateCamera 崩溃
  }
  printf("Starting Calibration with %d images...\n", (int)image_points.size());

  printf("Starting Calibration\n");
  Mat K;
  Mat D;
  vector< Mat > rvecs, tvecs;
  int flag = 0;
  flag |= CALIB_FIX_K4;
  flag |= CALIB_FIX_K5;

  calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flag);

  cout << "Calibration error: " << computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D) << endl;

  FileStorage fs(out_file, FileStorage::WRITE);
  fs << "K" << K;
  fs << "D" << D;
  fs << "board_width" << board_width;
  fs << "board_height" << board_height;
  fs << "square_size" << square_size;
  printf("Done Calibration\n");

  return 0;
}
