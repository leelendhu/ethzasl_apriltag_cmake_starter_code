#include <string>
#include <vector>
#include <sstream>
#include "apriltags/Tag36h11.h"
#include "apriltags/TagDetector.h"
#include "apriltags/AprilGrid.h"
#include "opencv2/opencv.hpp"
#include "vis.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "calibration.hpp"
#include "calibrationrandom.hpp"
#include "calibrationrandre.hpp"
#include "calibrationstore.hpp"
#include "writeconfig.hpp"
#include "rewriteconfig.hpp"
#include <thread>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <ctime>

int main(int argc, char* argv[]) {
  float grid_size = 0.036;
  float grid_spacing = 0.25;
  int grid_rows  = 6;
  int grid_columns = 8;
  int grid_startID = 0;
  AprilTags::AprilGrid Grid1(grid_rows,grid_columns,grid_size,grid_spacing,grid_startID);
  int numberofimages = 160;
  int imagesforcalibration = 40;
  // change to the name of the folder whose calibration results you will be using for undistorting and rectifying the images
  std::string imagefoldername0= "../data/8x6/40cm";
  int cameranumber0 = 0;
   
  std::vector<cv::Mat> matr0 = calibrationstore(Grid1,imagefoldername0,cameranumber0, imagesforcalibration, numberofimages);
 

  // for undistorting and rectifying the image uncomment below

  cv::Mat R1, R2, P1, P2, Q;
  cv::Mat K1, K2, R;
  cv::Vec3d T;
  cv::Mat D1, D2;

  // change the names of the images to the ones that you will be using for depth sensing
  cv::Mat img1 = cv::imread("../cube0.pgm", cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread("../cube1.pgm", cv::IMREAD_GRAYSCALE);

  cv::FileStorage fs1("stereo.txt", cv::FileStorage::READ);
  fs1["K1"] >> K1;
  fs1["K2"] >> K2;
  fs1["D1"] >> D1;
  fs1["D2"] >> D2;
  fs1["R"] >> R;
  fs1["T"] >> T;

  fs1["R1"] >> R1;
  fs1["R2"] >> R2;
  fs1["P1"] >> P1;
  fs1["P2"] >> P2;
  fs1["Q"] >> Q;

  cv::Mat lmapx, lmapy, rmapx, rmapy;
  cv::Mat imgU1, imgU2;

  cv::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);
  cv::remap(img1, imgU1, lmapx, lmapy, cv::INTER_LINEAR);
  cv::remap(img2, imgU2, rmapx, rmapy, cv::INTER_LINEAR);
  
  // the undistorted and rectified images based on the calibration results from imagefoldername0
  cv::imwrite("../cube0_8x6_70cm.png", imgU1);
  cv::imwrite("../cube1_8x6_70cm.png", imgU2);





return 0;
}
