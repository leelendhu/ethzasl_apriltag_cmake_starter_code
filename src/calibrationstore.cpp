#include <string>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>

#include <sys/stat.h>
#include <iostream>
#include "apriltags/Tag36h11.h"
#include "apriltags/TagDetector.h"
#include "apriltags/AprilGrid.h"
#include "opencv2/opencv.hpp"
#include "vis.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


std::vector<cv::Mat> calibrationstore(const AprilTags::AprilGrid Grid, const std::string imagefoldername, const int cameranumber, const int numberofimagesforcalibration, const int totalnumberofimages) {
  AprilTags::AprilGrid Grid1 = Grid;
  
  float grid_size = Grid1.size;
  float grid_spacing = Grid1.spacing;
  int grid_rows  = Grid1.rows;
  int grid_columns = Grid1.columns;
  int grid_startID = Grid1.start_ID;

  std::vector<std::vector<cv::Vec2f>> timagepoints1;
  std::vector<std::vector<cv::Vec3f>> tobjectpoints1;
  std::vector<std::vector<cv::Vec2f>> timagepoints2;
  std::vector<std::vector<cv::Vec3f>> tobjectpoints2;
  std::string imagefoldername0 = imagefoldername; 
  std::string cameranum = "_0";
  
  if(cameranumber == 0){
      cameranum = "_0";
  }else if(cameranumber == 1){
      cameranum = "_1";
  }else if(cameranumber == 2){
      cameranum = "_2";
  }else {
      std::cout << "wrong camera number, please enter from 0, 1 and 2. the default value is 0 and the results are given with default value" << endl;
  }

  int noimg = numberofimagesforcalibration;
  int totalnoimg = totalnumberofimages;
  int accimg = 0;
  
  for(int i = 0; accimg < noimg; i++){
      std::stringstream input_image_name1 ;
      int ii = rand() % totalnoimg;
      input_image_name1 << imagefoldername0;
      input_image_name1 << "/img";
      input_image_name1 << std::setw(6) << std::setfill('0')<<ii;
      input_image_name1 << cameranum;
      input_image_name1 << "_0";
      input_image_name1 << ".pgm";
      std::string image_name1 = input_image_name1.str();
      std::stringstream input_image_name2;
      input_image_name2 << imagefoldername0;
      input_image_name2 << "/img";
      input_image_name2 << std::setw(6) << std::setfill('0')<<ii;
      input_image_name2 << cameranum;
      input_image_name2 << "_1";
      input_image_name2 << ".pgm";
      std::string image_name2 = input_image_name2.str();
      cv::Mat image1 = cv::imread(image_name1, cv::IMREAD_GRAYSCALE);
      AprilTags::TagDetector tagDetector(AprilTags::tagCodes36h11);
      vector<AprilTags::TagDetection> detections1 = tagDetector.extractTags(image1);
      cv::Mat image2 = cv::imread(image_name2, cv::IMREAD_GRAYSCALE);
      vector<AprilTags::TagDetection> detections2 = tagDetector.extractTags(image2);
      std::vector<cv::Vec2f>  imagepoints1 = Grid1.imgpoints(detections1,Grid1.rows,Grid1.columns,Grid1.start_ID);     
      std::vector<cv::Vec3f>  objectpoints1 = Grid1.objpoints(detections1,Grid1.rows,Grid1.columns,Grid1.start_ID,Grid1.size,Grid1.spacing);
      std::vector<cv::Vec2f>  imagepoints2 = Grid1.imgpoints(detections2,Grid1.rows,Grid1.columns,Grid1.start_ID);     
      std::vector<cv::Vec3f>  objectpoints2 = Grid1.objpoints(detections2,Grid1.rows,Grid1.columns,Grid1.start_ID,Grid1.size,Grid1.spacing);
      int grid_points = Grid1.columns*Grid1.rows*4;
      int threshold = grid_points/2;
      
      if(objectpoints1 == objectpoints2 && objectpoints1.size() > threshold) {
      timagepoints1.push_back(imagepoints1);
      timagepoints2.push_back(imagepoints2);
      tobjectpoints1.push_back(objectpoints1);
      tobjectpoints2.push_back(objectpoints2);
      cout << "image number"<< ii << " accepted" << endl;
      accimg = accimg + 1;
      }else {
          cout << "image number" << ii << " rejected" << endl;
      }
      int th =  totalnoimg - 5;
      if(i > th){
          cout << "not enough good images" << endl;
          break;
      }
  }


  std::cout << "calibrating underway for camera " << cameranumber <<  std::endl;
  cv::destroyAllWindows();
  cv::Mat cameraMatrix1,distCoeffs1,R1,T1,newcameraMatrix1;
  cv::Mat cameraMatrix2,distCoeffs2,R2,T2,newcameraMatrix2;
  int flag1 = 0;
  flag1 |= cv::CALIB_FIX_K3;
  double reprojerror1 = cv::calibrateCamera(tobjectpoints1, timagepoints1, cv::Size(Grid1.rows,Grid1.columns), cameraMatrix1, distCoeffs1, R1, T1,flag1);
  double reprojerror2 = cv::calibrateCamera(tobjectpoints2, timagepoints2, cv::Size(Grid1.rows,Grid1.columns), cameraMatrix2, distCoeffs2, R2, T2,flag1);
  newcameraMatrix1 = cv::getOptimalNewCameraMatrix(cameraMatrix1,distCoeffs1,cv::Size(Grid1.rows,Grid1.columns),1,cv::Size(Grid1.rows,Grid1.columns),0);
  newcameraMatrix2 = cv::getOptimalNewCameraMatrix(cameraMatrix2,distCoeffs2,cv::Size(Grid1.rows,Grid1.columns),1,cv::Size(Grid1.rows,Grid1.columns),0);
  std::cout << "cameraMatrix of sensor 0: " << cameraMatrix1 << std::endl;
  std::cout << "distCoeffs of sensor 0 : " << distCoeffs1 << std::endl;
  std::cout << "cameraMatrix of sensor 1: " << cameraMatrix2 << std::endl;
  std::cout << "distCoeffs of sensor 1 : " << distCoeffs2 << std::endl;
  std::cout << "Reprojection error of sensor 0 : " << reprojerror1 << std::endl;
  std::cout << "Reprojection error of sensor 1 : " << reprojerror2 << std::endl;
  
  
  


  cv::Mat K1,K2,R,F,E,D1,D2;
  K1 = newcameraMatrix1;
  K2 = newcameraMatrix2;
  D1 = distCoeffs1;
  D2 = distCoeffs2;
  cv::Vec3d T;

  cv::FileStorage fsleft("left.txt", cv::FileStorage::WRITE);
  fsleft << "K" << K1;
  fsleft << "D" << D1;
  fsleft << "board_width" << Grid1.rows;
  fsleft << "board_height" << Grid1.columns;
  fsleft << "square_size" << Grid1.size;
  
  cv::FileStorage fsright("right.txt", cv::FileStorage::WRITE);
  fsright << "K" << K2;
  fsright << "D" << D2;
  fsright << "board_width" << Grid1.rows;
  fsright << "board_height" << Grid1.columns;
  fsright << "square_size" << Grid1.size;


  int flag = 0;
  flag |= cv::CALIB_USE_INTRINSIC_GUESS;
  std::cout << "stereo calibration underway for camera " << cameranumber << std::endl;
  cv::stereoCalibrate(tobjectpoints1, timagepoints1, timagepoints2, K1, D1, K2, D2, cv::Size(Grid1.rows,Grid1.columns), R, T, E, F,flag);
  std::cout << "new cameraMatrix of sensor 0: " << K1 << std::endl;
  std::cout << "new cameraMatrix of sensor 1: " << K2 << std::endl;
  std::cout << "translation vector is " << T << std::endl;
  
  cv::FileStorage fs1("stereo.txt", cv::FileStorage::WRITE);
  fs1 << "K1" << K1;
  fs1 << "K2" << K2;
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;


  cv::Mat rect1, rect2, proj_mat1, proj_mat2, Q;
  cv::stereoRectify(newcameraMatrix1,distCoeffs1,newcameraMatrix2,distCoeffs2,
                    cv::Size(Grid1.rows,Grid1.columns),R,T,rect1,rect2,proj_mat1,proj_mat2,Q,
                    1);
   std::cout << "stereo rectification underway for camera " << cameranumber <<  std::endl;
   std::cout << "rectification matrix of sensor 0 is  " << rect1 << std::endl;
   std::cout << "rectification matrix of sensor 1 is  " << rect2 << std::endl;
   std::cout << "projection matrix of sensor 0: " << proj_mat1 << std::endl;
   std::cout << "projection matrix of sensor 1 : " << proj_mat2 << std::endl;

   fs1 << "R1" << rect1;
   fs1 << "R2" << rect2;
   fs1 << "P1" << proj_mat1;
   fs1 << "P2" << proj_mat2;
   fs1 << "Q" << Q;

   std::vector<cv::Mat> calibrationmatrices;
   calibrationmatrices.push_back(K1);
   calibrationmatrices.push_back(distCoeffs1);
   calibrationmatrices.push_back(K2);
   calibrationmatrices.push_back(distCoeffs2);
   calibrationmatrices.push_back(R);
   cv::Mat Translation = cv::Mat(1,3,CV_32F);
   Translation.col(0).setTo(T[0]);
   Translation.col(1).setTo(T[1]);
   Translation.col(2).setTo(T[2]);
   calibrationmatrices.push_back(Translation);
   cout << "inside function T is" << T << endl;
return calibrationmatrices;
}
