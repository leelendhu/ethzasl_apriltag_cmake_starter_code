#include <string>
#include <vector>
#include <sstream>
#include "apriltags/Tag36h11.h"
#include "apriltags/TagDetector.h"
#include "apriltags/AprilGrid.h"
#include "opencv2/opencv.hpp"
#include "vis.hpp"



int main(int argc, char* argv[]) {
  float grid_size = 0.7;
  float grid_spacing = 0.25;
  int grid_rows  = 6;
  int grid_columns = 8;
  int grid_startID = 0;
  AprilTags::AprilGrid Grid1(grid_rows,grid_columns,grid_size,grid_spacing,grid_startID);
  std::vector<std::vector<cv::Vec2f>> timagepoints1;
  std::vector<std::vector<cv::Vec3f>> tobjectpoints1;
  std::vector<std::vector<cv::Vec2f>> timagepoints2;
  std::vector<std::vector<cv::Vec3f>> tobjectpoints2;
  
  for(int i = 0; i < 29; i++){
      std::stringstream input_image_name1;
      input_image_name1 << "../data/images_trial/img";
      input_image_name1 << std::setw(6) << std::setfill('0')<<i;
      input_image_name1 << "_0";
      input_image_name1 << "_0";
      input_image_name1 << ".png";
      std::string image_name1 = input_image_name1.str();
      std::stringstream input_image_name2;
      input_image_name2 << "../data/images_trial/img";
      input_image_name2 << std::setw(6) << std::setfill('0')<<i;
      input_image_name2 << "_0";
      input_image_name2 << "_1";
      input_image_name2 << ".png";
      std::string image_name2 = input_image_name2.str();
      cv::Mat image1 = cv::imread(image_name1, cv::IMREAD_GRAYSCALE);
      AprilTags::TagDetector tagDetector(AprilTags::tagCodes36h11);
      vector<AprilTags::TagDetection> detections1 = tagDetector.extractTags(image1);
      cv::Mat image2 = cv::imread(image_name2, cv::IMREAD_GRAYSCALE);
      vector<AprilTags::TagDetection> detections2 = tagDetector.extractTags(image2);
      std::vector<cv::Vec2f>  imagepoints1 = Grid1.imgpoints(detections1,Grid1.rows,Grid1.columns,Grid1.start_ID);     
      std::vector<cv::Vec3f>  objectpoints1 = Grid1.objpoints(Grid1.size,Grid1.spacing);
      std::vector<cv::Vec2f>  imagepoints2 = Grid1.imgpoints(detections2,Grid1.rows,Grid1.columns,Grid1.start_ID);     
      std::vector<cv::Vec3f>  objectpoints2 = Grid1.objpoints(Grid1.size,Grid1.spacing);
      int grid_points = Grid1.columns*Grid1.rows*4;
      if(imagepoints1.size() == grid_points && imagepoints2.size() == grid_points){
      timagepoints1.push_back(imagepoints1);
      timagepoints2.push_back(imagepoints2);
      tobjectpoints1.push_back(objectpoints1);
      tobjectpoints2.push_back(objectpoints2);
      cout << "image number"<< i << " accepted" << endl;
      }else {
          cout << "image number" << i << " rejected" << endl;
      }
  }
  cv::destroyAllWindows();
  cv::Mat cameraMatrix1,distCoeffs1,R1,T1;
  cv::Mat cameraMatrix2,distCoeffs2,R2,T2;
  cv::calibrateCamera(tobjectpoints1, timagepoints1, cv::Size(Grid1.rows,Grid1.columns), cameraMatrix1, distCoeffs1, R1, T1);
  cv::calibrateCamera(tobjectpoints1, timagepoints2, cv::Size(Grid1.rows,Grid1.columns), cameraMatrix2, distCoeffs2, R2, T2);

  std::cout << "cameraMatrix of camera 1: " << cameraMatrix1 << std::endl;
  std::cout << "distCoeffs of camera 1 : " << distCoeffs1 << std::endl;
  std::cout << "cameraMatrix of camera 2: " << cameraMatrix2 << std::endl;
  std::cout << "distCoeffs of camera 2 : " << distCoeffs2 << std::endl;
  //std::cout << "Rotation vector : " << R1 << std::endl;
  //std::cout << "Translation vector : " << T1 << std::endl;
return 0;
}
