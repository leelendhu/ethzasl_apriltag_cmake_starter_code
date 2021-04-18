#include <string>
#include <vector>

#include "apriltags/Tag36h11.h"
#include "apriltags/TagDetector.h"
#include "apriltags/AprilGrid.h"
#include "opencv2/opencv.hpp"
#include "vis.hpp"



int main(int argc, char* argv[]) {
  std::vector<cv::String> images;
  //the path of the images taken by a camera for calibration can be changed below
  std::string path = "../data/images/cam0_copy/*.png";
  cv::glob(path, images);
  float grid_size = 0.7;
  float grid_spacing = 0.25;
  int grid_rows  = 6;
  int grid_columns = 8;
  int grid_startID = 0;
  AprilTags::AprilGrid Grid1(grid_rows,grid_columns,grid_size,grid_spacing,grid_startID);
  std::vector<std::vector<cv::Vec2f>> timagepoints1;
  std::vector<std::vector<cv::Vec3f>> tobjectpoints1;
  

  for(int i{0}; i<images.size(); i++)
  {
      cv::Mat image = cv::imread(images[i], cv::IMREAD_GRAYSCALE);
      AprilTags::TagDetector tagDetector(AprilTags::tagCodes36h11);
      vector<AprilTags::TagDetection> detections = tagDetector.extractTags(image);
      //Uncomment the below line if you want to view the images used for calibration
      //displayDetection(image, detections);
      std::vector<cv::Vec2f>  imagepoints1 = Grid1.imgpoints(detections,Grid1.rows,Grid1.columns,Grid1.start_ID);     
      std::vector<cv::Vec3f>  objectpoints1 = Grid1.objpoints(Grid1.size,Grid1.spacing);
      int grid_points = Grid1.columns*Grid1.rows*4;
      if(imagepoints1.size() == grid_points){
      timagepoints1.push_back(imagepoints1);
      tobjectpoints1.push_back(objectpoints1);
      cout << "image number"<< i << " accepted" << endl;
      }else {
          cout << "image number" << i << " rejected" << endl;
      }
      
  }
  cv::destroyAllWindows();
  cv::Mat cameraMatrix,distCoeffs,R,T;
  cv::calibrateCamera(tobjectpoints1, timagepoints1, cv::Size(6,8), cameraMatrix, distCoeffs, R, T);

  std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
  std::cout << "distCoeffs : " << distCoeffs << std::endl;
  std::cout << "Rotation vector : " << R << std::endl;
  std::cout << "Translation vector : " << T << std::endl;



  return 0;
}
