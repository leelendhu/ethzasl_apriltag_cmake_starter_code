#include <string>
#include <vector>

#include "apriltags/Tag36h11.h"
#include "apriltags/TagDetector.h"
#include "opencv2/opencv.hpp"
#include "vis.hpp"
#include "objectpoints.hpp"
#include "imagepoints.hpp"


int main(int argc, char* argv[]) {
  std::vector<cv::String> images;
  std::string path = "../data/images/cam0_copy/*.png";
  cv::glob(path, images);
  
  //std::string filename = "../data/sample.png";
  //if (argc > 1) filename = argv[1];
  //cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
  std::vector<std::vector<cv::Vec2f>> timagepoints1;
  std::vector<std::vector<cv::Vec3f>> tobjectpoints1;
  for(int i{0}; i<images.size(); i++)
  {
      cv::Mat image = cv::imread(images[i], cv::IMREAD_GRAYSCALE);
      AprilTags::TagDetector tagDetector(AprilTags::tagCodes36h11);
      vector<AprilTags::TagDetection> detections = tagDetector.extractTags(image);
      displayDetection(image, detections);
      std::vector<cv::Vec2f>  imagepoints1 = imgpoints(image);
      float a = .07;
      float b = .25;
      std::vector<cv::Vec3f>  objectpoints1 = objpoints(a,b);
      
      if(imagepoints1.size() == 192){
      timagepoints1.push_back(imagepoints1);
      tobjectpoints1.push_back(objectpoints1);
      cout << "accepted" << endl;
      }else {
          cout << "rejected" << endl;
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
