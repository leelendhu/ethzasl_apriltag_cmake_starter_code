#include <string>
#include <vector>

#include "apriltags/Tag36h11.h"
#include "apriltags/TagDetector.h"
#include "opencv2/opencv.hpp"
#include "objectpoints.hpp"

std::vector<cv::Vec3f> objpoints(const float a1, const float b1) {
  

  float a;
  float b;
  float sum;
  a = a1;
  b = b1;
  float c = b*a;
  sum = c + a;

  std::vector<cv::Vec3f> objectpoints;
  
  for (int i = 0; i < 6; i++){
    for (int j = 0; j < 8; j++){
      std::vector<cv::Vec3f > obj;
      float toprightx = (float)j * sum;
      float toprighty = (float)i * sum;
      cv::Vec3f topright = {toprightx, toprighty, 0};
      cv::Vec3f topleft = {(float)j * sum + a, (float)i * sum, 0};
      cv::Vec3f bottomleft = {(float)j * sum + a, (float)i * sum + a, 0};
      cv::Vec3f bottomright = {(float)j * sum, (float)i * sum + a, 0};
      objectpoints.push_back(topright);
      objectpoints.push_back(topleft);
      objectpoints.push_back(bottomleft);
      objectpoints.push_back(bottomright);
    }
  }

 
  return objectpoints;
}