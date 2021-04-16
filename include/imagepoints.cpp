#include <string>
#include <vector>

#include "apriltags/Tag36h11.h"
#include "apriltags/TagDetector.h"
#include "opencv2/opencv.hpp"
#include "imagepoints.hpp"

std::vector<cv::Vec2f> imgpoints(const cv::Mat& image) {
  cv::Mat imagef;
  image.copyTo(imagef);

  AprilTags::TagDetector tagDetector(AprilTags::tagCodes36h11);
  vector<AprilTags::TagDetection> detections = tagDetector.extractTags(imagef);

  std::vector<cv::Vec2f> imagepoints;

  for (int h = 0; h <48;h++){
    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& det = detections[i];
        if (det.id == h){
            for (int j = 0; j < 4; ++j){
                cv::Vec2f currentpoint = {det.p[j].first,det.p[j].second};
                //std::cout << currentpoint << endl ;
                imagepoints.push_back(currentpoint);
                }
        
    }
  }    
  }

 
  return imagepoints;
}