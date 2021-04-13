#include <string>
#include <vector>

#include "apriltags/Tag36h11.h"
#include "apriltags/TagDetector.h"
#include "opencv2/opencv.hpp"
#include "vis.hpp"

int main(int argc, char* argv[]) {
  std::string filename = "../data/sample.png";
  if (argc > 1) filename = argv[1];
  cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);

  AprilTags::TagDetector tagDetector(AprilTags::tagCodes36h11);
  vector<AprilTags::TagDetection> detections = tagDetector.extractTags(image);

  displayDetection(image, detections);

  return 0;
}
