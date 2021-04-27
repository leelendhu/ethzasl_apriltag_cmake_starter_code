
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


int main(int argc, char* argv[]) {
  float grid_size = 0.7;
  float grid_spacing = 0.25;
  int grid_rows  = 6;
  int grid_columns = 8;
  int grid_startID = 0;
  AprilTags::AprilGrid Grid1(grid_rows,grid_columns,grid_size,grid_spacing,grid_startID);
  std::string imagefoldername= "../data/vlog42";
  int cameranumber = 0;
  std::vector<cv::Mat> pro = calibration(Grid1,imagefoldername,cameranumber);

return 0;
}
