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
#include "writeconfig.hpp"


int main(int argc, char* argv[]) {
  float grid_size = 0.035;
  float grid_spacing = 0.25;
  int grid_rows  = 6;
  int grid_columns = 8;
  int grid_startID = 0;
  AprilTags::AprilGrid Grid1(grid_rows,grid_columns,grid_size,grid_spacing,grid_startID);
  std::string imagefoldername0= "../data/vlog42";
  int cameranumber0 = 0;
  std::vector<cv::Mat> matr0 = calibration(Grid1,imagefoldername0,cameranumber0);
  std::string imagefoldername1= "../data/vlog42";
  int cameranumber1 = 1;
  std::vector<cv::Mat> matr1 = calibration(Grid1,imagefoldername1,cameranumber1);
  std::string imagefoldername2= "../data/vlog42";
  int cameranumber2 = 2;
  std::vector<cv::Mat> matr2 = calibration(Grid1,imagefoldername2,cameranumber2);
  
  int configfilerwrite = writeconfig(matr0,matr1,matr2);
  
return 0;
}
