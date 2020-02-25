#ifndef IMAGEDATA_H
#define IMAGEDATA_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "camera_pose.h"
using namespace cv;

class ImageData {
 public:
    Mat img;
    std::vector<KeyPoint> kp;
    Mat desc;
    CameraPose cam;
    //
    
};

#endif
