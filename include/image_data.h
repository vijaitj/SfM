#ifndef IMAGEDATA_H
#define IMAGEDATA_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "camera_pose.h"


class ImageData {
 public:
    cv::Mat img;
    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;
    CameraPose cam;
    //
    
};

#endif
