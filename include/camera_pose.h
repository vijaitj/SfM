#ifndef CAMERAPOSE_H
#define CAMERAPOSE_H

#include <opencv2/opencv.hpp>

class CameraPose {
public:
    // Projection Mat
    cv::Mat P;
    // Overall Transformation matrix
    cv::Mat T;
    // Rotation matrix as axis angle + translation (for Bundle Adjustment (Ceres-Solver))
    double optVal[6];
    //Getters and Setters
    cv::Mat Trans();
    cv::Mat Rot();
    cv::Mat IncTrans();
    cv::Mat IncRot();
    void SetTrans(cv::Mat t);
    void SetRot(cv::Mat R);
    // Default Constructor
    CameraPose();
    // Constructor when internal parameters are known
    CameraPose( float f, float cx, float cy);
    CameraPose( float f, float cx, float cy, cv::Mat inc_R, cv::Mat inc_t, cv::Mat prev);
    // Generate and return the projection matrix
    void CalculateProjectionMat();
    void FillOptValue();
private:
    // Variables
    // Stores overall rotation and translation
    Mat _t;
    Mat _r;
    //Stores incremental rotation and translation w.r.t to previous image/frame
    Mat _inc_r;
    Mat _inc_t;
    // Internal caliberation matrix
    Mat _k;
    
};                                    

#endif
