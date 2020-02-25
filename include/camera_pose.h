#ifndef CAMERAPOSE_H
#define CAMERAPOSE_H

#include <opencv2/opencv.hpp>

using namespace cv;

class CameraPose {
public:
    // Projection Mat
    Mat P;
    // Overall Transformation matrix
    Mat T;
    // Rotation matrix as axis angle + translation (for Bundle Adjustment (Ceres-Solver))
    double optVal[6];
    //Getters and Setters
    Mat Trans();
    Mat Rot();
    Mat IncTrans();
    Mat IncRot();
    void SetTrans(Mat t);
    void SetRot(Mat R);
    // Default Constructor
    CameraPose();
    // Constructor when internal parameters are known
    CameraPose( float f, float cx, float cy);
    CameraPose( float f, float cx, float cy, Mat inc_R, Mat inc_t, Mat prev);
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
