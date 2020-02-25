#include "camera_pose.h"

using namespace cv;
using namespace std;

Mat CameraPose::Trans(){
    return _t;
}
Mat CameraPose::Rot(){
    return _r;
}
void CameraPose::SetTrans(Mat t){
    _t = t;
}
void CameraPose::SetRot(Mat R){
    _r = R;
}
// Default Constructor
CameraPose::CameraPose(){
    _t = Mat::zeros(3,1,CV_64F);
    _r = Mat::eye(3,3, CV_64F);
    _k = Mat::eye(3,3, CV_64F);
    P = _k*Mat::eye(3, 4, CV_64F);
    _inc_r = Mat::eye(3,3, CV_64F);
    _inc_t = Mat::eye(3,3, CV_64F);
    T = Mat::eye(4, 4, CV_64F);
}
// Constructor when internal parameters are known
CameraPose::CameraPose( float f, float cx, float cy){
    _k = Mat::eye(3, 3, CV_64F);
    _k.at<double>(0,0) = f;
    _k.at<double>(1,1) = f;
    _k.at<double>(0,2) = cx;
    _k.at<double>(1,2) = cy;
    
    _t = Mat::zeros(3,1,CV_64F);
    _r = Mat::eye(3,3, CV_64F);
    P = _k*Mat::eye(3, 4, CV_64F);
    _inc_r = Mat::eye(3,3, CV_64F);
    _inc_t = Mat::eye(3,3, CV_64F);
    T = Mat::eye(4, 4, CV_64F);
    
}
// Constructor when internal parameters and the pose is known
CameraPose::CameraPose(float f, float cx, float cy, Mat inc_R, Mat inc_t, Mat prev){
    _k = Mat::eye(3, 3, CV_64F);
    _k.at<double>(0,0) = f;
    _k.at<double>(1,1) = f;
    _k.at<double>(0,2) = cx;
    _k.at<double>(1,2) = cy;
    // Set incremental values
    _inc_t = inc_t;
    _inc_r = inc_R;
    // First compute the overall transform from previous transform
    T = Mat::eye(4, 4, CV_64F);
    _inc_r.copyTo(T(Range(0, 3), Range(0, 3)));
    _inc_t.copyTo(T(Range(0, 3), Range(3, 4)));
    T = prev*T;
    // Retrieve overall rotation and translation
    _r = T(Range(0, 3), Range(0, 3));
    _t = T(Range(0, 3), Range(3, 4));
    // Compute the Projection Matrix
    CalculateProjectionMat();
    //cout << "T: " << T << endl;
    //cout << "P: " << P << endl;
    //cout << "K: " << _k << endl;
}
// Calculate the projection matrix
void CameraPose::CalculateProjectionMat(){
    // Compute the Projection Matrix
    Mat M = Mat::eye(3, 4, CV_64F);
    M(Range(0, 3), Range(0, 3)) = _r.t();
    M(Range(0, 3), Range(3, 4)) = -_r.t()*_t;
    //cout << "_r: " << _r << endl;
    //cout << "_t: " << _t << endl;
    //cout << "M: " << M << endl;
    P = _k*M;
}
void CameraPose::FillOptValue(){
    for( size_t i =0 ; i < 3; ++i){
        //cout << _t.at<double>(i,0) << endl;
        optVal[3+i] = _t.at<double>(i,0);
    }
}
//Mat CameraPose:: GetProjectionMat(){
//    return _p;
//}
