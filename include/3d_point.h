#ifndef THREEDPOINT_H
#define THREEDPOINT_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

class ThreeDPoint {
public:
    // List of images in which this point was seen
    vector<int> imgIndex;
    // Within those images in which it was seen, whats the key poit index of the 2D point corresponding to this 3D point.
    vector<int> kpIndex;
    // Location of the point
    Point3d position;
};

#endif

