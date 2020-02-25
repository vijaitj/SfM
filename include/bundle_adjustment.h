#ifndef BUNDLE_ADJUSTMENT_H
#define BUNDLE_ADJUSTMENT_H

#include "opencv2/opencv.hpp"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

// Reprojection error for bundle adjustment
struct ReprojectionError
{
    ReprojectionError(double observed_x, double observed_y, double f, double cx, double cy ) : observed_x(observed_x), observed_y(observed_y), focal_length(f), cx(cx) , cy(cy) {}

    template <typename T>
    bool operator()(const T* const camera, const T* const point, T* residuals) const
    {
        // Rotation and translation
        T p[3];
        // camera[0,1,2] are the angle-axis rotation.
        ceres::AngleAxisRotatePoint(camera, point, p);
        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Projection onto image plane and conversion to image coords
        T predicted_x = focal_length * p[0] / p[2] + cx;
        T predicted_y = focal_length * p[1] / p[2] + cy;

        //
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;
        return true;
    }

    static ceres::CostFunction* create(const double observed_x, const double observed_y, double f, double cx, double cy)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(new ReprojectionError(observed_x, observed_y, f, cx, cy)));
    }

    double observed_x;
    double observed_y;
    double focal_length;
    double cx;
    double cy;
};

#endif
