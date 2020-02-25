#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include "camera_pose.h"
#include "3d_point.h"
#include "image_data.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "bundle_adjustment.h"



using namespace std;
namespace bfs = boost::filesystem;

// downsample the image to speed up processing
const int DOWNSAMPLE_FACTOR = 4;
// focal length in pixels, after downsampling
const double FOCAL_LENGTH = 4308 / DOWNSAMPLE_FACTOR;
// Path to the directory withh all the image files
const string PATH_TO_FILES = "../Images/";
const bool SHOW_RESULTS = true;

// A helper function to enable code reuse. This function helps in building up the list of 3D points. It needs to be called twice, within each pair of image matches, once to obtain the 3D points needed to estimate scale and a second time to create the actual 3D points list.
void Process3DPoints(vector<uchar> &mask, Mat & cur3D, int i, vector<int> & idx1, vector<int> & idx2, map<int, int> &cur_featureTo3D, vector<pair<int,Point3f>> &seen3D,vector<ThreeDPoint> &all3D, map<int, int> &featureTo3D, bool addToAll3D){
    for (size_t k=0; k < mask.size(); k++) {
        // Keep only inlier matches
        if (mask[k]) {
            // Convert from homogeneous coordinates.
            Point3d tmp3D;
            tmp3D.x = cur3D.at<float>(0, k) / cur3D.at<float>(3, k);
            tmp3D.y = cur3D.at<float>(1, k) / cur3D.at<float>(3, k);
            tmp3D.z = cur3D.at<float>(2, k) / cur3D.at<float>(3, k);
            // If this 3D point was seen before
            if(featureTo3D.count(idx1[k]) > 0){
                // the index of this seen point in all3D
                auto idxToUnq3D = featureTo3D[idx1[k]];
                if(!addToAll3D){
                    seen3D.push_back(pair<int,Point3f>(idxToUnq3D,tmp3D));
                }
                else{
                    //cout << "all3d " << all3D[idxToUnq3D].position << endl;
                    //cout << "tmp3D " << tmp3D << endl;
                    // Update the already seen 3d point to indicate that i+1 th image saw it at feature point number idx2[k]
                    all3D[idxToUnq3D].imgIndex.emplace_back(i+1);
                    all3D[idxToUnq3D].kpIndex.push_back(idx2[k]);
                    // the two 3D locations could be different, so replace it with their average. Could be improved
                    all3D[idxToUnq3D].position = (all3D[idxToUnq3D].position + tmp3D)/2.0f ;
                }
                // Update the 2D to 3D map for next iteration
                cur_featureTo3D[idx2[k]] = idxToUnq3D;
                
            }
            else{
                if(addToAll3D){
                    // Add the newly seen 3D point to the list of all 3D points
                    ThreeDPoint t3D;
                    // Since this point is new and was seen in both images add that info to the ThreeDPoint object
                    t3D.imgIndex.emplace_back(i);
                    t3D.kpIndex.push_back(idx1[k]);
                    t3D.imgIndex.emplace_back(i+1);
                    t3D.kpIndex.push_back(idx2[k]);
                    
                    t3D.position = tmp3D;
                    all3D.emplace_back(t3D);
                    // Update the 2D to 3D map for next iteration
                    cur_featureTo3D[idx2[k]] = all3D.size() - 1;
                }
            }
        }
    }
}

int main() {
    
    //m = m.t();
    //float* mp = &m.at<float>(0);
    
    /**************** Read all files in folder and sort them by filename  ******************/
    bfs::path dirPath(PATH_TO_FILES);
    vector<string> filePaths;
    for (auto i = bfs::directory_iterator(dirPath); i != bfs::directory_iterator(); i++)
    {
        // Only files, no directories
        if (!bfs::is_directory(i->path()))
        {
            filePaths.emplace_back(i->path().filename().string()) ;
        }
        else
            continue;
    }
    //If we found no files
    if(filePaths.empty()){
        cout << "No image files found in the input directory. Exiting the program.\n" ;
        return 0;
    }
    
    // Sorting by filename
    sort(filePaths.begin(), filePaths.end());
    
    /**************** Detect features and extract descriptors  ******************/
    Ptr<AKAZE>  feature = AKAZE::create();
    namedWindow("img", WINDOW_NORMAL);
    
    //Vector to hold all image data
    vector<ImageData> allImgData;
    
    for (auto &f : filePaths)
    {
        ImageData id;
        // Read in the image
        Mat img = imread(PATH_TO_FILES + f);
        if(img.empty())
            continue;
        // Downsample
        resize(img, img, img.size()/DOWNSAMPLE_FACTOR);
        id.img = img;
        // convert to grayscale
        cvtColor(img, img, COLOR_BGR2GRAY);
        // Detect feature points and compute descriptors for the detected feature points
        feature->detectAndCompute(img, noArray(), id.kp, id.desc);
        
        allImgData.emplace_back(id);
    }
    //If we found no files
       if(allImgData.empty()){
           cout << "No image files found in the input directory. Exiting the program.\n" ;
           return 0;
       }
    /************************* Match Features, Estimate Pose and Estimate Realtive Scale  ****************************/
    // We are going to assume that all images have the same pricipal point
    float cx = allImgData[0].img.size().width/2;
    float cy = allImgData[0].img.size().height/2;
    Point2d pp(cx, cy);
    // Set the camera pose for the first image
    CameraPose cp(FOCAL_LENGTH,cx,cy);
    allImgData[0].cam = move(cp);
    //Initialize the overall transform
    Mat overallT = Mat::eye(4, 4, CV_64F);
    // Initilaize the map that tells you if a feature point was observed in the prior image
    // Note: this map can be made to map from a std::pair to 3D points, that way we could identify obseved 3D points in all prior images (instead of just the last image)
    map<int, int> featureTo3D;
    // Stores unique 3D points
    vector<ThreeDPoint> all3D;
    
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    const float ratio_thresh = 0.7f;
    for(size_t i = 0 ; i < allImgData.size() - 1; ++i){
        auto &id1 = allImgData[i];
        auto &id2 = allImgData[i+1];
        // Get the best and second best matches
        vector< vector<DMatch> > knn_matches;
        matcher->knnMatch( id1.desc, id2.desc, knn_matches, 2 );
        // Filter matches using the Lowe's ratio test
        std::vector<DMatch> good_matches;
        vector<Point2f> px1, px2;
        vector<int> idx1, idx2;
        for (size_t k = 0; k < knn_matches.size(); ++k)
        {
            if (knn_matches[k][0].distance < ratio_thresh * knn_matches[k][1].distance)
            {
                good_matches.push_back(knn_matches[k][0]);
                idx1.emplace_back(knn_matches[k][0].queryIdx);
                idx2.emplace_back(knn_matches[k][0].trainIdx);
                px1.push_back(id1.kp[idx1.back()].pt);
                px2.push_back(id2.kp[idx2.back()].pt);
            }
        }
        // Show matches if the flag is set.
        if(SHOW_RESULTS){
            //Draw matches
            Mat img_matches;
            drawMatches( id1.img, id1.kp, id2.img, id2.kp, good_matches, img_matches, Scalar::all(-1),
                        Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            //Show detected matches
            imshow("Good Matches", img_matches );
            waitKey(500);
        }
        // Estimate the essential matrix
        vector<uchar> mask;
        Mat E = findEssentialMat(px2, px1, FOCAL_LENGTH, pp, RANSAC, 0.999, 1.0, mask);
        // Recover the pose
        Mat incR, inc_t;
        recoverPose(E, px2, px1, incR, inc_t, FOCAL_LENGTH, pp, mask);
        //
        //cout << "IncR: " << incR << endl;
        //cout << "Inc T: " << inc_t << endl;
        CameraPose cp(FOCAL_LENGTH,cx,cy,incR,inc_t,id1.cam.T);
        id2.cam = move(cp);
        // Triangulate to get 3D points
        //cout << "ID1 Cam P: " << id1.cam.P << endl;
        //cout << "ID2 Cam P: " << id2.cam.P << endl;
        
        Mat cur3D;
        triangulatePoints(id1.cam.P, id2.cam.P, px1, px2, cur3D);
        // count of number of 3D points so far
        //int num3D = all3D.size();
        //Vector to store the new values of 3D points sen before
        vector<pair<int,Point3f>> seen3D;
        map<int, int> cur_featureTo3D;
        
        if(i == 0)
            Process3DPoints(mask,cur3D,i,idx1,idx2,cur_featureTo3D,seen3D,all3D,featureTo3D,true);
        else
            Process3DPoints(mask,cur3D,i,idx1,idx2,cur_featureTo3D,seen3D,all3D,featureTo3D,false);
        // Compute scale if i > 0
        if(i > 0){
            // Use a few random point pairs to compute scale
            const int t = seen3D.size() - 1;
            int Lmt = 2500, counter =0 ;
            double scale = 0;
            for( int k =0 ; k < Lmt ; ++k){
                auto i1 = rand()%t ;
                auto i2 = rand()%t;
                if( i1 == i2)
                    continue;
                scale += (norm(all3D[seen3D[i1].first].position - all3D[seen3D[i2].first].position) / norm(seen3D[i1].second - seen3D[i2].second));
                counter++;
            }
            scale /= counter;
            cout << "Sacle: " << i << " to " << i + 1 << " : " << scale << endl;
            //Scale the translation vector and Recalculate the projection matrix
            CameraPose cp(FOCAL_LENGTH,cx,cy,incR,scale*inc_t,id1.cam.T);
            id2.cam = move(cp);
            //Recalculate the 3D points
            Mat cur3D;
            triangulatePoints(id1.cam.P, id2.cam.P, px1, px2, cur3D);
            //Vector to store the new values of 3D points seen before
            vector<pair<int,Point3f>> seen3D;
            map<int, int> cur_featureTo3D;
            Process3DPoints(mask,cur3D,i,idx1,idx2,cur_featureTo3D,seen3D,all3D,featureTo3D,true);
        }
        
        featureTo3D = cur_featureTo3D;
    }
    /*************************  Bundle Adjustemnt  ****************************/
    // For the camerapose of each image set the transposed rotation matrix
    for( auto &id :  allImgData){
        Mat tmp = id.cam.Rot().t();
        double* mp = &tmp.at<double>(0);
        ceres::RotationMatrixToAngleAxis( mp, id.cam.optVal);
        //add translation components too
        id.cam.FillOptValue();
        //cout << "opt: " << id.cam.optVal[3] << " " << id.cam.optVal[4] << " " << id.cam.optVal[5] << endl;
    }
    
    ceres::Problem problem;
    //For each 3D point
    for( size_t i = 0  ; i < all3D.size() ; ++i){
        //For each image in whcih this 3D point was observed add a residual block
        auto &cur_pnt = all3D[i];
        for(size_t j = 0; j <  cur_pnt.imgIndex.size(); ++j)
        {
            int img_id = cur_pnt.imgIndex[j];
            int kp_id = cur_pnt.kpIndex[j];
            //
            Point2f observed = allImgData[img_id].kp[kp_id].pt;
            ceres::CostFunction* cost_function = ReprojectionError::create(observed.x,observed.y, FOCAL_LENGTH, cx, cy);
            double* camera = (double*)  &allImgData[img_id].cam.optVal[0];
            double * p3D = (double*) &cur_pnt.position;
            problem.AddResidualBlock(cost_function,NULL, camera,p3D);
        }
        
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 5e2;
    options.function_tolerance = 1e-6;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    //Write 3D points to file
    ofstream myfile(PATH_TO_FILES + "/3DPoints.txt");
    if (myfile.is_open()){
        for (size_t i = 0; i < all3D.size(); i++){
            myfile << all3D[i].position.x << "\t" << all3D[i].position.y << "\t" << all3D[i].position.z << "\n";
        }
        myfile.close();
    }
    
    return 0;
}
