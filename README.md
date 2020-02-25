# Structure From Motion (SFM)

This program aims at performing 3D reconstruction from multiple images. Given an ordered list of images within a folder, the algorithm performs the following steps:
1. Feature detetction and extarction of descriptors
2. Feature Matching
3. Estimate the essential matrix and the camera pose
4. Estimate relative scale (when we have 3 or more images)
5. Refine estimates using Bundle Adjustment.

The main classes and their basic purposes are as follows:
1. ImageData: There is one instance of this class per image. Its stores image related metadata (like keypoints and descriptors for the image and the camera pose)
2. CameraPose: This class deals with the camera related functions. Each ImageData class will have a CameraPose object as its member (composition). The CameraPose would tell us the rotation and translation of the camera when a particular image was taken. It also has related functionalities like calculating the projection matrix etc.
3. ThreeDPoint : This class stores the 3D points reconstructed. It also tells us whether a 3D point was observed in an image and if it was observed which key point within the image corresponds to it.
4. BundleAdjustment: This class defines the necessary structures (like the cost function) for optimization. Google Ceres solver is used to optimize the 3D locations and the cmaera poses jointly in bundle adjustment.

The program when run will show you the feature matches and then proceed to calculate the relative scale (which will be displayed on the console). The Optimization step summary is also printed on to the console. The reconstructed 3D points are written as a text file onto the input directory in which the input images are present.

## Dependencies
* cmake >= 3.11
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Google Ceres Solver >= 1.14.0
  * Instructions for installation: http://ceres-solver.org/installation.html
* Boost Library >= 1.72.0
  * Can be downloaded here: https://www.boost.org/users/download/
* OpenCV >= 4.2.0 
  * Linux installation instructions: https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html.
* Eigen Matrix Library >= 3.3.7
  * It can be downloaded from: http://eigen.tuxfamily.org/index.php?title=Main_Page

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./SFM`.


This project is based on this tutorial: http://rpg.ifi.uzh.ch/visual_odometry_tutorial.html
