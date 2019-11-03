 #pragma once

#include <opencv2/core/core.hpp>	
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//----------------------------------------------------------------------------------

class VideoStable {
  
public:
    
    /**
     * Choose your feature type from "SIFT", "SURF" and "ORB"
     */
    VideoStable(const string inputPath, const string outputPath, const string feature_type = "SIFT");
    
    /**
     * Run the static-video algorithm
     */
    void run() const;

private:

    /**
     * return the homography matrix with different kinds of features  
     */
    Mat calcRigidTransform(const Mat &img1, const Mat &img2) const;


private:
    string inputPath, outputPath;  
    string feature_type;
};
 