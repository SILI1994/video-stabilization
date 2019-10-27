 #pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>

using namespace std;
using namespace cv;

//----------------------------------------------------------------------------------

struct TransformParam {
    TransformParam() {}
    TransformParam(double _dx, double _dy, double _da) {
        dx = _dx;
        dy = _dy;
        da = _da;
    }

    double dx;
    double dy;
    double da; // angle
};

struct Trajectory {
    Trajectory() {}
    Trajectory(double _x, double _y, double _a) {
        x = _x;
        y = _y;
        a = _a;
    }
    double x;
    double y;
    double a; // angle
};

//----------------------------------------------------------------------------------

class VideoStable {
  
public:
    /**
     * Params:
     * SMOOTHING_RADIUS      : In frames. The larger the more stable the video, but less reactive to sudden panning
     * HORIZONTAL_BORDER_CROP: In pixels. Crops the border to reduce the black borders from stabilisation being too noticeable.
     */
    VideoStable(const string inputPath, const string outputPath, const int SMOOTHING_RADIUS = 70, const int HORIZONTAL_BORDER_CROP = 60);
    
    /**
     * Run the stabilization algorithm
     */
    void run() const;

private:
    vector <Trajectory> generateTrajectory(const vector<TransformParam> &prev_to_cur_transform) const;
    vector <TransformParam> smooth(const vector<Trajectory> &trajectory, const vector<TransformParam> &prev_to_cur_transform) const;

public:


private:
    string inputPath, outputPath;  
    int SMOOTHING_RADIUS;
    int HORIZONTAL_BORDER_CROP;

};
 

