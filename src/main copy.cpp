#include <opencv2/core/core.hpp>	
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    VideoCapture cap("inputPath");
    assert(cap.isOpened());
    cout << "there are " << cap.get(CAP_PROP_FRAME_COUNT) << " frames in the video" << endl;

    return 0;
}