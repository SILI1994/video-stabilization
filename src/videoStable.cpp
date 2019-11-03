#include "videoStable.h"

//-------------------------------------------------------------------------------

VideoStable::VideoStable(const string inputPath, const string outputPath, const int SMOOTHING_RADIUS, const int HORIZONTAL_BORDER_CROP) {
	this->inputPath = inputPath;
	this->outputPath = outputPath;
	this->SMOOTHING_RADIUS = SMOOTHING_RADIUS;
	this->HORIZONTAL_BORDER_CROP = HORIZONTAL_BORDER_CROP;
}

//-------------------------------------------------------------------------------

void VideoStable::run() const {
    // Step 0 - read in the video
	VideoCapture cap(inputPath);
    assert(cap.isOpened());
    Mat cur, cur_grey;
    Mat prev, prev_grey;
    cap >> prev;
    cvtColor(prev, prev_grey, COLOR_BGR2GRAY);

    // Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames 
    vector <TransformParam> prev_to_cur_transform; // previous to current
    Mat last_T;
    while(true) {
        cap >> cur;
        if(cur.data == NULL) { break; }
        cvtColor(cur, cur_grey, COLOR_BGR2GRAY);
        // vector from prev to cur
        vector <Point2f> prev_corner, cur_corner;
        vector <Point2f> prev_corner2, cur_corner2;
        vector <uchar> status;
        vector <float> err;
        goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
        calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);
        // weed out bad matches
        for(size_t i=0; i < status.size(); i++) {
            if(status[i]) {
                prev_corner2.push_back(prev_corner[i]);
                cur_corner2.push_back(cur_corner[i]);
            }
        }
        // translation + rotation only
        Mat T = estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing
        // in rare cases no transform is found. We'll just use the last known good transform.
        if(T.data == NULL) { last_T.copyTo(T); }
        T.copyTo(last_T);
        // decompose T
        double dx = T.at<double>(0,2);
        double dy = T.at<double>(1,2);
        double da = atan2(T.at<double>(1,0), T.at<double>(0,0));
        prev_to_cur_transform.push_back(TransformParam(dx, dy, da));
        cur.copyTo(prev);
        cur_grey.copyTo(prev_grey);
    }

    // Step 2 - Accumulate the frame to frame transformations to get the image trajectory 
    vector<Trajectory> trajectory = generateTrajectory(prev_to_cur_transform);

    // Step 3 - Smooth out the trajectory and generate new set of previous to current transform,
	vector<TransformParam> new_prev_to_cur_transform = smooth(trajectory, prev_to_cur_transform);

    // Step 4 - Apply the new transformation to the video
    cap.set(CAP_PROP_POS_FRAMES, 0);
    Mat T(2,3,CV_64F);
	vector<Mat> stabilizedFrames;
    int vert_border = HORIZONTAL_BORDER_CROP * prev.rows / prev.cols; // get the aspect ratio correct
    for(int k=0; k<cap.get(CAP_PROP_FRAME_COUNT)-1; ++k) { // don't process the very last frame, no valid transform
        cap >> cur;
        if(cur.data == NULL) { break; }
        T.at<double>(0,0) = cos(new_prev_to_cur_transform[k].da);
        T.at<double>(0,1) = -sin(new_prev_to_cur_transform[k].da);
        T.at<double>(1,0) = sin(new_prev_to_cur_transform[k].da);
        T.at<double>(1,1) = cos(new_prev_to_cur_transform[k].da);
        T.at<double>(0,2) = new_prev_to_cur_transform[k].dx;
        T.at<double>(1,2) = new_prev_to_cur_transform[k].dy;
        Mat cur2; //the stabilized frame
        warpAffine(cur, cur2, T, cur.size());
        cur2 = cur2(Range(vert_border, cur2.rows-vert_border), Range(HORIZONTAL_BORDER_CROP, cur2.cols-HORIZONTAL_BORDER_CROP));
        resize(cur2, cur2, cur.size());
		
        // Now draw the original and stablised side by side for coolness
        Mat canvas = Mat::zeros(cur.rows, cur.cols*2+10, cur.type());
        cur.copyTo(canvas(Range::all(), Range(0, cur2.cols)));
        cur2.copyTo(canvas(Range::all(), Range(cur2.cols+10, cur2.cols*2+10)));
        // If too big to fit on the screen, then scale it down by 2, hopefully it'll fit :)
        if(canvas.cols > 1920) {
            resize(canvas, canvas, Size(canvas.cols/2, canvas.rows/2));
        }
        imshow("before and after", canvas);
        waitKey(20);

        stabilizedFrames.push_back(cur2);
    }

	//Step 5 - output the stablized video
    int width = stabilizedFrames[0].cols;
    int height = stabilizedFrames[0].rows;
	VideoWriter dstVideo(outputPath, VideoWriter::fourcc('M','J','P','G'), 30, Size(width, height));
    for (int i=0; i<stabilizedFrames.size(); ++i) {
        dstVideo.write(stabilizedFrames[i]);
    }

}

//-------------------------------------------------------------------------------

/**
 * Step2: Accumulate the frame to frame transformations to get the image trajectory 
 */
vector <Trajectory> VideoStable::generateTrajectory(const vector<TransformParam> &prev_to_cur_transform) const {
	double a = 0; double x = 0; double y = 0;
    vector <Trajectory> trajectory; // trajectory at all frames
    for(size_t i=0; i<prev_to_cur_transform.size(); i++) {
        x += prev_to_cur_transform[i].dx;
        y += prev_to_cur_transform[i].dy;
        a += prev_to_cur_transform[i].da;
        trajectory.push_back(Trajectory(x,y,a));
    }
	return trajectory;
}

//-------------------------------------------------------------------------------

/**
 * Step3: Smooth out the trajectory using an averaging window 
 * 		  And then generate new prev-to-cur transform with the smoothed trajectory
 */
vector <TransformParam> VideoStable::smooth(const vector<Trajectory> &trajectory, const vector<TransformParam> &prev_to_cur_transform) const { 
	vector <Trajectory> smoothed_trajectory; // trajectory at all frames
	for(size_t i=0; i<trajectory.size(); i++) {
		double sum_x = 0; double sum_y = 0; double sum_a = 0;
		int count = 0;
		for(int j=-SMOOTHING_RADIUS; j <= SMOOTHING_RADIUS; j++) {
			if(i+j >= 0 && i+j < trajectory.size()) {
				sum_x += trajectory[i+j].x;
				sum_y += trajectory[i+j].y;
				sum_a += trajectory[i+j].a;
				count++;
			}
		}
		double avg_a = sum_a / count;
		double avg_x = sum_x / count;
		double avg_y = sum_y / count;
		smoothed_trajectory.push_back(Trajectory(avg_x, avg_y, avg_a));
	}
	double a = 0; double x = 0; double y = 0;
	vector<TransformParam> new_prev_to_cur_transform;
    for(size_t i=0; i < prev_to_cur_transform.size(); i++) {
        x += prev_to_cur_transform[i].dx;
        y += prev_to_cur_transform[i].dy;
        a += prev_to_cur_transform[i].da;
        // target - current
        double diff_x = smoothed_trajectory[i].x - x;
        double diff_y = smoothed_trajectory[i].y - y;
        double diff_a = smoothed_trajectory[i].a - a;
        double dx = prev_to_cur_transform[i].dx + diff_x;
        double dy = prev_to_cur_transform[i].dy + diff_y;
        double da = prev_to_cur_transform[i].da + diff_a;
        new_prev_to_cur_transform.push_back(TransformParam(dx, dy, da));
    }
	return new_prev_to_cur_transform;
}

//-------------------------------------------------------------------------------
