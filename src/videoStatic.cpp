#include "videoStatic.h"

//-------------------------------------------------------------------------------

VideoStatic::VideoStatic(const string inputPath, const string outputPath, const string feature_type) {
	this->inputPath = inputPath;
    this->outputPath = outputPath;
    this->feature_type = feature_type;
}

//-------------------------------------------------------------------------------

void VideoStatic::run() const {
    /* read in the video */
    VideoCapture cap(inputPath);
    assert(cap.isOpened());
    Mat cur, cur_grey;
    Mat prev, prev_grey;
    cap >> prev;
    cvtColor(prev, prev_grey, COLOR_BGR2GRAY);

    vector<Mat> stabilized_frames;

    /* frame-wise registration */
    while(true) {
        cap >> cur;
        if(cur.data == NULL) { break; }
        cvtColor(cur, cur_grey, COLOR_BGR2GRAY);
        Mat homo = calcHomography(cur_grey, prev_grey);
        warpPerspective( cur, cur, homo, Size(prev.cols, prev.rows) );	
        cur.copyTo(prev);
        cur_grey.copyTo(prev_grey);	
        stabilized_frames.push_back(cur);
        imshow("after", cur);
        waitKey(20);
    }
    
}

//-------------------------------------------------------------------------------

Mat VideoStatic::calcHomography(const Mat &img1, const Mat &img2) const {
    // -- Step1: extract features and describe them
    vector<KeyPoint> kp1, kp2;  
    Mat desc1, desc2;  
    if (feature_type == "SIFT") {
        Ptr<SIFT> detector = SIFT::create();
        detector->detectAndCompute( img1, noArray(), kp1, desc1 );
        detector->detectAndCompute( img2, noArray(), kp2, desc2 );
    }
    else if (feature_type == "SURF") {
        Ptr<SURF> detector = SURF::create();
        detector->detectAndCompute( img1, noArray(), kp1, desc1 );
        detector->detectAndCompute( img2, noArray(), kp2, desc2 );
    }
    else if (feature_type == "ORB") {
        Ptr<ORB> detector = ORB::create();
        detector->detectAndCompute( img1, noArray(), kp1, desc1 );
        detector->detectAndCompute( img2, noArray(), kp2, desc2 );
    }
    else {
        cout << "\n WRONG FEATURE TYPE!! \n" << endl;
        abort();
    }
    
    // -- Step 2: Matching descriptor vectors with a FLANN based matcher
    if (desc1.type() != CV_32F || desc2.type() != CV_32F) {
		desc1.convertTo(desc1, CV_32F);
		desc2.convertTo(desc2, CV_32F);
	}
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( desc1, desc2, knn_matches, 2 );

    // -- Step 3: Filter matches using the Lowe's ratio test
    vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < 0.75 * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
   
    // -- Step 4: Get the keypoints from the good matches and calc homography
    vector<Point2f> good_kp1, good_kp2;
    for( size_t i = 0; i < good_matches.size(); i++ ){
        good_kp1.push_back( kp1[ good_matches[i].queryIdx ].pt );
        good_kp2.push_back( kp2[ good_matches[i].trainIdx ].pt );
    }
    Mat H = findHomography( good_kp1, good_kp2, RANSAC );
    return H;
}