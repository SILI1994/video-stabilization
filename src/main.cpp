#include "videoStable.h"

//---------------------------------------------------------------------------

int main(int argc, char **argv) {

    // if (argc < 3) cout << "\nYOU NEED MORE INPUT ARGS!\n" << endl;
    // string inputPath = argv[1];
    // string outputPath = argv[2];
    // VideoStable stabilizer = VideoStable(inputPath, outputPath);
    // stabilizer.run();
  
    Mat img1 = imread(argv[1]);
    Mat img2 = imread(argv[2]);

    // -- Step1: extract features and describe them
    vector<KeyPoint> kp1, kp2;  
    Mat desc1, desc2;  
    Ptr<ORB> detector = ORB::create();
    detector->detectAndCompute( img1, noArray(), kp1, desc1 );
    detector->detectAndCompute( img2, noArray(), kp2, desc2 );
    
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
   
    // -- Step 4: Get the keypoints from the good matches 
    vector<Point2f> good_kp1, good_kp2;
    for( size_t i = 0; i < good_matches.size(); i++ ){
        good_kp1.push_back( kp1[ good_matches[i].queryIdx ].pt );
        good_kp2.push_back( kp2[ good_matches[i].trainIdx ].pt );
    }

    // -- Step 5: Calc homography
    Mat T = estimateRigidTransform(good_kp2, good_kp1, false); // false = rigid transform, no scaling/shearing
 
    // -- Step 6: Warp the image
    warpAffine(img2, img2, T, img2.size());

    imwrite("ret.JPG", img2);



    return 0;
}