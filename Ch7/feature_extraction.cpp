#include <iostream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        cout << "usage: feature_extraction img1 img2" << endl;
        return 1;
    }

    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");


    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);


    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    Mat outimg1;
    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB points", outimg1);


    vector<DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);


    double min_dist = 10000, max_dist = 0;
    for (int i = 0; i < descriptors_1.rows; ++i)
    {
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    printf("----max dist: %f \n", max_dist);
    printf("----min dist: %f \n", min_dist);

    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; ++i)
        if (matches[i].distance <= max(2 * min_dist, 30.0))
            good_matches.push_back(matches[i]);

    Mat img_match, img_goodmatch;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);

    imshow("all matches", img_match);
    imshow("good matches", img_goodmatch);
    waitKey(0);

    return 0;
}
