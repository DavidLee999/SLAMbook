#include "DBoW3/DBoW3.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    cout << "reading images ...\n";
    vector<Mat> images;
    for (int i = 0; i < 10; i++)
    {
        string path = "../../Ch12/data/" + to_string(i + 1) + ".png";
        images.push_back(imread(path));
    }

    cout <<"detecting ORB features ...\n";
    Ptr<Feature2D> detector = ORB::create();
    vector<Mat> descriptors;
    for (Mat& image : images)
    {
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detectAndCompute(image, Mat(), keypoints, descriptor);
        descriptors.push_back(descriptor);
    }

    cout << "creating vocabulary ...\n";
    DBoW3::Vocabulary vocab;
    vocab.create(descriptors);

    cout << "vocabulary info: " << vocab << endl;
    vocab.save("vocabulary.yml.gz");
    cout << "done.\n";

    return 0;
}
