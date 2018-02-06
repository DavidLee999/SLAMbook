#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#inlcude "myslam/visual_odometry.h"

namespace myslam
{
    VisualOdometry::VisualOdometry() : state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_( 0 ), num_inliers_ ( 0 )
    {
        num_of_features_ = Config::get<int>("number_of_features");
        scale_factor_ = Config::get<double>("scale_factor");
        level_pyramid_ = Config::get<int>("level_pyramid");
        match_ratio_ = Config::get<float>("macth_ratio");
        max_num_lost_ = Config::get<float>("max_num_lost");
        min_inliers_ = Config::get<int>("min_inliers");
        key_frame_min_rot = Config::get<double>("keyframe_rotation");
        key_frame_min_trans = Config::get<double>("keyframe_translation");
        orb_ = cv::OBR::create(num_of_features_, scale_factor_, level_pyramid_);
    }

    VisualOdometry::~VisualOdometry() {}

    bool VisualOdometry::addFrame(Frame::Ptr frame)
    {
        switch (state_)
        {
            case INITIALIZING:
                {
                    state_ = OK;
                    curr_ = ref_ = frame;

                    map_->insertKeyFrame(frame);
                    extractKeyPoints();
                    computeDescriptors();

                    setRef3DPoints();

                    break;
                }

            case OK:
                {
                    curr_ = frame;
                    extractKeyPoints();
                    computeDescriptors();
                    featureMatching();
                    poseEstimationPnP();

                    if (checkEstimatedPose() == true)
                    {
                        curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;
                        ref_ = curr_;

                        setRef3DPoints();

                        num_lost_ = 0;

                        if (checkKeyFrame() = true)
                        {
                            addKeyFrame();
                        }
                    }
                    else
                    {
                        num_lost_++;
                        if (num_lost_ > max_num_lost_)
                        {
                            state_ = LOST;
                        }

                        return false;
                    }

                    break;
                }
            case LOST:
                {
                    cout << "vo has lost.\n";
                    break;
                }
        }

        return true;
    }

    void VisualOdometry::extractKeyPoints()
    { orb_->detect(curr_->color_, keypoints_curr_); }

    void VisualOdometry::computeDescriptors()
    { orb_->compute(curr_->color_, keypoints_cur_, descriptors_curr_); }

    void VisualOdometry::featureMatching()
    {
        vector<cv::DMatch> matches;
        cv::BFMathcer matcher(cv::NORM_HAMMING);
        matcher.match(descriptors_ref_, descriptors_curr_, matches);

        float min_dis = std::min_element(matches.begin(), matches.end(),
                [](const cv::DMatch& m1, const cv::DMatch& m2)
                { return m1.distance < m2.distance; });

        feature_matches_.clear();

        for (cv::DMatch& m : mathces)
        {
            if (m.distance < max<float>(min_dis * match_rotio_, 30.0))
            {
                feature_matches_.push_back(m);
            }
        }

        cout << "good matches: " << feature_matches_.size() << endl;
    }

}
