#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam
{
    class VisualOdometry
    {
        public:
            typedef shared_ptr<VisualOdometry> Ptr;

            enum VOState {
                INITIALIZING = -1,
                OK = 0,
                LOST
            };

            VOState state_;
            Map::Ptr map_;
        
            Frame::Ptr ref_;
            Frame::Ptr curr_;

            cv::Ptr<cv::ORB> orb_;
            // vector<cv::Point3f> pts_3d_ref_;
            vector<cv::KeyPoint> keypoints_curr_;
            Mat descriptors_curr_;

            // Mat descriptors_ref_;
            // vector<cv::DMatch> feature_matches_;
            cv::FlannBasedMatcher matcher_flann_;
            vector<MapPoint::Ptr> match_3dpts_;
            vector<int> match_2dkp_index_;

            SE3 T_c_w_estimated_;
            int num_inliers_;
            int num_lost_;

            int num_of_features_;
            double scale_factor_;
            int level_pyramid_;
            float match_ratio_;
            int max_num_lost_;
            int min_inliers_;
            double key_frame_min_rot;
            double key_frame_min_trans;
            double map_point_erase_ratio_;

        public:
            VisualOdometry();
            ~VisualOdometry();

            bool addFrame(Frame::Ptr frame);

        protected:
            void extractKeyPoints();
            void computeDescriptors();
            void featureMatching();
            void poseEstimationPnP();
            // void setRef3DPoints();
            void optimizeMap();

            void addKeyFrame();
            void addMapPoints();
            bool checkEstimatedPose();
            bool checkKeyFrame();

            double getViewAngle(Frame::Ptr frame, MapPoint::Ptr point);
    };
}

#endif
