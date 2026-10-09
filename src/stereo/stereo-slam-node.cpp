#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    //stringstream ss(strDoRectify);
    //ss >> boolalpha >> doRectify;

    doRectify_ = std::stoi(strDoRectify);
    // doEqual_ = std::stoi(strDoEqual);

    //bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    // std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera_left/image_color");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera_right/image_color");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
    m_SLAM->SaveMapPoints("MapPoints.txt");

    m_SLAM->OutputEvaluationParams();
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Copy the left ros image message to cv::Mat with bgr encoding
    // this enables the custom channel weighting before converting to grayscale -> Tracking.cc
    // cv_bridge handles the ROS bayer_rggb8 (BayerBG in OpenCV) conversion to mono8, bgr8 or rgb8
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the right ros image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat imLeft, imRight;
    double t = Utility::StampToSec(msgLeft->header.stamp);

    imLeft = cv_ptrLeft->image;
    imRight = cv_ptrRight->image;

    // // set equalization to true for CLAHE
    // if (bClahe_)
    // {
    //     clahe_->apply(imLeft, imLeft);
    //     clahe_->apply(imRight, imRight);
    // }

    if (doRectify_){
        cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
    }
    
    m_SLAM->TrackStereo(imLeft, imRight, t);

    // states definition from enum in Tracking.h
    static const char* states[] = {"NO_IMAGES_YET", "NOT_INITIALIZED", "OK", "RECENTLY_LOST", "LOST", "OK_KLT"};
    int state = m_SLAM->GetTrackingState();
    const char* name = (state >= 0 && state <= 5) ? states[state] : "SYSTEM_NOT_READY";


    RCLCPP_INFO(this->get_logger(), "frame %.3f processed, state: %s", t, name);

}
