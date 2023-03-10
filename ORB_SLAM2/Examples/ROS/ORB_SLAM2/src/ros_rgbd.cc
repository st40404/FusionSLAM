/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

// add laser message type
#include <sensor_msgs/LaserScan.h>


// add pcl publish header
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// add Camera pose publish type header
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

// add Camera service
#include "all_process/CameraPose.h"

// #include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>   
using namespace std;
#define PI 3.14159265


class ImageGrabber
{
public:

    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    // void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::LaserScanConstPtr &msgSCAN);

    // define Pointcloud topic
    ros::Publisher KeyFrame_pub;

    // define camera Pose topic
    // ros::Publisher CamPose_pub;

    // define camera Pose service
    ros::ServiceServer CamPose_serv;

    // save every previous poase untill client send the request
    vector <geometry_msgs::PointStamped> PreviousPose ;


    // define CurrentFrame to get 2D(x,y) data from tracker
    vector<cv::KeyPoint> CurrentFrame;
    // define CurrentDepth to get depth(z) from tracker
    vector<float> CurrentDepth;
    // define CurrentPose to get camera pose from keyfram
    cv::Mat CurrentPose;
    // define pose to get save camera pose
    std_msgs::Float64MultiArray pose;
    // set pose varity setting
    void SetPose();

    // save lidar stamp
    ros::Time current_lidar_time_;
    // add callback function to get current frame from Tracker, and publish
    void callback();
    // add PublishPointcloud function to publish point cloud
    void PublishPointcloud();
    // add PublishCamPose function to publish camera pose
    void PublishCamPose();
    // add SurvicePose function of master service for camera pose
    bool SurvicePose(   all_process::CameraPose::Request  &req,
                        all_process::CameraPose::Response &res);

    ORB_SLAM2::System* mpSLAM;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);


    ros::NodeHandle nh;

    // define /Ron/KeyFrame topic (in 2021 project)
    // igb.KeyFrame_pub = nh.advertise<PointCloud>("/ORB/KeyFrame", 5);

    // define /Ron/CamPose topic (in 2021 project)
    // igb.CamPose_pub = nh.advertise<std_msgs::Float64MultiArray>("/Ron/CamPose", 1);
    // igb.CamPose_pub = nh.advertise<geometry_msgs::Twist>("/mobile_position", 1);

    // define /ORB/camera_pose topic (thesis use publish pose)
    // igb.CamPose_pub = nh.advertise<geometry_msgs::PointStamped>("/ORB/camera_pose", 5);

    // define /ORB/pose service master
    igb.CamPose_serv = nh.advertiseService("/ORB/pose", &ImageGrabber::SurvicePose, &igb);

    // set publish message of member pose
    igb.SetPose();


    // original code (in 2021 project)
    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/cam_AGV/color/image_raw", 1);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "cam_AGV/aligned_depth_to_color/image_raw", 1);
    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    // message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    // sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 5);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 5);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 5);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::LaserScan> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub, scan_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2, _3));


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");  

    // Save customized Map 
    SLAM.SaveMap("MapPointandKeyFrame.bin");

    ros::shutdown();

    return 0;
}


// get current data from tracker
void ImageGrabber::callback()
{
    CurrentFrame = mpSLAM->GetmpTracker();
    CurrentDepth = mpSLAM->GetmvDepth();
    CurrentPose = mpSLAM->Getpose();
}

// publish point clouds
void ImageGrabber::PublishPointcloud()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "camera_aligned_depth_to_color_frame";
    cloud.points.resize (CurrentFrame.size());
    for (size_t i=0; i<CurrentFrame.size(); i++)
    {
        cloud.points[i].x = CurrentFrame[i].pt.x / 100.0;
        cloud.points[i].y = CurrentFrame[i].pt.y / 100.0 ;
        cloud.points[i].z = CurrentDepth[i];
    }

    KeyFrame_pub.publish(cloud);
}
float numx = 0.0;
float numy = 0.0;
float numz = 0.0;

// publish camera pose
void ImageGrabber::PublishCamPose()
{
    // float* matData = (float*)CurrentPose.data;
   
    cv::Mat TransP = cv::Mat::eye(4,4,CV_32F);
    // TransP.at<float>(0, 3) = -2.47;
    // TransP.at<float>(1, 3) = -3.72;
    // cv::Mat TransX = cv::Mat::eye(4,4,CV_32F);
    cv::Mat TransZ = cv::Mat::eye(4,4,CV_32F);

    ///////     project use    //////
    // int param = 180;
    // TransX.at<float>(1, 2) = cos ( param * PI / 180.0 );
    // TransX.at<float>(1, 3) = -1*sin ( param * PI / 180.0 );
    // TransX.at<float>(2, 2) = sin ( param * PI / 180.0 );
    // TransX.at<float>(2, 3) = cos ( param * PI / 180.0 );

    // int rotate = 180;
    // TransZ.at<float>(0, 0) = cos ( rotate * PI / 180.0 );
    // TransZ.at<float>(0, 1) = -1*sin ( rotate * PI / 180.0 );
    // TransZ.at<float>(1, 0) = sin ( rotate * PI / 180.0 );
    // TransZ.at<float>(1, 1) = cos ( rotate * PI / 180.0 );

    // TransP = TransX*TransZ*TransP*CurrentPose;

    // if ( TransP.at<float>(0, 2) > 0 &&  TransP.at<float>(1, 2) < 0 )
    //     angularZ = angularZ ;

    // else if ( TransP.at<float>(0, 2) > 0 &&  TransP.at<float>(1, 2) > 0 )
    //     angularZ = 180 + angularZ ;

    // else if ( TransP.at<float>(0, 2) < 0 &&  TransP.at<float>(1, 2) < 0)
    //     angularZ = angularZ ;

    // else if ( TransP.at<float>(0, 2) < 0 &&  TransP.at<float>(1, 2) > 0 )
    //     angularZ = 180 + angularZ ;

    // angularZ = 90 + angularZ;


    /////   thesis    //////
    int rotate = 90;
    TransZ.at<float>(0, 0) = cos ( rotate * PI / 180.0 );
    TransZ.at<float>(0, 1) = -1*sin ( rotate * PI / 180.0 );
    TransZ.at<float>(1, 0) = sin ( rotate * PI / 180.0 );
    TransZ.at<float>(1, 1) = cos ( rotate * PI / 180.0 );

    TransP = TransZ*CurrentPose;

    float angularZ = atan( TransP.at<float>(1, 2)/TransP.at<float>(2, 2) ) * 180.0 / PI;

    if ( TransP.at<float>(1, 2) > 0 &&  TransP.at<float>(2, 2) > 0 )
        angularZ = angularZ ;

    else if ( TransP.at<float>(1, 2) > 0 &&  TransP.at<float>(2, 2) < 0 )
        angularZ = 180 + angularZ ;

    else if ( TransP.at<float>(1, 2) < 0 &&  TransP.at<float>(2, 2) < 0)
        angularZ = 180 + angularZ ;

    else if ( TransP.at<float>(1, 2) < 0 &&  TransP.at<float>(2, 2) > 0 )
        angularZ = angularZ ;

    if ( angularZ > 180 )
        angularZ -= 360;
    else if ( angularZ < -180 )
        angularZ += 360;

    // geometry_msgs::Twist msg;
    // msg.linear.x = TransP.at<float>(0, 3)*100;
    // msg.linear.y = TransP.at<float>(0, 7)*100;
    // msg.angular.z = angularZ;

    geometry_msgs::PointStamped msg;
    // msg.header.stamp = ros::Time::now();
    msg.header.stamp = current_lidar_time_;
    msg.point.x = TransP.at<float>(0, 3)*100;
    msg.point.y = TransP.at<float>(0, 7)*100;
    msg.point.z = angularZ;

    // save every previous poase untill client send the request
    PreviousPose.push_back(msg);

    // publish pose as topic
    // CamPose_pub.publish(msg);
 
    // pose.data = vec;
    // CamPose_pub.publish(pose);
}

bool ImageGrabber::SurvicePose(  all_process::CameraPose::Request  &req,
                                 all_process::CameraPose::Response &res)
{
    // std::cerr << "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz" << std::endl;
//     std::cerr << PreviousPose.size() << std::endl;
    // std::cerr << req.sec << "  " << req.nsec << std::endl;
    
    for ( int i=0; i<=PreviousPose.size(); i++)
        if (req.sec == PreviousPose[i].header.stamp.sec && req.nsec == PreviousPose[i].header.stamp.nsec)
        {
            // std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
            // std::cerr << PreviousPose[i].point.x << "  " << PreviousPose[i].point.y << "  " << PreviousPose[i].point.z << std::endl;
            res.x = PreviousPose[i].point.x;
            res.y = PreviousPose[i].point.y;
            res.z = PreviousPose[i].point.z;
            PreviousPose.clear();
            return true;
        }
    // if (PreviousPose.size() >= 1000)
    //     PreviousPose.clear();

    return false;   
}

// set publish message of member pose
void ImageGrabber::SetPose()
{
    // fill out message:
    pose.layout.dim.push_back(std_msgs::MultiArrayDimension());
    pose.layout.dim.push_back(std_msgs::MultiArrayDimension());
    pose.layout.dim[0].label = "height";
    pose.layout.dim[1].label = "width";
    pose.layout.dim[0].size = 4;
    pose.layout.dim[1].size = 4;
    pose.layout.dim[0].stride = 16;
    pose.layout.dim[1].stride = 4;
    pose.layout.data_offset = 0;
}


// void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::LaserScanConstPtr &msgSCAN)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;

    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    // get lidar stamp
    current_lidar_time_ = msgSCAN->header.stamp;               // last stamp 

    // get current data from tracker and keyframe
    callback();
    // publish point cloud and camera pose
    // PublishPointcloud();
    PublishCamPose();
}