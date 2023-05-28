/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Brian Gerkey */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"

#include <boost/thread.hpp>

// compute execute time
#include <chrono>
// IP-ICP will using this library
#include <csm/csm_all.h>
// Subscribe ORB pose
#include <geometry_msgs/PointStamped.h>
// add Camera service
#include "all_process/CameraPose.h"
// add Kill node Trigger service
#include "all_process/Trigger.h"
// use log
#include <fstream>

// use to read yaml
#include "yaml-cpp/yaml.h"

// use for MSE
#include <cmath>
#include <vector>

// use matplotlibcpp to draw trajectory
#include "matplotlibcpp.h"

// add UKF library
// #include "Eigen/Dense"
// #include "ukf.h"
// #include "ground_truth_package.h"
// #include "measurement_package.h"
#include "ukf.h"
#include "tools.h"

#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

// generate new folder
#include <sys/stat.h>
#include <sys/types.h>


namespace plt = matplotlibcpp;

class SlamGMapping
{
  public:
    SlamGMapping();
    SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    SlamGMapping(unsigned long int seed, unsigned long int max_duration_buffer);
    ~SlamGMapping();

    void init();
    void startLiveSlam();
    void startReplay(const std::string & bag_fname, std::string scan_topic);
    void publishTransform();
  
    // void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan, const nav_msgs::Odometry::ConstPtr& odom);

    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);
    void publishLoop(double transform_publish_period);

    ///////////////////////
    /// PLICP function ///
    ///////////////////////
    std::vector<double> a_cos_;             // for saving each cos value
    std::vector<double> a_sin_;             // for saving each sin value

    // **** keyframe params: when to generate the keyframe scan
    // if either is set to 0, reduces to frame-to-frame matching
    double kf_dist_linear_;
    double kf_dist_linear_sq_;
    double kf_dist_angular_;
    int kf_scan_count_;
    int scan_count_;

    geometry_msgs::Twist latest_velocity_;

    tf::Transform base_in_map_;           // base_link在map坐标系下的坐标
    tf::Transform base_in_map_keyframe_;  // base_link在map坐标系下的keyframe的坐标

    tf::Transform base_to_laser_;
    tf::Transform laser_to_base_;

    // csm
    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;

    // change method : 0 = gmapping, 1 = PLICP, 2 = PLICP+ORB
    int mymethod = 2;

    // initialize PLICP parameters
    void InitICPParams();
    // save the lidar angle, prevent to compute angle every time
    void CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    // initialize prev_ldp_scan_ and last_icp_time_
    void LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp);
    // use PLICP to compute transform between last frame and current frame
    void ScanMatchWithPLICP(LDP &curr_ldp_scan, const ros::Time &time);
    // using PLICP method
    bool ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, GMapping::OrientedPoint& gmap_pose);
    void GetPrediction(double &prediction_change_x, double &prediction_change_y, double &prediction_change_angle, double dt);
    void CreateTfFromXYTheta(double x, double y, double theta, tf::Transform& t);
    bool NewKeyframeNeeded(const tf::Transform &d);
    bool GetBaseToLaserTf(const std::string &frame_id);

    //

    // subscribe ORB pose
    ros::Subscriber sub;
    void SubsPose(const geometry_msgs::PointStamped::ConstPtr &pose);
    ros::ServiceClient CamPose_client;
    all_process::CameraPose srv;

    // define kill node trigger  service
    ros::ServiceServer Trigger_serv;


    // save last period pose of plicp
    // GMapping::OrientedPoint last_odom_pose;
    // GMapping::OrientedPoint last_ORB_pose;
    tf::Transform plicp_pose;
    tf::Transform base_to_laser;


    // set two Unscented-Kalman-Filter of ORB and PLICP
    UKF orb_ukf;
    UKF plicp_ukf;

    GMapping::OrientedPoint odom_last;
    // plicp last period predict by UKF
    // GMapping::OrientedPoint plicp_last_predict;


    // save each model's residual
    void SaveResidual(double curr_time);
    //this parameter control used how many recent data to compute hypothesis
    int residual_sum;

    // container to save recent residual value (size: residual_sum * 2)
    MatrixXd ORB_res;
    MatrixXd PLICP_res;

    // container to save all residual value
    std::vector<double> ORB_res_all_x;
    std::vector<double> ORB_res_all_y;
    std::vector<double> PLICP_res_all_x;
    std::vector<double> PLICP_res_all_y;
    std::vector<double> res_time_stamp;

    // the current counter of both residual saving matrix
    int count_res;
    // calculate the amount of the saving residual when the first time that matrix wasn't full
    int sum_res;

    // set pose calculate by Hypothesis Testing 
    GMapping::OrientedPoint hypothesis_pose;
    void HypothesisTesting();

    VectorXd ORB_weight;
    VectorXd PLICP_weight;
    
    // according to residual to adjust weight
    void AdjustWeight(VectorXd &a, VectorXd &b, int x);

    /////// compute the average and MSE of ORBSLAM2 & PLICP /////////////
    std::vector<double> AE_PLICP;             // for saving PLICP average error 
    std::vector<double> AE_ORBSLAM;           // for saving ORBSLAM2 average error

    // for saving PLICP MSE (without divide size)
    double MSE_PLICP_x = 0.0;
    double MSE_PLICP_y = 0.0;
    double MSE_PLICP_sum = 0.0;

    // for saving ORBSLAM2 MSE (without divide size)
    double MSE_ORBSLAM_x = 0.0;
    double MSE_ORBSLAM_y = 0.0;
    double MSE_ORBSLAM_sum = 0.0;

    /////// compute the average and MSE of ORBSLAM2 & PLICP /////////////
    std::vector<double> AE_UKF_ORB_odom;        // for saving UKF-ORBSLAM2(compare with odom) average error 
    std::vector<double> AE_UKF_ORB;             // for saving UKF-ORBSLAM2(compare with ORBSLAM2) average error

    double MSE_UKF_ORB_odom_x = 0.0;
    double MSE_UKF_ORB_odom_y = 0.0;
    double MSE_UKF_ORB_odom_sum = 0.0;

    double MSE_UKF_ORB_x = 0.0;
    double MSE_UKF_ORB_y = 0.0;
    double MSE_UKF_ORB_sum = 0.0;


    std::vector<double> AE_UKF_PLICP_odom;        // for saving UKF-PLICP(compare with odom) average error 
    std::vector<double> AE_UKF_PLICP;             // for saving UKF-PLICP(compare with PLICP) average error

    double MSE_UKF_PLICP_odom_x = 0.0;
    double MSE_UKF_PLICP_odom_y = 0.0;
    double MSE_UKF_PLICP_odom_sum = 0.0;

    double MSE_UKF_PLICP_x = 0.0;
    double MSE_UKF_PLICP_y = 0.0;
    double MSE_UKF_PLICP_sum = 0.0;

    std::vector<double> AE_best_odom;             // for saving UKF-PLICP(compare with PLICP) average error

    double MSE_best_odom_x = 0.0;
    double MSE_best_odom_y = 0.0;
    double MSE_best_odom_sum = 0.0;

    std::vector<double> trajectory_PLICP;             // for saving PLICP trajectory
    std::vector<double> trajectory_ORB;               // for saving ORB trajectory
    std::vector<double> trajectory_UKF_PLICP;        // for saving UKF of PLICP trajectory
    std::vector<double> trajectory_UKF_ORB;           // for saving UKF of ORB trajectory
    std::vector<double> trajectory_real;              // for saving odom trajectory
    std::vector<double> trajectory_best;              // for saving best trajectory

    // compute precision of PLICP
    void Precision_PLICP(const nav_msgs::Odometry::ConstPtr& odom, double x, double y);
    // compute precision of ORB
    void Precision_ORB(const nav_msgs::Odometry::ConstPtr& odom, double x, double y);

    // compute precision of PLICP in UKF
    void Precision_UKF_PLICP(const nav_msgs::Odometry::ConstPtr& odom, double x_, double y_, double x, double y);
    // compute precision of ORB in UKF
    void Precision_UKF_ORB(const nav_msgs::Odometry::ConstPtr& odom, double x_, double y_, double x, double y);

    // compute precision of best pose
    void Precision_Best_Pose(const nav_msgs::Odometry::ConstPtr& odom, double x, double y);

    // add KillTrigger function of master service for kill node trigger
    bool KillTrigger(   all_process::Trigger::Request  &req,
                        all_process::Trigger::Response &res);

    // add TerminateTrigger function when termicate command was trigger
    void TerminateTrigger();

    // in tuning process, set and save param in yaml
    bool SetORBParam();
    bool SetPLICPParam();

    bool SetUKFORBParam();
    bool SetUKFPLICPParam();

    bool SetHypothesisParam();

    void SavePosition(std::vector<double>& container, double x, double y, double current_time);
    std::pair<std::vector<double>, std::vector<double>> GetPoints(std::vector<double> container);
    void SaveTrajectoryGraph();

    void SaveTrajectoryGraph_(std::string path);
    void CheckFolder(std::string file_path);
    void SaveParam(std::string file_path);



    //define config path
    std::string  config_path = "/home/ron/work/src/all_process/config.yaml";

    // save the path of log , orb config and plicp config 
    std::string  log_path;
    std::string  plicp_path;
    std::string  orb_path;
    // bool flag_plicp_ukf = true;

    double first_sec;
    double first_nsec;
    
    
    bool first_time = true;
  private:
    ros::NodeHandle node_;
    ros::Publisher entropy_publisher_;
    ros::Publisher sst_;
    ros::Publisher sstm_;
    ros::ServiceServer ss_;
    tf::TransformListener tf_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    tf::TransformBroadcaster* tfB_;

    tf2_ros::Buffer tfBuffer_;
    // add Odometry subscriber
    message_filters::Subscriber<nav_msgs::Odometry>* odom_filter_sub_;


    // PLICP function
    ros::Time last_icp_time_;               // last stamp 
    ros::NodeHandle private_node_;          // ros private node
    // new thread to subscribe ORB pose 
    boost::thread* pose_thread_;
    // ros::NodeHandle tnode_;


    GMapping::GridSlamProcessor* gsp_;
    GMapping::RangeSensor* gsp_laser_;
    // The angles in the laser, going from -x to x (adjustment is made to get the laser between
    // symmetrical bounds as that's what gmapping expects)
    std::vector<double> laser_angles_;
    // The pose, in the original laser frame, of the corresponding centered laser with z facing up
    tf::Stamped<tf::Pose> centered_laser_pose_;
    // Depending on the order of the elements in the scan and the orientation of the scan frame,
    // We might need to change the order of the scan
    bool do_reverse_range_;
    unsigned int gsp_laser_beam_count_;
    GMapping::OdometrySensor* gsp_odom_;

    bool got_first_scan_;

    bool got_map_;
    nav_msgs::GetMap::Response map_;

    ros::Duration map_update_interval_;
    tf::Transform map_to_odom_;
    boost::mutex map_to_odom_mutex_;
    boost::mutex map_mutex_;

    int laser_count_;
    int throttle_scans_;

    boost::thread* transform_thread_;

    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    void updateMap(const sensor_msgs::LaserScan& scan);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);
    bool initMapper(const sensor_msgs::LaserScan& scan);
    bool addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose);
    double computePoseEntropy();
    


    void publishMap(const sensor_msgs::LaserScan& scan);

    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;
    
    ros::NodeHandle private_nh_;
    
    unsigned long int seed_;
    
    double transform_publish_period_;
    double tf_delay_;
};

