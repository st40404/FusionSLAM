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
/* Modified by: Charles DuHadway */


/**

@mainpage slam_gmapping

@htmlinclude manifest.html

@b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
scans and odometry and computes a map. This map can be
written to a file using e.g.

  "rosrun map_server map_saver static_map:=dynamic_map"

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner 
- @b "/tf": odometry from the robot


Publishes to (name/type):
- @b "/tf"/tf/tfMessage: position relative to the map


@section services
 - @b "~dynamic_map" : returns the map


@section parameters ROS parameters

Reads the following parameters from the parameter server

Parameters used by our GMapping wrapper:

- @b "~throttle_scans": @b [int] throw away every nth laser scan
- @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map


Parameters used by GMapping itself:

Laser Parameters:
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)
- @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)
- @b "~/kernelSize" @b [int] search window for the scan matching process
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [int] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)
- @b "~/minimumScore" @b [double] minimum score for considering the outcome of the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

Motion Model Parameters (all standard deviations of a gaussian noise model)
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

- @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

Likelihood sampling (used in scan matching)
- @b "~/llsamplerange" @b [double] linear range
- @b "~/lasamplerange" @b [double] linear step size
- @b "~/llsamplestep" @b [double] linear range
- @b "~/lasamplestep" @b [double] angular step size

Initial map dimensions and resolution:
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/

//////////////////////////////////////////////////////////////////////////////////////////////////////
// PLICP part : copy from github xiangli0608/Creating-2D-laser-slam-from-scratch (by xiangli0608)
// github : https://github.com/xiangli0608/Creating-2D-laser-slam-from-scratch.git
//////////////////////////////////////////////////////////////////////////////////////////////////////



#include "slam_gmapping.h"

#include <iostream>

#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0),node_(nh), private_nh_(pnh), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}

SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int max_duration_buffer):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL),
  seed_(seed), tf_(ros::Duration(max_duration_buffer))
{
  init();
}


void SlamGMapping::init()
{
  // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  // The library is pretty chatty
  gsp_ = new GMapping::GridSlamProcessor();
  ROS_ASSERT(gsp_);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  gsp_laser_ = NULL;
  gsp_odom_ = NULL;

  got_first_scan_ = false;
  got_map_ = false;
  

  
  // Parameters used by our GMapping wrapper
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  
  // Parameters used by GMapping itself
  maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
  if(!private_nh_.getParam("minimumScore", minimum_score_))
    minimum_score_ = 0;
  if(!private_nh_.getParam("sigma", sigma_))
    sigma_ = 0.05;
  if(!private_nh_.getParam("kernelSize", kernelSize_))
    kernelSize_ = 1;
  if(!private_nh_.getParam("lstep", lstep_))
    lstep_ = 0.05;
  if(!private_nh_.getParam("astep", astep_))
    astep_ = 0.05;
  if(!private_nh_.getParam("iterations", iterations_))
    iterations_ = 5;
  if(!private_nh_.getParam("lsigma", lsigma_))
    lsigma_ = 0.075;
  if(!private_nh_.getParam("ogain", ogain_))
    ogain_ = 3.0;
  if(!private_nh_.getParam("lskip", lskip_))
    lskip_ = 0;
  if(!private_nh_.getParam("srr", srr_))
    srr_ = 0.1;
  if(!private_nh_.getParam("srt", srt_))
    srt_ = 0.2;
  if(!private_nh_.getParam("str", str_))
    str_ = 0.1;
  if(!private_nh_.getParam("stt", stt_))
    stt_ = 0.2;
  if(!private_nh_.getParam("linearUpdate", linearUpdate_))
    linearUpdate_ = 1.0;
  if(!private_nh_.getParam("angularUpdate", angularUpdate_))
    angularUpdate_ = 0.5;
  if(!private_nh_.getParam("temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;
  if(!private_nh_.getParam("resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;
  if(!private_nh_.getParam("particles", particles_))
    particles_ = 30;
  if(!private_nh_.getParam("xmin", xmin_))
    xmin_ = -100.0;
  if(!private_nh_.getParam("ymin", ymin_))
    ymin_ = -100.0;
  if(!private_nh_.getParam("xmax", xmax_))
    xmax_ = 100.0;
  if(!private_nh_.getParam("ymax", ymax_))
    ymax_ = 100.0;
  if(!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;
  if(!private_nh_.getParam("occ_thresh", occ_thresh_))
    occ_thresh_ = 0.25;
  if(!private_nh_.getParam("llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;
  if(!private_nh_.getParam("llsamplestep", llsamplestep_))
    llsamplestep_ = 0.01;
  if(!private_nh_.getParam("lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;
  if(!private_nh_.getParam("lasamplestep", lasamplestep_))
    lasamplestep_ = 0.005;
    
  if(!private_nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;

}


void SlamGMapping::startLiveSlam()
{
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  
  // original gmapping registerCallback scan and tf
  // scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  // scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  // scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));
  

  // add subscribe odom for registerCallback
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  odom_filter_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, "odom", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy>* sync;
  sync= new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *scan_filter_,  *odom_filter_sub_ );
  sync->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1, _2));


  // define /trigger service master
  Trigger_serv = node_.advertiseService("/trigger", &SlamGMapping::KillTrigger, this);

  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));

  if (mymethod == 0)
    ROS_INFO("Using method : Gmapping");
  else if (mymethod == 1)
    ROS_INFO("Using method PLICP");
  // subscribe ORB pose
  else if (mymethod == 2)
  {
    ROS_INFO("Using method : PLICP + ORB");
    // using service (client)
    CamPose_client = node_.serviceClient<all_process::CameraPose>("/ORB/pose");

    // using subscribe
    // sub = node_.subscribe("/ORB/camera_pose", 5, &SlamGMapping::SubsPose, this);
  }
}

void SlamGMapping::startReplay(const std::string & bag_fname, std::string scan_topic)
{
  double transform_publish_period;
  ros::NodeHandle private_nh_("~");
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  
  rosbag::Bag bag;
  bag.open(bag_fname, rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string("/tf"));
  topics.push_back(scan_topic);
  rosbag::View viewall(bag, rosbag::TopicQuery(topics));

  // Store up to 5 messages and there error message (if they cannot be processed right away)
  std::queue<std::pair<sensor_msgs::LaserScan::ConstPtr, std::string> > s_queue;
  foreach(rosbag::MessageInstance const m, viewall)
  {
    tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
    if (cur_tf != NULL) {
      for (size_t i = 0; i < cur_tf->transforms.size(); ++i)
      {
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform stampedTf;
        transformStamped = cur_tf->transforms[i];
        tf::transformStampedMsgToTF(transformStamped, stampedTf);
        tf_.setTransform(stampedTf);
      }
    }

    sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
    if (s != NULL) {
      if (!(ros::Time(s->header.stamp)).is_zero())
      {
        s_queue.push(std::make_pair(s, ""));
      }
      // Just like in live processing, only process the latest 5 scans
      if (s_queue.size() > 5) {
        ROS_WARN_STREAM("Dropping old scan: " << s_queue.front().second);
        s_queue.pop();
      }
      // ignoring un-timestamped tf data 
    }

    // Only process a scan if it has tf data
    while (!s_queue.empty())
    {
      try
      {
        tf::StampedTransform t;
        tf_.lookupTransform(s_queue.front().first->header.frame_id, odom_frame_, s_queue.front().first->header.stamp, t);
        // original code have this part, but I change laserCallback to need two input as follow code
        // this->laserCallback(s_queue.front().first, testing);

        // I gave the fake nav_msgs::Odometry to laser Callback function
        const nav_msgs::Odometry::ConstPtr testing;
        this->laserCallback(s_queue.front().first, testing);

        s_queue.pop();
      }
      // If tf does not have the data yet
      catch(tf2::TransformException& e)
      {
        // Store the error to display it if we cannot process the data after some time
        s_queue.front().second = std::string(e.what());
        break;
      }
    }
  }

  bag.close();
}

void SlamGMapping::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    publishTransform();
    r.sleep();
  }
}


void SlamGMapping::SubsPose(const geometry_msgs::PointStamped::ConstPtr &pose){
  std::cerr << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
  // std::cerr << pose->header << std::endl;
  std::cerr << pose->header.stamp.sec << "  " << pose->header.stamp.nsec << std::endl;
  // std::cerr << pose->header.stamp.nsec << std::endl;
  // std::cerr << pose->point.x << std::endl;
  // std::cerr << pose->point.y << std::endl;
  // std::cerr << pose->point.z << std::endl;
}



SlamGMapping::~SlamGMapping()
{
  if(transform_thread_){
    transform_thread_->join();
    delete transform_thread_;
  }

  delete gsp_;
  if(gsp_laser_)
    delete gsp_laser_;
  if(gsp_odom_)
    delete gsp_odom_;
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

bool
SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  // odom_pose作為odom座標系下的lidar姿態,主要透過centered_laser_pose_從scan座標系轉換至odom座標系所獲得,
  // 而當我們透過teleop來控制機器人時, remote(機器人驅動)會更改機器人於Gazwbo中的位置, 從而更改centered_laser_pose_的位置

  // Get the pose of the centered laser at the current time
  centered_laser_pose_.stamp_ = t;
  // Get the laser's pose that is centered
  tf::Stamped<tf::Transform> odom_pose;

  try
  {
    tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}

bool
SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan)
{
  laser_frame_ = scan.header.frame_id;
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;

  try
  {
    tf_.transformPose(base_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  // create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                                      base_frame_);
  try
  {
    tf_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }
  
  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 up.z());
    return false;
  }

  gsp_laser_beam_count_ = scan.ranges.size();
  double angle_center = (scan.angle_min + scan.angle_max)/2;

  if (up.z() > 0)
  {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upwards.");
  }
  else
  {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }

  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  laser_angles_.resize(scan.ranges.size());
  // Make sure angles are started so that they are centered
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
  for(unsigned int i=0; i<scan.ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);
  }

  ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
            scan.angle_increment);
  ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan.angle_increment));

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // setting maxRange and maxUrange here so we can set a reasonable default
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;
  if(!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_;

  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,
                                         fabs(scan.angle_increment),
                                         gmap_pose,
                                         0.0,
                                         maxRange_);
  ROS_ASSERT(gsp_laser_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);


  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
  {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }

  //////////////////////////////   adding    //////////////////////////////
  // save path of log, PLICP config and ORB config
  YAML::Node config = YAML::LoadFile(config_path);
  log_path   = config["log_path"].as<std::string>();
  plicp_path = config["plicp_config"].as<std::string>();
  orb_path   = config["orb_config"].as<std::string>();

  // need to give plicp_pose a initialpose by laser pose 
  plicp_pose.setOrigin( tf::Vector3(initialPose.x, initialPose.y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, initialPose.theta);
  plicp_pose.setRotation(q);

  base_in_map_ = plicp_pose;
  base_in_map_keyframe_ = plicp_pose;

  // use for PLICP first guass by odometry
  odom_last.x = initialPose.x;
  odom_last.y = initialPose.y;
  /////////////////////////////////////////////////////////////////////////


  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1,seed_);

  ROS_INFO("Initialization complete");
  return true;
}

bool
SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
  if(!getOdomPose(gmap_pose, scan.header.stamp))
     return false;

  if(scan.ranges.size() != gsp_laser_beam_count_)
    return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  if (do_reverse_range_)
  {
    ROS_DEBUG("Inverting scan");
    int num_ranges = scan.ranges.size();
    for(int i=0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[num_ranges - i - 1] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  } else 
  {
    for(unsigned int i=0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);

  /*
  ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
            scan.header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */
  ROS_DEBUG("processing scan");

  // compute odom by gmapping and return
  return gsp_->processScan(reading);
}

bool
SlamGMapping::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, GMapping::OrientedPoint& gmap_pose)
{
  if(!getOdomPose(gmap_pose, scan->header.stamp))
     return false;

  if(scan->ranges.size() != gsp_laser_beam_count_)
    return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan->ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  if (do_reverse_range_)
  {
    ROS_DEBUG("Inverting scan");
    int num_ranges = scan->ranges.size();
    for(int i=0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan->ranges[num_ranges - i - 1] < scan->range_min)
        ranges_double[i] = (double)scan->range_max;
      else
        ranges_double[i] = (double)scan->ranges[num_ranges - i - 1];
    }
  } else 
  {
    for(unsigned int i=0; i < scan->ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan->ranges[i] < scan->range_min)
        ranges_double[i] = (double)scan->range_max;
      else
        ranges_double[i] = (double)scan->ranges[i];
    }
  }

  GMapping::RangeReading reading(scan->ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 scan->header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  /*
  ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
            scan.header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */
  ROS_DEBUG("processing scan");


  odom_last.x = gmap_pose.x - odom_last.x;
  odom_last.y = gmap_pose.y - odom_last.y;

  // using PLICP compute
  LDP curr_ldp_scan;
  // change scan type transform to csm type
  LaserScanToLDP(scan, curr_ldp_scan);
  // compute PL-ICP compute the transform
  ScanMatchWithPLICP(curr_ldp_scan, scan->header.stamp);

  odom_last.x = gmap_pose.x;
  odom_last.y = gmap_pose.y;

  // std::cerr << output_.x[0] << "  " << output_.x[1] << std::endl;

  // compute PLICP current position, and save in plicp_pose
  // tf::Transform plicp_trans;
  // plicp_trans.setOrigin( tf::Vector3(-1*output_.x[0], -1*output_.x[1], 0.0) );
  // tf::Quaternion q;
  // q.setRPY(0.0, 0.0, output_.x[2]);
  // plicp_trans.setRotation(q);
  // plicp_pose = plicp_pose * plicp_trans;


  GMapping::OrientedPoint icp_pose( base_in_map_.getOrigin().getX(),
                                    base_in_map_.getOrigin().getY(),
                                    tf::getYaw(base_in_map_.getRotation()));

  // save pose and scan reading
  reading.setPose(icp_pose);

  return gsp_->mprocessScan(reading);
}

// void
// SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
void SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan, const nav_msgs::Odometry::ConstPtr& odom)
{
  // std::cerr <<  "sssssssssssssssssssssssssssssssssss" << std::endl;
  // std::cerr << scan->header.stamp.sec << "  " << scan->header.stamp.nsec << std::endl;
  // std::cerr <<  "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;

  // // TESTING PLICP
  // LDP curr_ldp_scan;
  // // change scan type transform to csm type
  // LaserScanToLDP(scan, curr_ldp_scan);
  // // compute PL-ICP compute the transform
  // ScanMatchWithPLICP(curr_ldp_scan, scan->header.stamp);

  // std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
  // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
  // std::cerr << "\naaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa: " << time_used.count() << " s " << std::endl;
  // std::cerr << output_.x[0] << "  " << output_.x[1] << "  " << output_.x[2] * 180 / M_PI << std::endl;

  ///////    Gmapping
  if (mymethod == 0)
  {
    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
      return;

    static ros::Time last_map_update(0,0);

    // We can't initialize the mapper until we've got the first scan
    if(!got_first_scan_)
    {
      if(!initMapper(*scan))
        return;

      last_icp_time_ = scan->header.stamp;
      got_first_scan_ = true;
    }

    // odom_pose : laser pose in odom frame
    // MAYBE : odom_pose is computed by motor motion
    GMapping::OrientedPoint odom_pose;


    if(addScan(*scan, odom_pose))
    {
      // std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
      // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
      // std::cout << "\naaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa: " << time_used.count() << " s " << std::endl;
      // std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

      ROS_DEBUG("scan processed");
      GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;


      std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
      std::cerr << mpose.x << "  " << mpose.y << "  " << mpose.theta*180/M_PI << std::endl;
      std::cerr << odom_pose.x << "  " << odom_pose.y << "  " << odom_pose.theta*180/M_PI << std::endl;
      // std::cerr << odom->pose.pose.position.x << "  " << odom->pose.pose.position.y << " " << odom->pose.pose.position.z << std::endl;


      ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
      ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
      ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

      // laser_to_map : lidar pose in map frame
      // odom_to_laser : lidar pose in odom frame
      // tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0)).inverse();

      tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
      tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

      // map_to_odom_ : compute relationship between odom frame and map frame
      map_to_odom_mutex_.lock();
      map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
      map_to_odom_mutex_.unlock();

      // map update will follow map_update_interval_ this parameter
      if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
      {
        updateMap(*scan);
        last_map_update = scan->header.stamp;
        ROS_DEBUG("Updated the map");
      }
    } else
      ROS_DEBUG("cannot process scan");

    // std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    // std::cout << "\naaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa: " << time_used.count() << " s " << std::endl;
  }

///////    PLICP
  else if (mymethod == 1)
  {
    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
      return;

    static ros::Time last_map_update(0,0);

    // We can't initialize the mapper until we've got the first scan
    if(!got_first_scan_)
    {
      if(!initMapper(*scan))
        return;

      // initialize csm param
      InitICPParams();
      CreateCache(scan);
      LaserScanToLDP(scan, prev_ldp_scan_);

      if (!GetBaseToLaserTf(scan->header.frame_id))
      {
          ROS_WARN("Skipping scan");
          return;
      }

      // last_odom_pose.x = 0;
      // last_odom_pose.y = 0;
      // last_odom_pose.theta = 0;


      input_.laser[0] = 0.0;
      input_.laser[1] = 0.0;
      input_.laser[2] = 0.0;

      // Initialize output_ vectors as Null for error-checking
      output_.cov_x_m = 0;
      output_.dx_dy1_m = 0;
      output_.dx_dy2_m = 0;

      last_icp_time_ = scan->header.stamp;
      got_first_scan_ = true;
    }

    // odom_pose : laser pose in odom frame
    // MAYBE : odom_pose is computed by motor motion
    GMapping::OrientedPoint odom_pose;

    if (ScanCallback(scan, odom_pose))
    {
      // std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
      // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
      // std::cout << "\naaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa: " << time_used.count() << " s " << std::endl;
      // std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

      ROS_DEBUG("scan processed");

      // std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
      // std::cerr << odom->pose.pose.position.x << " " << odom->pose.pose.position.y << " " << -2 * acos(odom->pose.pose.orientation.w) / 3.14159 * 180.0 << std::endl;

      // std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
      // std::cerr << odom_pose.x << "  " << odom_pose.y << "  " << odom_pose.theta*180/M_PI << std::endl;
      // std::cerr << output_.x[0] << "  " << output_.x[1] << "  " << output_.x[2]*180/M_PI << std::endl;
      // std::cerr <<  base_in_map_.getOrigin().getX() << " " << base_in_map_.getOrigin().getY() << " " << tf::getYaw(base_in_map_.getRotation())*180/M_PI <<std::endl;

      Precision_PLICP(odom, base_in_map_.getOrigin().getX(), base_in_map_.getOrigin().getY());

      ROS_DEBUG("new best pose: %.3f %.3f %.3f", base_in_map_.getOrigin().getX(), base_in_map_.getOrigin().getY(), tf::getYaw(base_in_map_.getRotation()));
      ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
      ROS_DEBUG("correction: %.3f %.3f %.3f", output_.x[0], output_.x[1], output_.x[2]);


      // laser_to_map : lidar pose in map frame
      // odom_to_laser : lidar pose in odom frame
      // tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, tf::getYaw(plicp_pose.getRotation())), tf::Vector3(plicp_pose.getOrigin().getX(), plicp_pose.getOrigin().getY(), 0.0)).inverse();
      tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, tf::getYaw(base_in_map_.getRotation())), tf::Vector3(base_in_map_.getOrigin().getX(), base_in_map_.getOrigin().getY(), 0.0)).inverse();
      tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

      // map_to_odom_ : compute relationship between odom frame and map frame
      map_to_odom_mutex_.lock();
      map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
      map_to_odom_mutex_.unlock();

      // last_odom_pose = odom_pose;
      
      // map update will follow map_update_interval_ this parameter
      if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
      {
        // std::cerr << "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz" << std::endl;
        publishMap(*scan);
        last_map_update = scan->header.stamp;
        ROS_DEBUG("Updated the map");
      }
    } else
      ROS_DEBUG("cannot process scan");
  } 

  // std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
  // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
  // std::cout << "\naaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa: " << time_used.count() << " s " << std::endl;
  
///////    PLICP + ORB
  else if (mymethod == 2)
  {
    srv.request.sec = scan->header.stamp.sec;
    srv.request.nsec = scan->header.stamp.nsec;

    // if the pose request with same stamp between camera and lidar was accept
    if (CamPose_client.call(srv))
    {
      // ROS_INFO("Get pose : x : %lf, y : %lf, z : %lf", srv.response.x, srv.response.y, srv.response.z);

      laser_count_++;
      if ((laser_count_ % throttle_scans_) != 0)
        return;

      static ros::Time last_map_update(0,0);

      // We can't initialize the mapper until we've got the first scan
      if(!got_first_scan_)
      {
        if(!initMapper(*scan))
          return;

        // initialize csm param
        InitICPParams();
        CreateCache(scan);
        LaserScanToLDP(scan, prev_ldp_scan_);

        // plicp_last_predict.x = 0.0;
        // plicp_last_predict.y = 0.0;


        input_.laser[0] = 0.0;
        input_.laser[1] = 0.0;
        input_.laser[2] = 0.0;

        // Initialize output_ vectors as Null for error-checking
        output_.cov_x_m = 0;
        output_.dx_dy1_m = 0;
        output_.dx_dy2_m = 0;

        last_icp_time_ = scan->header.stamp;

        // UKF params
        MeasurementPackage meas_init_pose;
        meas_init_pose.raw_measurements_ = VectorXd(2);
        meas_init_pose.raw_measurements_ << 0, 0;

        // meas_init_pose.raw_measurements_ = VectorXd(3);
        // meas_init_pose.raw_measurements_ << 0, 0, 0;
        meas_init_pose.timestamp_ = srv.request.sec * 1000000000 + srv.request.nsec;
        meas_init_pose.sensor_type_ = MeasurementPackage::LASER;

        // init ORBSLAM2 and PLICP parameter
        orb_ukf.SetUKFParam(1, log_path);
        plicp_ukf.SetUKFParam(2, log_path);

        got_first_scan_ = orb_ukf.ProcessMeasurement(meas_init_pose);
        got_first_scan_ = plicp_ukf.ProcessMeasurement(meas_init_pose);

        first_sec =  srv.request.sec;
        first_nsec = srv.request.nsec;

        got_first_scan_ = true;
      }

      // odom_pose : laser pose in odom frame
      // MAYBE : odom_pose is computed by motor motion
      GMapping::OrientedPoint odom_pose;

      // to save the best variety of Agv 
      GMapping::OrientedPoint best_pose;

      // save ORB and PLICP pose, rotate, timestamp
      MeasurementPackage meas_ORB_pose;
      MeasurementPackage meas_PLICP_pose;

      if (ScanCallback(scan, odom_pose))
      {
        // std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
        // std::cout << "\naaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa: " << time_used.count() << " s " << std::endl;
        // std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        ROS_DEBUG("scan processed");
 

        Precision_ORB(odom, srv.response.x, srv.response.y);
        Precision_PLICP(odom, base_in_map_.getOrigin().getX(), base_in_map_.getOrigin().getY());


        // put ORB pose variety and PLICP pose variety into Unscented Kalman Filter
        // ORB pose part
        meas_ORB_pose.raw_measurements_ = VectorXd(2);
        meas_ORB_pose.raw_measurements_ << srv.response.x, srv.response.y;
        meas_ORB_pose.timestamp_ = srv.request.sec * 1000000000 + srv.request.nsec;
        meas_ORB_pose.sensor_type_ = MeasurementPackage::LASER;

        // PLICP pose part
        meas_PLICP_pose.raw_measurements_ = VectorXd(2);
        meas_PLICP_pose.raw_measurements_ << base_in_map_.getOrigin().getX(), base_in_map_.getOrigin().getY();
        meas_PLICP_pose.timestamp_ = srv.request.sec * 1000000000 + srv.request.nsec;
        meas_PLICP_pose.sensor_type_ = MeasurementPackage::LASER;

        bool flag_orb, flag_plicp;
        flag_orb = false;
        flag_plicp = false;

        // plicp_last_predict.x = plicp_ukf.x_[0];
        // plicp_last_predict.y = plicp_ukf.x_[1];

        flag_orb = orb_ukf.ProcessMeasurement(meas_ORB_pose);
        flag_plicp = plicp_ukf.ProcessMeasurement(meas_PLICP_pose);

        Precision_UKF_ORB(odom, srv.response.x, srv.response.y, orb_ukf.x_[0], orb_ukf.x_[1]);
        Precision_UKF_PLICP(odom, base_in_map_.getOrigin().getX(), base_in_map_.getOrigin().getY(), plicp_ukf.x_[0], plicp_ukf.x_[1]);

        if (flag_orb == true && flag_plicp == true)
        {
          // save the recent amount (param: residual_sum) of residual
          SaveResidual();
          HypothesisTesting();

          // std::cerr << ORB_weight(0) << "  " << PLICP_weight(0) << std::endl;
          // std::cerr << ORB_weight(1) << "  " << PLICP_weight(1) << std::endl;

        }
        
        // if ( flag_plicp_ukf == true)
        // {
        //   plicp_last_predict.x = plicp_ukf.x_[0];
        //   plicp_last_predict.y = plicp_ukf.x_[1];
        //   flag_plicp_ukf == false;
        // }
        // std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
        // std::cerr << odom->pose.pose.position.x << "  " << odom->pose.pose.position.y << " " << odom->pose.pose.position.z << std::endl;
        // std::cerr << plicp_pose.getOrigin().getX() << "  " << plicp_pose.getOrigin().getY() << std::endl;
        // std::cerr << plicp_ukf.x_[0] << "  " << plicp_ukf.x_[1] << std::endl;
        // std::cerr << srv.response.x << "  " << srv.response.y << " " << srv.response.z << std::endl;
        // std::cerr << orb_ukf.x_[0] << "  " << orb_ukf.x_[1] << std::endl;
        
        // std::cerr << odom_pose.x << "  " << odom_pose.y << " " << odom_pose.theta << std::endl;



        // best_pose.x = plicp_pose.getOrigin().getX() + ORB_weight(0)*orb_ukf.x_[0] + PLICP_weight(0)*plicp_ukf.x_[0];
        // best_pose.y = plicp_pose.getOrigin().getY() + ORB_weight(1)*orb_ukf.x_[1] + PLICP_weight(1)*plicp_ukf.x_[1];

        // origin PLICP and ORMSLAM2 method
        // best_pose.x = 0.5 * plicp_pose.getOrigin().getX() + 0.5 * srv.response.x;
        // best_pose.y = 0.5 *  plicp_pose.getOrigin().getY() + 0.5 * srv.response.y;
        best_pose.x = 0.5 * plicp_ukf.x_[0] + 0.5 * orb_ukf.x_[0];
        best_pose.y = 0.5 * plicp_ukf.x_[1] + 0.5 * orb_ukf.x_[1];



        // best_pose.x = ORB_weight(0)*orb_ukf.x_[0] + PLICP_weight(0)*plicp_ukf.x_[0];
        // best_pose.y = ORB_weight(1)*orb_ukf.x_[1] + PLICP_weight(1)*plicp_ukf.x_[1];
        best_pose.theta = odom_pose.theta;
        // best_pose.theta = last_odom_pose.theta + ORB_weight(2)*orb_ukf.x_[2] + PLICP_weight(2)*plicp_ukf.x_[2];


        // first_sec =  srv.request.sec;
        // first_nsec = std::cerr << srv.request.nsec;

        // std::cerr << (first_sec + first_nsec/1000000000) << std::endl;
        double now_sec = srv.request.sec;
        double now_nsec = srv.request.nsec;
        double current_time = now_sec + now_nsec/1000000000 - (first_sec + first_nsec/1000000000);
        // std::cerr << (now_sec + now_nsec/1000000000) << std::endl;

        // save trajectory
        SavePosition(trajectory_PLICP, base_in_map_.getOrigin().getX(), base_in_map_.getOrigin().getY(), current_time);
        SavePosition(trajectory_ORB, srv.response.x, srv.response.y, current_time);
        SavePosition(trajectory_UKF_PLICP, plicp_ukf.x_[0], plicp_ukf.x_[1], current_time);
        SavePosition(trajectory_UKF_ORB, orb_ukf.x_[0], orb_ukf.x_[1], current_time);
        SavePosition(trajectory_real, odom->pose.pose.position.x, odom->pose.pose.position.y, current_time);
        SavePosition(trajectory_best, best_pose.x, best_pose.y, current_time);


        Precision_Best_Pose(odom, best_pose.x, best_pose.y);

        ROS_DEBUG("new best pose: %.3f %.3f %.3f", best_pose.x, best_pose.y, odom_pose.theta);
        ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
        ROS_DEBUG("correction: %.3f %.3f %.3f", output_.x[0], output_.x[1], output_.x[2]);

        // laser_to_map : lidar pose in map frame
        // odom_to_laser : lidar pose in odom frame
        tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, tf::getYaw(base_in_map_.getRotation())), tf::Vector3(base_in_map_.getOrigin().getX(), base_in_map_.getOrigin().getY(), 0.0)).inverse();
        // tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, best_pose.theta), tf::Vector3(best_pose.x, best_pose.y, 0.0)).inverse();
        tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

        // map_to_odom_ : compute relationship between odom frame and map frame
        map_to_odom_mutex_.lock();
        map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
        map_to_odom_mutex_.unlock();

        // map update will follow map_update_interval_ this parameter
        if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
        {
          // std::cerr << "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz" << std::endl;
          publishMap(*scan);
          last_map_update = scan->header.stamp;
          ROS_DEBUG("Updated the map");
        }
      } else
        ROS_DEBUG("cannot process scan");
    } 
    // else
    //   ROS_ERROR("Failed to call %d sec and %d nsec scan topic", (long int)srv.request.sec, (long int)srv.request.nsec);
  }
}

double
SlamGMapping::computePoseEntropy()
{
  double weight_total=0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}

void
SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
  ROS_DEBUG("Update map");
  boost::mutex::scoped_lock map_lock (map_mutex_);
  GMapping::ScanMatcher matcher;

  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  // entropy (熵) : 機器人的精度(姿態分佈的估計值),值越高代表較高的不確定性
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);

  if(!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  GMapping::Point center;
  center.x=(xmin_ + xmax_) / 2.0;
  center.y=(ymin_ + ymax_) / 2.0;

  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, 
                                delta_);

  ROS_DEBUG("Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    ROS_DEBUG("  %.3f %.3f %.3f",
              n->pose.x,
              n->pose.y,
              n->pose.theta);

    if(!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  // the map may have expanded, so resize ros message as well
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;
    
    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ > occ_thresh_)
      {
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}

bool 
SlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock map_lock (map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

// map_to_odom_ : transform between map frame and odom frame 
// publish TF which transform from map to odom
void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}

// save the lidar angle, prevent to compute angle every time
void SlamGMapping::CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    a_cos_.clear();
    a_sin_.clear();
    double angle;

    for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
    {
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        a_cos_.push_back(cos(angle));
        a_sin_.push_back(sin(angle));
    }

    input_.min_reading = scan_msg->range_min;
    input_.max_reading = scan_msg->range_max;
}

// change scan type transform to csm type
void SlamGMapping::LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp)
{
    unsigned int n = scan_msg->ranges.size();
    // 调用csm里的函数进行申请空间
    ldp = ld_alloc_new(n);

    for (unsigned int i = 0; i < n; i++)
    {
        // calculate position in laser frame
        double r = scan_msg->ranges[i];

        if (r > scan_msg->range_min && r < scan_msg->range_max)
        {
            // 填充雷达数据
            ldp->valid[i] = 1;
            ldp->readings[i] = r;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1; // for invalid range
        }

        ldp->theta[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
        ldp->cluster[i] = -1;
    }

    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[n - 1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    // ldp->estimate[0] = 0.0;
    // ldp->estimate[1] = 0.0;
    // ldp->estimate[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}

// initialize PL-ICP parameters
// http://wiki.ros.org/canonical_scan_matcher
void SlamGMapping::InitICPParams()
{
    // **** CSM 的参数 - comments copied from algos.h (by Andrea Censi)

    // Maximum angular displacement between scans
    if (!private_node_.getParam("max_angular_correction_deg", input_.max_angular_correction_deg))
        // input_.max_angular_correction_deg = 45.0;
        input_.max_angular_correction_deg = 360.0;

    // Maximum translation between scans (m)
    // according to lidar, if lidar have more accuracy, the value of max_linear_correction need to be lower
    if (!private_node_.getParam("max_linear_correction", input_.max_linear_correction))
        input_.max_linear_correction = 0.5;

    // Maximum ICP cycle iterations
    if (!private_node_.getParam("max_iterations", input_.max_iterations))
        input_.max_iterations = 10;

    // epsilon_xy and epsilon_theta
    // when we finding the corresponding point
    // when the point xy distance / angle lower than the value
    // and this point maybe can be the correspond point

    // A threshold for stopping (m)
    if (!private_node_.getParam("epsilon_xy", input_.epsilon_xy))
        // input_.epsilon_xy = 0.001;
        input_.epsilon_xy = 0.000001;

    // A threshold for stopping (rad)
    if (!private_node_.getParam("epsilon_theta", input_.epsilon_theta))
        // input_.epsilon_theta = 0.00872;
        input_.epsilon_theta = 0.000001;

    // Maximum distance for a correspondence to be valid
    // when the scan point finding the correspondence point, this param is to control the finding range 
    // if the finding range too big, it will effect the accuracy
    // if the finding range too small, maybe it can't find the point
    if (!private_node_.getParam("max_correspondence_dist", input_.max_correspondence_dist))
        input_.max_correspondence_dist = 0.5;
        // input_.max_correspondence_dist = 1.0;

    // Noise in the scan (m)
    if (!private_node_.getParam("sigma", input_.sigma))
        input_.sigma = 0.010;

    // Use smart tricks for finding correspondences.
    // if value of use_corr_tricks is 1, means use smart tricks to find correspondences (effect spead)
    if (!private_node_.getParam("use_corr_tricks", input_.use_corr_tricks))
        input_.use_corr_tricks = 1;

    // Restart: Restart if error is over threshold
    // Restart: If 1, restart if error is over threshold
    if (!private_node_.getParam("restart", input_.restart))
        input_.restart = 0;

    // Restart: Threshold for restarting
    if (!private_node_.getParam("restart_threshold_mean_error", input_.restart_threshold_mean_error))
        input_.restart_threshold_mean_error = 0.01;

    // Restart: displacement for restarting. (m)
    if (!private_node_.getParam("restart_dt", input_.restart_dt))
        input_.restart_dt = 1.0;

    // Restart: displacement for restarting. (rad)
    if (!private_node_.getParam("restart_dtheta", input_.restart_dtheta))
        input_.restart_dtheta = 0.1;

    // Max distance for staying in the same clustering
    if (!private_node_.getParam("clustering_threshold", input_.clustering_threshold))
        input_.clustering_threshold = 0.25;

    // Number of neighbour rays used to estimate the orientation
    if (!private_node_.getParam("orientation_neighbourhood", input_.orientation_neighbourhood))
        input_.orientation_neighbourhood = 10;
        // input_.orientation_neighbourhood = 20;

    // If 0, it's vanilla ICP
    if (!private_node_.getParam("use_point_to_line_distance", input_.use_point_to_line_distance))
        input_.use_point_to_line_distance = 1;

    // Discard correspondences based on the angles
    // If 1, discard correspondences based on the angles
    if (!private_node_.getParam("do_alpha_test", input_.do_alpha_test))
        input_.do_alpha_test = 0;

    // Discard correspondences based on the angles - threshold angle, in degrees
    if (!private_node_.getParam("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
        input_.do_alpha_test_thresholdDeg = 20.0;

    // Percentage of correspondences to consider: if 0.9,
    // always discard the top 10% of correspondences with more error
    if (!private_node_.getParam("outliers_maxPerc", input_.outliers_maxPerc))
        input_.outliers_maxPerc = 0.90;

    // Parameters describing a simple adaptive algorithm for discarding.
    //  1) Order the errors.
    //  2) Choose the percentile according to outliers_adaptive_order.
    //     (if it is 0.7, get the 70% percentile)
    //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
    //     with the value of the error at the chosen percentile.
    //  4) Discard correspondences over the threshold.
    //  This is useful to be conservative; yet remove the biggest errors.
    if (!private_node_.getParam("outliers_adaptive_order", input_.outliers_adaptive_order))
        input_.outliers_adaptive_order = 0.7;

    if (!private_node_.getParam("outliers_adaptive_mult", input_.outliers_adaptive_mult))
        input_.outliers_adaptive_mult = 2.0;

    // If you already have a guess of the solution, you can compute the polar angle
    // of the points of one scan in the new position. If the polar angle is not a monotone
    // function of the readings index, it means that the surface is not visible in the
    // next position. If it is not visible, then we don't use it for matching.
    if (!private_node_.getParam("do_visibility_test", input_.do_visibility_test))
        input_.do_visibility_test = 0;

    // no two points in laser_sens can have the same corr.
    if (!private_node_.getParam("outliers_remove_doubles", input_.outliers_remove_doubles))
        input_.outliers_remove_doubles = 1;

    // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
    if (!private_node_.getParam("do_compute_covariance", input_.do_compute_covariance))
        input_.do_compute_covariance = 0;

    // If 1, Checks that find_correspondences_tricks gives the right answer
    if (!private_node_.getParam("debug_verify_tricks", input_.debug_verify_tricks))
        input_.debug_verify_tricks = 0;

    // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
    // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
    if (!private_node_.getParam("use_ml_weights", input_.use_ml_weights))
        input_.use_ml_weights = 0;

    // If 1, the field 'readings_sigma' in the second scan is used to weight the
    // correspondence by 1/sigma^2
    if (!private_node_.getParam("use_sigma_weights", input_.use_sigma_weights))
        input_.use_sigma_weights = 0;
    
    // /////// mapping param /////////
    // if (!private_node_.getParam("maxRange", max_range_))
    //     max_range_ = 30 - 0.01;
    // if (!private_node_.getParam("maxUrange", max_use_range_))
    //     max_use_range_ = 25;

    // if (!private_node_.getParam("xmin", xmin_))
    //     xmin_ = -40.0;
    // if (!private_node_.getParam("ymin", ymin_))
    //     ymin_ = -40.0;
    // if (!private_node_.getParam("xmax", xmax_))
    //     xmax_ = 40.0;
    // if (!private_node_.getParam("ymax", ymax_))
    //     ymax_ = 40.0;
    // if (!private_node_.getParam("delta", resolution_))
    //     resolution_ = 0.05;
    // if (!private_node_.getParam("occ_thresh", occ_thresh_))
    //     occ_thresh_ = 0.25;

    YAML::Node plicp_config = YAML::LoadFile(plicp_path);

    input_.max_angular_correction_deg = plicp_config["max_angular_correction_deg"].as<float>();
    input_.max_linear_correction      = plicp_config["max_linear_correction"].as<float>();
    input_.max_iterations             = plicp_config["max_iterations"].as<int>();
    input_.epsilon_xy                 = plicp_config["epsilon_xy"].as<float>();
    input_.epsilon_theta              = plicp_config["epsilon_theta"].as<float>();
    input_.max_correspondence_dist    = plicp_config["max_correspondence_dist"].as<float>();
    input_.outliers_maxPerc           = plicp_config["outliers_maxPerc"].as<float>();

    // kf_dist_linear_    = plicp_config["kf_dist_linear"].as<double>();
    // kf_dist_angular_   = plicp_config["kf_dist_angular"].as<double>();
    // kf_scan_count_     = plicp_config["kf_scan_count"].as<int>();
    // kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;
    scan_count_ = 0;


    ORB_weight = VectorXd(3);
    PLICP_weight = VectorXd(3);
    
    ORB_weight << 0.5, 0.5, 0.5;
    PLICP_weight << 0.5, 0.5, 0.5;

    residual_sum = 10;

    count_res = 0;
    sum_res = 0;

    ORB_res = MatrixXd::Zero(residual_sum, 2);
    PLICP_res= MatrixXd::Zero(residual_sum, 2);
}

// compute PL-ICP compute the transform
void SlamGMapping::ScanMatchWithPLICP(LDP &curr_ldp_scan, const ros::Time &time)
{
    // CSM is used in the following way:
    // The scans are always in the laser frame
    // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
    // The new scan (currLDPScan) has a pose equal to the movement
    // of the laser in the laser frame since the last scan
    // The computed correction is then propagated using the tf machinery

    prev_ldp_scan_->odometry[0] = 0.0;
    prev_ldp_scan_->odometry[1] = 0.0;
    prev_ldp_scan_->odometry[2] = 0.0;

    prev_ldp_scan_->estimate[0] = 0.0;
    prev_ldp_scan_->estimate[1] = 0.0;
    prev_ldp_scan_->estimate[2] = 0.0;

    prev_ldp_scan_->true_pose[0] = 0.0;
    prev_ldp_scan_->true_pose[1] = 0.0;
    prev_ldp_scan_->true_pose[2] = 0.0;

    input_.laser_ref = prev_ldp_scan_;
    input_.laser_sens = curr_ldp_scan;

    // 匀速模型，速度乘以时间，得到预测的odom坐标系下的位姿变换
    double dt = (time - last_icp_time_).toSec();
    double pr_ch_x, pr_ch_y, pr_ch_a;
    GetPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

    tf::Transform prediction_change;
    // CreateTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, prediction_change);

    ////////   use odom as the first predict
    CreateTfFromXYTheta(odom_last.x, odom_last.y, pr_ch_a, prediction_change);
    prediction_change = prediction_change * (base_in_map_ * base_in_map_keyframe_.inverse());

    input_.first_guess[0] = sqrt(odom_last.x*odom_last.x + odom_last.y*odom_last.y);
    input_.first_guess[1] = 0.0;
    input_.first_guess[2] = tf::getYaw(prediction_change.getRotation());


    //////// use UKF predict
    // double x_change = plicp_ukf.x_[0] - plicp_last_predict.x;
    // double y_change = plicp_ukf.x_[1] - plicp_last_predict.y;

    // input_.first_guess[0] = sqrt(x_change*x_change + y_change*y_change);
    // input_.first_guess[1] = 0.0;
    // input_.first_guess[2] = tf::getYaw(prediction_change.getRotation());


    // If they are non-Null, free covariance gsl matrices to avoid leaking memory
    if (output_.cov_x_m)
    {
        gsl_matrix_free(output_.cov_x_m);
        output_.cov_x_m = 0;
    }
    if (output_.dx_dy1_m)
    {
        gsl_matrix_free(output_.dx_dy1_m);
        output_.dx_dy1_m = 0;
    }
    if (output_.dx_dy2_m)
    {
        gsl_matrix_free(output_.dx_dy2_m);
        output_.dx_dy2_m = 0;
    }

    // 位姿的预测值为0，就是不进行预测
    // input_.first_guess[0] = 0;
    // input_.first_guess[1] = 0;
    // input_.first_guess[2] = 0;

    // 调用csm里的函数进行plicp计算帧间的匹配，输出结果保存在output里
    sm_icp(&input_, &output_);


    // std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
    // std::cerr <<  plicp_ukf.x_[0] - plicp_last_predict.x << "  " << plicp_ukf.x_[1] - plicp_last_predict.y << std::endl;
    // std::cerr <<  plicp_ukf.x_[0] << "  " << plicp_ukf.x_[1]  << std::endl;
    // std::cerr <<   plicp_last_predict.x << "  " <<  plicp_last_predict.y << std::endl;
    // std::cerr <<  output_.x[0] << "  " << output_.x[1] << std::endl;
    // std::cerr <<  x_change << "  " << y_change << std::endl;


    tf::Transform corr_ch;

    if (output_.valid)
    {
        // 雷达坐标系下的坐标变换
        tf::Transform corr_ch_l;
        CreateTfFromXYTheta(-1*output_.x[0], -1*output_.x[1], output_.x[2], corr_ch_l);

        base_in_map_ = base_in_map_ * corr_ch_l;
        
        latest_velocity_.linear.x = corr_ch_l.getOrigin().getX() / dt;
        // latest_velocity_.linear.y = corr_ch_l.getOrigin().getY() / dt;
        latest_velocity_.angular.z = tf::getYaw(corr_ch_l.getRotation()) / dt;
    }
    else
    {
        ROS_WARN("not Converged");
    }

    // 检查是否需要更新关键帧坐标
    if (NewKeyframeNeeded(corr_ch))
    {
        // 更新关键帧坐标
        ld_free(prev_ldp_scan_);
        prev_ldp_scan_ = curr_ldp_scan;
        base_in_map_keyframe_ = base_in_map_;
    }
    else
    {
        ld_free(curr_ldp_scan);
    }

    // 删除prev_ldp_scan_，用curr_ldp_scan进行替代
    // ld_free(prev_ldp_scan_);
    // prev_ldp_scan_ = curr_ldp_scan;
    last_icp_time_ = time;
}

/**
 * 推测从上次icp的时间到当前时刻间的坐标变换
 * 使用匀速模型，根据当前的速度，乘以时间，得到推测出来的位移
 */
void SlamGMapping::GetPrediction( double &prediction_change_x,
                                  double &prediction_change_y,
                                  double &prediction_change_angle,
                                  double dt)
{
    // 速度小于 1e-6 , 则认为是静止的
    prediction_change_x = latest_velocity_.linear.x < 1e-6 ? 0.0 : dt * latest_velocity_.linear.x;
    prediction_change_y = latest_velocity_.linear.y < 1e-6 ? 0.0 : dt * latest_velocity_.linear.y;
    prediction_change_angle = latest_velocity_.linear.z < 1e-6 ? 0.0 : dt * latest_velocity_.linear.z;

    if (prediction_change_angle >= M_PI)
        prediction_change_angle -= 2.0 * M_PI;
    else if (prediction_change_angle < -M_PI)
        prediction_change_angle += 2.0 * M_PI;
}

/**
 * 从x,y,theta创建tf
 */
void SlamGMapping::CreateTfFromXYTheta(double x, double y, double theta, tf::Transform &t)
{
    t.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    t.setRotation(q);
}


/**
 * 如果平移大于阈值，角度大于阈值，则创新新的关键帧
 * @return 需要创建关键帧返回true, 否则返回false
 */
bool SlamGMapping::NewKeyframeNeeded(const tf::Transform &d)
{
    scan_count_++;

    if (fabs(tf::getYaw(d.getRotation())) > kf_dist_angular_)
        return true;

    if (scan_count_ == kf_scan_count_)
    {
        scan_count_ = 0;
        return true;
    }
        
    double x = d.getOrigin().getX();
    double y = d.getOrigin().getY();
    if (x * x + y * y > kf_dist_linear_sq_)
        return true;

    return false;
}

/**
 * 获取机器人坐标系与雷达坐标系间的坐标变换
 */
bool SlamGMapping::GetBaseToLaserTf(const std::string &frame_id)
{
    ros::Time t = ros::Time::now();

    geometry_msgs::TransformStamped transformStamped;
    // 获取tf并不是瞬间就能获取到的，要给1秒的缓冲时间让其找到tf
    try
    {
      tf2_ros::TransformListener tfListener_(tfBuffer_);
      transformStamped = tfBuffer_.lookupTransform("base_link", frame_id, t, ros::Duration(1.0));
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }

    // 将获取的tf存到base_to_laser_中
    tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
    base_to_laser_.setOrigin(tf::Vector3(transformStamped.transform.translation.x, transformStamped.transform.translation.y, 0.0));
    base_to_laser_.setRotation(q);

    laser_to_base_ = base_to_laser_.inverse();
    return true;
}


void SlamGMapping::publishMap(const sensor_msgs::LaserScan& scan)
{
  ROS_DEBUG("Update map");
  boost::mutex::scoped_lock map_lock (map_mutex_);
  GMapping::ScanMatcher matcher;

  GMapping::OrientedPoint fakepose;
  fakepose.x = 0;
  fakepose.y = 0;
  fakepose.theta = 0;

  // matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
  //                            gsp_laser_->getPose());
  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             fakepose);

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  // entropy (熵) : 機器人的精度(姿態分佈的估計值),值越高代表較高的不確定性
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);

  if(!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  GMapping::Point center;
  center.x=(xmin_ + xmax_) / 2.0;
  center.y=(ymin_ + ymax_) / 2.0;

  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, 
                                delta_);

  ROS_DEBUG("Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    ROS_DEBUG("  %.3f %.3f %.3f",
              n->pose.x,
              n->pose.y,
              n->pose.theta);
    if(!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));

  }

  // the map may have expanded, so resize ros message as well
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;
    
    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ > occ_thresh_)
      {
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}


// compute precision of PLICP
void SlamGMapping::Precision_PLICP(const nav_msgs::Odometry::ConstPtr& odom, double x, double y)
{
  AE_PLICP.push_back( abs(odom->pose.pose.position.x - x));
  AE_PLICP.push_back( abs(odom->pose.pose.position.y - y));

  double flag_x = pow(odom->pose.pose.position.x - x, 2);
  double flag_y = pow(odom->pose.pose.position.y - y, 2);
  MSE_PLICP_x   += flag_x; 
  MSE_PLICP_y   += flag_y; 
  MSE_PLICP_sum += flag_x + flag_y;
}
// compute precision of ORB
void SlamGMapping::Precision_ORB(const nav_msgs::Odometry::ConstPtr& odom, double x, double y)
{
  AE_ORBSLAM.push_back( abs(odom->pose.pose.position.x - x ));
  AE_ORBSLAM.push_back( abs(odom->pose.pose.position.y - y ));

  double flag_x = pow(odom->pose.pose.position.x - x, 2);
  double flag_y = pow(odom->pose.pose.position.y - y, 2);
  MSE_ORBSLAM_x   += flag_x; 
  MSE_ORBSLAM_y   += flag_y; 
  MSE_ORBSLAM_sum += flag_x + flag_y;
}


// compute precision of PLICP in UKF
void SlamGMapping::Precision_UKF_PLICP(const nav_msgs::Odometry::ConstPtr& odom, double x_, double y_, double x, double y)
{
  // compute average error and MSE by position of UKF and Gazebo
  AE_UKF_PLICP_odom.push_back( abs(odom->pose.pose.position.x - x ));
  AE_UKF_PLICP_odom.push_back( abs(odom->pose.pose.position.y - y ));

  double flag_x = pow(odom->pose.pose.position.x - x, 2);
  double flag_y = pow(odom->pose.pose.position.y - y, 2);
  MSE_UKF_PLICP_odom_x   += flag_x; 
  MSE_UKF_PLICP_odom_y   += flag_y; 
  MSE_UKF_PLICP_odom_sum += flag_x + flag_y;


  // compute MSE by position of ORBSLAM2-UKF and ORBSLAM2 
  AE_UKF_PLICP.push_back( abs(x_ - x ));
  AE_UKF_PLICP.push_back( abs(y_ - y ));

  flag_x = pow(x_ - x, 2);
  flag_y = pow(y_ - y, 2);
  MSE_UKF_PLICP_x   += flag_x; 
  MSE_UKF_PLICP_y   += flag_y; 
  MSE_UKF_PLICP_sum += flag_x + flag_y;
}
// compute precision of ORB in UKF
void SlamGMapping::Precision_UKF_ORB(const nav_msgs::Odometry::ConstPtr& odom, double x_, double y_, double x, double y)
{
  // compute average error and MSE by position of UKF and Gazebo
  AE_UKF_ORB_odom.push_back( abs(odom->pose.pose.position.x - x ));
  AE_UKF_ORB_odom.push_back( abs(odom->pose.pose.position.y - y ));

  double flag_x = pow(odom->pose.pose.position.x - x, 2);
  double flag_y = pow(odom->pose.pose.position.y - y, 2);
  MSE_UKF_ORB_odom_x   += flag_x; 
  MSE_UKF_ORB_odom_y   += flag_y; 
  MSE_UKF_ORB_odom_sum += flag_x + flag_y;


  // compute MSE by position of ORBSLAM2-UKF and ORBSLAM2 
  AE_UKF_ORB.push_back( abs(x_ - x ));
  AE_UKF_ORB.push_back( abs(y_ - y ));

  flag_x = pow(x_ - x, 2);
  flag_y = pow(y_ - y, 2);
  MSE_UKF_ORB_x   += flag_x; 
  MSE_UKF_ORB_y   += flag_y; 
  MSE_UKF_ORB_sum += flag_x + flag_y;
}


void SlamGMapping::Precision_Best_Pose(const nav_msgs::Odometry::ConstPtr& odom, double x, double y)
{
  // compute average error and MSE between best position and Gazebo
  AE_best_odom.push_back( abs(odom->pose.pose.position.x - x ));
  AE_best_odom.push_back( abs(odom->pose.pose.position.y - y ));

  double flag_x = pow(odom->pose.pose.position.x - x, 2);
  double flag_y = pow(odom->pose.pose.position.y - y, 2);
  MSE_best_odom_x   += flag_x; 
  MSE_best_odom_y   += flag_y; 
  MSE_best_odom_sum += flag_x + flag_y;
}



bool SlamGMapping::KillTrigger(  all_process::Trigger::Request  &req,
                                 all_process::Trigger::Response &res)
{
  SaveTrajectoryGraph();
////////////////////////////////  update param  ////////////////////////////////////////
  bool flag_orb = false;
  bool flag_plicp;
  bool flag_best;
  // tuning ORB and PLICP param
  // flag_orb = SetORBParam();
  // flag_plicp = SetPLICPParam();

  // tuning UKF param of ORB and PLICP
  flag_orb = SetUKFORBParam();
  flag_plicp = SetUKFPLICPParam();
  
  flag_best = SetHypothesisParam();
////////////////////////////////////////////////////////////////////////////////////////

  if (flag_orb == true || flag_plicp == true)
  { 
    res.trigger = true;
    return true;
  }
  if (flag_orb == false && flag_plicp == false)
  {
    res.trigger = false;
    return false;
  }
}

void SlamGMapping::TerminateTrigger()
{
  SaveTrajectoryGraph();
////////////////////////////////  update param  ////////////////////////////////////////
  bool flag_orb = false;
  bool flag_plicp;
  bool flag_best;

  // tuning ORB and PLICP param
  flag_orb = SetORBParam();
  flag_plicp = SetPLICPParam();

  // tuning UKF param of ORB and PLICP
  // flag_orb = SetUKFORBParam();
  // flag_plicp = SetUKFPLICPParam();
  
  flag_best = SetHypothesisParam();
}


void SlamGMapping::SaveTrajectoryGraph()
{
  //// save graph
  std::pair<std::vector<double>, std::vector<double>> points_PLICP = GetPoints(trajectory_PLICP);
  plt::plot(points_PLICP.first, points_PLICP.second, "m", {{"label", "PLICP"}});

  std::pair<std::vector<double>, std::vector<double>> points_ORB = GetPoints(trajectory_ORB);
  plt::plot(points_ORB.first, points_ORB.second, "b", {{"label", "ORB"}});

  std::pair<std::vector<double>, std::vector<double>> points_UKF_PLICP = GetPoints(trajectory_UKF_PLICP);
  plt::plot(points_UKF_PLICP.first, points_UKF_PLICP.second, "g", {{"label", "UKF_PLICP"}});

  std::pair<std::vector<double>, std::vector<double>> points_UKF_ORB = GetPoints(trajectory_UKF_ORB);
  plt::plot(points_UKF_ORB.first, points_UKF_ORB.second, "y", {{"label", "UKF_ORB"}});

  std::pair<std::vector<double>, std::vector<double>> points_real = GetPoints(trajectory_real);
  plt::plot(points_real.first, points_real.second, "k", {{"label", "real"}});

  std::pair<std::vector<double>, std::vector<double>> points_best = GetPoints(trajectory_best);
  plt::plot(points_best.first, points_best.second, "r", {{"label", "Our_method"}});

  // set lable
  plt::xlabel("x (cm)");
  plt::ylabel("y (cm)");
  // enable legend.
  plt::legend();

  // save path
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y_%m_%d_%X", &tstruct);

  std::string graph_path = log_path + "/data/Trajectory/Graph/" + std::string(buf) + ".pdf";
  plt::title("Trajectory");
  plt::savefig(graph_path);
  

  // save log
  YAML::Node config;

  YAML::Node orb_config = YAML::LoadFile(orb_path);
  config["ORB"]["nFeatures"]   = orb_config["ORBextractor.nFeatures"].as<int>();
  config["ORB"]["scaleFactor"] = orb_config["ORBextractor.scaleFactor"].as<float>();
  config["ORB"]["nLevels"]     = orb_config["ORBextractor.nLevels"].as<int>();
  config["ORB"]["iniThFAST"]   = orb_config["ORBextractor.iniThFAST"].as<int>();
  config["ORB"]["minThFAST"]   = orb_config["ORBextractor.minThFAST"].as<int>();

  YAML::Node plicp_config = YAML::LoadFile(plicp_path);
  config["PLICP"]["max_angular_correction_deg"] = plicp_config["max_angular_correction_deg"].as<float>();
  config["PLICP"]["max_linear_correction"]      = plicp_config["max_linear_correction"].as<float>();
  config["PLICP"]["max_iterations"]             = plicp_config["max_iterations"].as<int>();
  config["PLICP"]["epsilon_xy"]                 = plicp_config["epsilon_xy"].as<float>();
  config["PLICP"]["epsilon_theta"]              = plicp_config["epsilon_theta"].as<float>();
  config["PLICP"]["max_correspondence_dist"]    = plicp_config["max_correspondence_dist"].as<float>();
  config["PLICP"]["outliers_maxPerc"]           = plicp_config["outliers_maxPerc"].as<float>();

  std::string ukf_orb_path = log_path + "/ORB_UKF_config.yaml";
  YAML::Node ukf_orb_config = YAML::LoadFile(ukf_orb_path);
  config["ORB_UKF"]["std_a_"]     = ukf_orb_config["std_a_"].as<double>();
  config["ORB_UKF"]["std_yawdd_"] = ukf_orb_config["std_yawdd_"].as<double>();
  config["ORB_UKF"]["std_laspx_"] = ukf_orb_config["std_laspx_"].as<double>();
  config["ORB_UKF"]["std_laspy_"] = ukf_orb_config["std_laspy_"].as<double>();
  config["ORB_UKF"]["std_beta_"]  = ukf_orb_config["std_beta_"].as<double>();
  config["ORB_UKF"]["std_alpha_"] = ukf_orb_config["std_alpha_"].as<double>();
  config["ORB_UKF"]["std_k_"]     = ukf_orb_config["std_k_"].as<double>();

  std::string ukf_plicp_path = log_path + "/PLICP_UKF_config.yaml";
  YAML::Node ukf_plicp_config = YAML::LoadFile(ukf_plicp_path);
  config["PLICP_UKF"]["std_a_"]     = ukf_plicp_config["std_a_"].as<double>();
  config["PLICP_UKF"]["std_yawdd_"] = ukf_plicp_config["std_yawdd_"].as<double>();
  config["PLICP_UKF"]["std_laspx_"] = ukf_plicp_config["std_laspx_"].as<double>();
  config["PLICP_UKF"]["std_laspy_"] = ukf_plicp_config["std_laspy_"].as<double>();
  config["PLICP_UKF"]["std_beta_"]  = ukf_plicp_config["std_beta_"].as<double>();
  config["PLICP_UKF"]["std_alpha_"] = ukf_plicp_config["std_alpha_"].as<double>();
  config["PLICP_UKF"]["std_k_"]     = ukf_plicp_config["std_k_"].as<double>();

  config["Trajectory"]["PLICP"] = trajectory_PLICP;
  config["Trajectory"]["ORB"] = trajectory_ORB;
  config["Trajectory"]["UKF_PLICP"] = trajectory_UKF_PLICP;
  config["Trajectory"]["UKF_ORB"] = trajectory_UKF_ORB;
  config["Trajectory"]["real"] = trajectory_real;
  config["Trajectory"]["Our_method"] = trajectory_best;


  std::string graph_log_path = log_path + "/data/Trajectory/Log/" + std::string(buf) + ".yaml";
  std::ofstream fout(graph_log_path);
  fout << config;
}

bool SlamGMapping::SetORBParam()
{
  YAML::Node orb_config = YAML::LoadFile(orb_path);

  int nFeatures     = orb_config["ORBextractor.nFeatures"].as<int>();
  float scaleFactor = orb_config["ORBextractor.scaleFactor"].as<float>();
  int nLevels       = orb_config["ORBextractor.nLevels"].as<int>();
  int iniThFAST     = orb_config["ORBextractor.iniThFAST"].as<int>();
  int minThFAST     = orb_config["ORBextractor.minThFAST"].as<int>();
  
  // adjust the param setting (first tune)
  // int nFeatures_max = 6500;
  // int nFeatures_min = 500;
  // int nFeatures_add = 1000;

  // float scaleFactor_max = 1.5;
  // float scaleFactor_min = 1.1;
  // float scaleFactor_add = 0.1;

  // int nLevels_max = 10;
  // int nLevels_min = 4;
  // int nLevels_add = 2;

  // int iniThFAST_max = 30;
  // int iniThFAST_min = 12;
  // int iniThFAST_add = 6;

  // int minThFAST_max = 12;
  // int minThFAST_min = 3;
  // int minThFAST_add = 3;

  // adjust the param setting (second tune)
  int nFeatures_max = 2000;
  int nFeatures_min = 1000;
  int nFeatures_add = 250;

  float scaleFactor_max = 1.25;
  float scaleFactor_min = 1.15;
  float scaleFactor_add = 0.05;

  int nLevels_max = 5;
  int nLevels_min = 3;
  int nLevels_add = 1;

  int minThFAST_max = 13;
  int minThFAST_min = 7;
  int minThFAST_add = 2;

  int iniThFAST_max = 27;
  int iniThFAST_min = 21;
  int iniThFAST_add = 3;

  // set flag to avoid program
  if ( nFeatures >= nFeatures_max )
    if ( scaleFactor >= scaleFactor_max )
      if ( nLevels >= nLevels_max )
        if ( minThFAST >= minThFAST_max )
          if ( iniThFAST >= iniThFAST_max )
            return false;


  //compute average error
  float sum = 0;
  for (int i=0; i<AE_ORBSLAM.size(); i++)
    sum += AE_ORBSLAM[i];
  sum = sum/AE_ORBSLAM.size();

  MSE_ORBSLAM_x = MSE_ORBSLAM_x/AE_ORBSLAM.size()*2;
  MSE_ORBSLAM_y = MSE_ORBSLAM_y/AE_ORBSLAM.size()*2;
  MSE_ORBSLAM_sum = MSE_ORBSLAM_sum/AE_ORBSLAM.size();

  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y_%m_%d_%X", &tstruct);

  std::fstream logFile_ORB;
  // Open File
  // std::string path = log_path + "/data/ORB/log_" + std::to_string(iniThFAST) + '_' + std::to_string(minThFAST) + ".txt";
  // std::string path = log_path + "/data/ORB/log_" + std::to_string(minThFAST) + '_' + std::to_string(iniThFAST) + ".txt";
  std::string path = log_path + "/data/ORB/log_" + std::string(buf) + ".txt";
  logFile_ORB.open( path, std::ios::app);

  //Write data into log file
  logFile_ORB << "Features:" + std::to_string(nFeatures);
  logFile_ORB << " scale:"   + std::to_string(scaleFactor);
  logFile_ORB << " Levels:"  + std::to_string(nLevels);
  logFile_ORB << " iniFAST:" + std::to_string(iniThFAST);
  logFile_ORB << " minFAST:" + std::to_string(minThFAST);
  logFile_ORB << "---avg:"   + std::to_string(sum);
  logFile_ORB << " mseX:"    + std::to_string(MSE_ORBSLAM_x);
  logFile_ORB << " mseY:"    + std::to_string(MSE_ORBSLAM_y);
  logFile_ORB << " mseS:"    + std::to_string(MSE_ORBSLAM_sum);
  logFile_ORB << "\n";
  // close file stream
  logFile_ORB.close();
  return true;

  if (nFeatures >= nFeatures_min && nFeatures < nFeatures_max)
    orb_config["ORBextractor.nFeatures"] = nFeatures + nFeatures_add;
  else 
  {
    orb_config["ORBextractor.nFeatures"] = nFeatures_min;

    if (scaleFactor >= scaleFactor_min && scaleFactor < scaleFactor_max)
      orb_config["ORBextractor.scaleFactor"] = scaleFactor + scaleFactor_add;
    else
    {
      orb_config["ORBextractor.scaleFactor"] = scaleFactor_min;

      if (nLevels >= nLevels_min && nLevels < nLevels_max)
        orb_config["ORBextractor.nLevels"] = nLevels + nLevels_add;
      else
      {
        orb_config["ORBextractor.nLevels"] = nLevels_min;

        if (minThFAST >= minThFAST_min && minThFAST < minThFAST_max)
          orb_config["ORBextractor.minThFAST"] = minThFAST + minThFAST_add;
        else
        {
          orb_config["ORBextractor.minThFAST"] = minThFAST_min;
          if (iniThFAST >= iniThFAST_min && iniThFAST < iniThFAST_max)
            orb_config["ORBextractor.iniThFAST"] = iniThFAST + iniThFAST_add;
          else
            return true;
        }
      }
    }
  }

  std::ofstream fout(orb_path);
  YAML::Emitter orb_emitter;
  orb_emitter << orb_config;
  
  fout << "%YAML:1.0" << endl;
  fout << "---" << endl;
  fout << orb_emitter.c_str() << endl;
  fout.close();
  return true;
}


bool SlamGMapping::SetPLICPParam()
{
  YAML::Node plicp_config = YAML::LoadFile(plicp_path);

  float max_angular_correction_deg = plicp_config["max_angular_correction_deg"].as<float>();
  float max_linear_correction      = plicp_config["max_linear_correction"].as<float>();
  int   max_iterations             = plicp_config["max_iterations"].as<int>();
  float epsilon_xy                 = plicp_config["epsilon_xy"].as<float>();
  float epsilon_theta              = plicp_config["epsilon_theta"].as<float>();
  float max_correspondence_dist    = plicp_config["max_correspondence_dist"].as<float>();
  float outliers_maxPerc           = plicp_config["outliers_maxPerc"].as<float>();

  // adjust the param setting (first tune)
  // float deg_max = 180.0;
  // float deg_min = 45.0;
  // float deg_add = 45.0;

  // float cor_max = 0.9;
  // float cor_min = 0.5;
  // float cor_add = 0.2;

  // int itera_max = 15;
  // int itera_min = 5;
  // int itera_add = 5;

  // float ep_xy_max = 0.001;
  // float ep_xy_min = 0.0000001;
  // float ep_xy_add = 10;

  // float ep_theta_max = 0.001;
  // float ep_theta_min = 0.0000001;
  // float ep_theta_add = 10;

  // float dist_max = 1.0;
  // float dist_min = 0.5;
  // float dist_add = 0.25;

  // float Perc_max = 0.9;
  // float Perc_min = 0.5;
  // float Perc_add = 0.1;

    // adjust the param setting (second tune)
  // float deg_max = 360.0;
  // float deg_min = 180.0;
  // float deg_add = 60.0;

  // float cor_max = 0.6;
  // float cor_min = 0.2;
  // float cor_add = 0.1;

  // int itera_max = 7;
  // int itera_min = 4;
  // int itera_add = 1;

  // float dist_max = 0.7;
  // float dist_min = 0.4;
  // float dist_add = 0.1;

  // float Perc_max = 0.7;
  // float Perc_min = 0.6;
  // float Perc_add = 0.1;

  // adjust the param setting (third tune)
  float deg_max = 180.0;
  float deg_min = 45.0;
  float deg_add = 45.0;

  float cor_max = 1.25;
  float cor_min = 0.75;
  float cor_add = 0.25;

  int itera_max = 12;
  int itera_min = 6;
  int itera_add = 2;

  float ep_xy_max = 0.0001;
  float ep_xy_min = 0.0000001;
  float ep_xy_add = 10;

  float ep_theta_max = 0.0001;
  float ep_theta_min = 0.0000001;
  float ep_theta_add = 10;

  float dist_max = 1.5;
  float dist_min = 0.5;
  float dist_add = 0.5;



  //compute average error
  float sum = 0;
  for (int i=0; i<AE_PLICP.size(); i++)
    sum += AE_PLICP[i];
  sum = sum/AE_PLICP.size();

  MSE_PLICP_x = MSE_PLICP_x/AE_PLICP.size()*2;
  MSE_PLICP_y = MSE_PLICP_y/AE_PLICP.size()*2;
  MSE_PLICP_sum = MSE_PLICP_sum/AE_PLICP.size();

  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y_%m_%d_%X", &tstruct);

  std::fstream logFile_PLICP;
  // Open File
  // std::string path = log_path + "/data/PLICP/log_" + std::to_string(epsilon_theta) + '_' + std::to_string(max_correspondence_dist) + '_' + std::to_string(outliers_maxPerc) + ".txt";
  // std::string path = log_path + "/data/PLICP/log_" + std::to_string(max_correspondence_dist) + '_' + std::to_string(outliers_maxPerc) + ".txt";
  std::string path = log_path + "/data/PLICP/log_" + std::string(buf) + ".txt";
  logFile_PLICP.open( path, std::ios::app);

  //Write data into log file
  logFile_PLICP << "deg:"      + std::to_string(max_angular_correction_deg);
  logFile_PLICP << " cor:"      + std::to_string(max_linear_correction);
  logFile_PLICP << " itera:"    + std::to_string(max_iterations);
  logFile_PLICP << " ep_xy:"    + std::to_string(epsilon_xy);
  logFile_PLICP << " ep_theta:" + std::to_string(epsilon_theta);
  logFile_PLICP << " dist:"     + std::to_string(max_correspondence_dist);
  logFile_PLICP << " Perc:"     + std::to_string(outliers_maxPerc);

  logFile_PLICP << "---avg:"   + std::to_string(sum);
  logFile_PLICP << " mseX:"    + std::to_string(MSE_PLICP_x);
  logFile_PLICP << " mseY:"    + std::to_string(MSE_PLICP_y);
  logFile_PLICP << " mseS:"    + std::to_string(MSE_PLICP_sum);
  logFile_PLICP << "\n";
  // close file stream
  logFile_PLICP.close();
  return true;

  if (max_angular_correction_deg >= deg_min && max_angular_correction_deg < deg_max)
    plicp_config["max_angular_correction_deg"] = max_angular_correction_deg + deg_add;
  else 
  {
    plicp_config["max_angular_correction_deg"] = deg_min;

    if (max_linear_correction >= cor_min && max_linear_correction < cor_max)
      plicp_config["max_linear_correction"] = max_linear_correction + cor_add;
    else
    {
      plicp_config["max_linear_correction"] = cor_min;

      if (max_iterations >= itera_min && max_iterations < itera_max)
        plicp_config["max_iterations"] = max_iterations + itera_add;
      else
      {
        plicp_config["max_iterations"] = itera_min;

        if (epsilon_xy >= ep_xy_min && epsilon_xy < ep_xy_max)
          plicp_config["epsilon_xy"] = epsilon_xy * ep_xy_add;
        else
        {
          plicp_config["epsilon_xy"] = ep_xy_min;

          if (epsilon_theta >= ep_theta_min && epsilon_theta < ep_theta_max)
            plicp_config["epsilon_theta"] = epsilon_theta * ep_theta_add;
          else
          {
            plicp_config["epsilon_theta"] = ep_theta_min;

            if (max_correspondence_dist >= dist_min && max_correspondence_dist < dist_max)
              plicp_config["max_correspondence_dist"] = max_correspondence_dist + dist_add;
            else
            {
              // plicp_config["max_correspondence_dist"] = dist_min;

              // if (outliers_maxPerc >= Perc_min && outliers_maxPerc < Perc_max)
              //   plicp_config["outliers_maxPerc"] = outliers_maxPerc + Perc_add;
              // else
                return false;
            }
          }
        }
      }
    }
  }

  std::ofstream fout(plicp_path);
  YAML::Emitter plicp_emitter;
  plicp_emitter << plicp_config;
  fout << "%YAML:1.0" << endl;
  fout << "---" << endl;
  fout << plicp_emitter.c_str() << endl;
  fout.close();
  return true;
}

bool SlamGMapping::SetUKFORBParam()
{
  std::string my_path = log_path + "/ORB_UKF_config.yaml";

  YAML::Node orb_config = YAML::LoadFile(my_path);

  double std_a_     = orb_config["std_a_"].as<double>();
  double std_yawdd_ = orb_config["std_yawdd_"].as<double>();
  double std_laspx_ = orb_config["std_laspx_"].as<double>();
  double std_laspy_ = orb_config["std_laspy_"].as<double>();

  // for second tune, add alpha, beta, k
  double std_alpha_ = orb_config["std_alpha_"].as<double>();
  double std_k_ = orb_config["std_k_"].as<double>();
  double std_beta_ = orb_config["std_beta_"].as<double>();

  // // adjust the param setting (first tune)
  // double a_max = 1.8;
  // double a_min = 0.2;
  // double a_add = 0.2;

  // double yawdd_max = 0.5;
  // double yawdd_min = 0.05;
  // double yawdd_add = 0.05;

  // double laspx_max = 0.18;
  // double laspx_min = 0.03;
  // double laspx_add = 0.03;

  // double laspy_max = 0.09;
  // double laspy_min = 0.03;
  // double laspy_add = 0.03;

  // adjust the param setting (second tune)


  double yawdd_max = 0.4;
  double yawdd_min = 0.3;
  double yawdd_add = 0.02;

  double laspy_max = 0.15;
  double laspy_min = 0.09;
  double laspy_add = 0.01;

  double beta_max = 5;
  double beta_min = 0.0;
  double beta_add = 0.5;

  double alpha_max = 1.0;
  double alpha_min = 0.1;
  double alpha_add = 0.1;

  // double k_max = 10.0;
  // double k_min = 0.0;
  // double k_add = 1.0;





  //compute average error
  float sum_odom = 0;
  float sum = 0;
  for (int i=0; i<AE_UKF_ORB.size(); i++)
  {
    sum_odom += AE_UKF_ORB_odom[i];
    sum      += AE_UKF_ORB[i];
  }

  sum_odom = sum_odom/AE_UKF_ORB_odom.size();
  sum      = sum/AE_UKF_ORB.size();



  MSE_UKF_ORB_x = MSE_UKF_ORB_x/AE_UKF_ORB.size()*2;
  MSE_UKF_ORB_y = MSE_UKF_ORB_y/AE_UKF_ORB.size()*2;
  MSE_UKF_ORB_sum = MSE_UKF_ORB_sum/AE_UKF_ORB.size();

  MSE_UKF_ORB_odom_x = MSE_UKF_ORB_odom_x/AE_UKF_ORB_odom.size()*2;
  MSE_UKF_ORB_odom_y = MSE_UKF_ORB_odom_y/AE_UKF_ORB_odom.size()*2;
  MSE_UKF_ORB_odom_sum = MSE_UKF_ORB_odom_sum/AE_UKF_ORB_odom.size();

  std::fstream logFile_ORB;
  std::fstream logFile_ORB_odom;

  // Open File
  std::string odom_path  = log_path + "/data/ORB/ORB_odom/log_" + std::to_string(std_k_) + '_' + std::to_string(std_alpha_) + ".txt";
  std::string ORB_path2 = log_path + "/data/ORB/ORB/log_" + std::to_string(std_k_) + '_' + std::to_string(std_alpha_) + ".txt";

  logFile_ORB.open( ORB_path2, std::ios::app);

  //Write data into log file
  logFile_ORB << "a:"      + std::to_string(std_a_);
  logFile_ORB << " yawdd:" + std::to_string(std_yawdd_);
  logFile_ORB << " laspx:" + std::to_string(std_laspx_);
  logFile_ORB << " laspy:" + std::to_string(std_laspy_);
  logFile_ORB << " alpha:" + std::to_string(std_alpha_);
  logFile_ORB << " k:" + std::to_string(std_k_);
  logFile_ORB << " beta:" + std::to_string(std_beta_);

  logFile_ORB << "---avg:"   + std::to_string(sum);
  logFile_ORB << " mseX:"    + std::to_string(MSE_UKF_ORB_x);
  logFile_ORB << " mseY:"    + std::to_string(MSE_UKF_ORB_y);
  logFile_ORB << " mseS:"    + std::to_string(MSE_UKF_ORB_sum);
  logFile_ORB << "\n";
  // close file stream
  logFile_ORB.close();

  logFile_ORB_odom.open( odom_path, std::ios::app);
  //Write data into log file
  logFile_ORB_odom << "a:"      + std::to_string(std_a_);
  logFile_ORB_odom << " yawdd:" + std::to_string(std_yawdd_);
  logFile_ORB_odom << " laspx:" + std::to_string(std_laspx_);
  logFile_ORB_odom << " laspy:" + std::to_string(std_laspy_);
  logFile_ORB_odom << " alpha:" + std::to_string(std_alpha_);
  logFile_ORB_odom << " k:" + std::to_string(std_k_);
  logFile_ORB_odom << " beta:" + std::to_string(std_beta_);

  logFile_ORB_odom << "---avg:"   + std::to_string(sum);
  logFile_ORB_odom << " mseX:"    + std::to_string(MSE_UKF_ORB_odom_x);
  logFile_ORB_odom << " mseY:"    + std::to_string(MSE_UKF_ORB_odom_y);
  logFile_ORB_odom << " mseS:"    + std::to_string(MSE_UKF_ORB_odom_sum);
  logFile_ORB_odom << "\n";
  // close file stream (first)
  logFile_ORB_odom.close();

  ////////// furst tune //////////////
  // if (std_a_ >= a_min && std_a_ < a_max)
  //   orb_config["std_a_"] = std_a_ + a_add;
  // else 
  // {
  //   orb_config["std_a_"] = a_min;

  //   if (std_yawdd_ >= yawdd_min && std_yawdd_ < yawdd_max)
  //     orb_config["std_yawdd_"] = std_yawdd_ + yawdd_add;
  //   else
  //   {
  //     orb_config["std_yawdd_"] = yawdd_min;

  //     if (std_laspx_ >= laspx_min && std_laspx_ < laspx_max)
  //       orb_config["std_laspx_"] = std_laspx_ + laspx_add;
  //     else
  //     {
  //       orb_config["std_laspx_"] = laspx_min;
  //       if (std_laspy_ >= laspy_min && std_laspy_ < laspy_max)
  //         orb_config["std_laspy_"] = std_laspy_ + laspy_add;
  //       else
  //         return false;
  //     }
  //   }
  // }

  ////////// second tune //////////////

  if (std_yawdd_ >= yawdd_min && std_yawdd_ < yawdd_max)
    orb_config["std_yawdd_"] = std_yawdd_ + yawdd_add;
  else
  {
    orb_config["std_yawdd_"] = yawdd_min;

    if (std_laspy_ >= laspy_min && std_laspy_ < laspy_max)
      orb_config["std_laspy_"] = std_laspy_ + laspy_add;
    else
    {
      orb_config["std_laspy_"] = laspy_min;

      if (std_beta_ >= beta_min && std_beta_ < beta_max)
        orb_config["std_beta_"] = std_beta_ + beta_add;
      else
      {
        orb_config["std_beta_"] = beta_min;

        if (std_alpha_ >= alpha_min && std_alpha_ < alpha_max )
          orb_config["std_alpha_"] = std_alpha_ + alpha_add;

        if ((std_alpha_ + alpha_add ) > alpha_max)
        {
          // orb_config["std_k_"] = std_k_ + k_add;
          // if (std_k_ > k_max )
            return false;
        }
      }
    }
  }
  

  std::ofstream fout(my_path);
  YAML::Emitter orb_emitter;
  orb_emitter << orb_config;
  fout << "%YAML:1.0" << endl;
  fout << "---" << endl;
  fout << orb_emitter.c_str() << endl;
  fout.close();
  return true; 
}



bool SlamGMapping::SetUKFPLICPParam()
{
  std::string my_path = log_path + "/PLICP_UKF_config.yaml";

  YAML::Node plicp_config = YAML::LoadFile(my_path);

  double std_a_     = plicp_config["std_a_"].as<double>();
  double std_yawdd_ = plicp_config["std_yawdd_"].as<double>();
  double std_laspx_ = plicp_config["std_laspx_"].as<double>();
  double std_laspy_ = plicp_config["std_laspy_"].as<double>();

  // for second tune, add alpha, beta, k
  double std_alpha_ = plicp_config["std_alpha_"].as<double>();
  double std_k_ = plicp_config["std_k_"].as<double>();
  double std_beta_ = plicp_config["std_beta_"].as<double>();

  // adjust the param setting (first tune)
  // double a_max = 1.8;
  // double a_min = 0.2;
  // double a_add = 0.2;

  // double yawdd_max = 0.5;
  // double yawdd_min = 0.05;
  // double yawdd_add = 0.05;

  // double laspx_max = 0.18;
  // double laspx_min = 0.03;
  // double laspx_add = 0.03;

  // double laspy_max = 0.09;
  // double laspy_min = 0.03;
  // double laspy_add = 0.03;

  // adjust the param setting (second tune)
  double yawdd_max = 0.4;
  double yawdd_min = 0.3;
  double yawdd_add = 0.02;

  double laspy_max = 0.15;
  double laspy_min = 0.09;
  double laspy_add = 0.01;

  double beta_max = 5;
  double beta_min = 0.0;
  double beta_add = 0.5;

  double alpha_max = 1.0;
  double alpha_min = 0.1;
  double alpha_add = 0.1;

  double k_max = 10.0;
  double k_min = 0.0;
  double k_add = 1.0;



  //compute average error
  float sum_odom = 0;
  float sum = 0;
  for (int i=0; i<AE_UKF_PLICP.size(); i++)
  {
    sum_odom += AE_UKF_PLICP_odom[i];
    sum      += AE_UKF_PLICP[i];
  }

  sum_odom = sum_odom/AE_UKF_PLICP_odom.size();
  sum      = sum/AE_UKF_PLICP.size();



  MSE_UKF_PLICP_x = MSE_UKF_PLICP_x/AE_UKF_PLICP.size()*2;
  MSE_UKF_PLICP_y = MSE_UKF_PLICP_y/AE_UKF_PLICP.size()*2;
  MSE_UKF_PLICP_sum = MSE_UKF_PLICP_sum/AE_UKF_PLICP.size();

  MSE_UKF_PLICP_odom_x = MSE_UKF_PLICP_odom_x/AE_UKF_PLICP_odom.size()*2;
  MSE_UKF_PLICP_odom_y = MSE_UKF_PLICP_odom_y/AE_UKF_PLICP_odom.size()*2;
  MSE_UKF_PLICP_odom_sum = MSE_UKF_PLICP_odom_sum/AE_UKF_PLICP_odom.size();

  std::fstream logFile_PLICP;
  std::fstream logFile_PLICP_odom;

  // Open File
  std::string odom_path  = log_path + "/data/PLICP/PLICP_odom/log_" + std::to_string(std_k_) + '_' + std::to_string(std_alpha_) + ".txt";
  std::string PLICP_path2 = log_path + "/data/PLICP/PLICP/log_" + std::to_string(std_k_) + '_' + std::to_string(std_alpha_) + ".txt";

  logFile_PLICP.open( PLICP_path2, std::ios::app);

  //Write data into log file
  logFile_PLICP << "a:"      + std::to_string(std_a_);
  logFile_PLICP << " yawdd:" + std::to_string(std_yawdd_);
  logFile_PLICP << " laspx:" + std::to_string(std_laspx_);
  logFile_PLICP << " laspy:" + std::to_string(std_laspy_);
  logFile_PLICP << " alpha:" + std::to_string(std_alpha_);
  logFile_PLICP << " k:" + std::to_string(std_k_);
  logFile_PLICP << " beta:" + std::to_string(std_beta_);

  logFile_PLICP << "---avg:"   + std::to_string(sum);
  logFile_PLICP << " mseX:"    + std::to_string(MSE_UKF_PLICP_x);
  logFile_PLICP << " mseY:"    + std::to_string(MSE_UKF_PLICP_y);
  logFile_PLICP << " mseS:"    + std::to_string(MSE_UKF_PLICP_sum);
  logFile_PLICP << "\n";
  // close file stream
  logFile_PLICP.close();

  logFile_PLICP_odom.open( odom_path, std::ios::app);
  //Write data into log file
  logFile_PLICP_odom << "a:"      + std::to_string(std_a_);
  logFile_PLICP_odom << " yawdd:" + std::to_string(std_yawdd_);
  logFile_PLICP_odom << " laspx:" + std::to_string(std_laspx_);
  logFile_PLICP_odom << " laspy:" + std::to_string(std_laspy_);
  logFile_PLICP_odom << " alpha:" + std::to_string(std_alpha_);
  logFile_PLICP_odom << " k:" + std::to_string(std_k_);
  logFile_PLICP_odom << " beta:" + std::to_string(std_beta_);

  logFile_PLICP_odom << "---avg:"   + std::to_string(sum);
  logFile_PLICP_odom << " mseX:"    + std::to_string(MSE_UKF_PLICP_odom_x);
  logFile_PLICP_odom << " mseY:"    + std::to_string(MSE_UKF_PLICP_odom_y);
  logFile_PLICP_odom << " mseS:"    + std::to_string(MSE_UKF_PLICP_odom_sum);
  logFile_PLICP_odom << "\n";
  // close file stream
  logFile_PLICP_odom.close();


  //////////////// first tune ///////////////////////////
  // if (std_a_ >= a_min && std_a_ < a_max)
  //   plicp_config["std_a_"] = std_a_ + a_add;
  // else 
  // {
  //   plicp_config["std_a_"] = a_min;

  //   if (std_yawdd_ >= yawdd_min && std_yawdd_ < yawdd_max)
  //     plicp_config["std_yawdd_"] = std_yawdd_ + yawdd_add;
  //   else
  //   {
  //     plicp_config["std_yawdd_"] = yawdd_min;

  //     if (std_laspx_ >= laspx_min && std_laspx_ < laspx_max)
  //       plicp_config["std_laspx_"] = std_laspx_ + laspx_add;
  //     else
  //     {
  //       plicp_config["std_laspx_"] = laspx_min;
  //       if (std_laspy_ >= laspy_min && std_laspy_ < laspy_max)
  //         plicp_config["std_laspy_"] = std_laspy_ + laspy_add;
  //       else
  //         return false;
  //     }
  //   }
  // }

  //////////////// second tune ///////////////////////////
  if (std_yawdd_ >= yawdd_min && std_yawdd_ < yawdd_max)
    plicp_config["std_yawdd_"] = std_yawdd_ + yawdd_add;
  else
  {
    plicp_config["std_yawdd_"] = yawdd_min;

    if (std_laspy_ >= laspy_min && std_laspy_ < laspy_max)
      plicp_config["std_laspy_"] = std_laspy_ + laspy_add;
    else
    {
      plicp_config["std_laspy_"] = laspy_min;

      if (std_beta_ >= beta_min && std_beta_ < beta_max)
        plicp_config["std_beta_"] = std_beta_ + beta_add;
      else
      {
        plicp_config["std_beta_"] = beta_min;

        if (std_alpha_ >= alpha_min && std_alpha_ < alpha_max )
          plicp_config["std_alpha_"] = std_alpha_ + alpha_add;

        if ((std_alpha_ + alpha_add ) > alpha_max)
        {
          // plicp_config["std_k_"] = std_k_ + k_add;
          // if (std_k_ > k_max )
            return false;
        }
      }
    }
  }

  std::ofstream fout(my_path);
  YAML::Emitter plicp_emitter;
  plicp_emitter << plicp_config;
  fout << "%YAML:1.0" << endl;
  fout << "---" << endl;
  fout << plicp_emitter.c_str() << endl;
  fout.close();
  return true;
}



void SlamGMapping::SavePosition(std::vector<double>& container, double x, double y, double current_time)
{
  // if (container.size() == 0)
  // {
  //   container.push_back(100*x);
  //   container.push_back(100*y);
  //   container.push_back(current_time);
  // }
  // else if (container[container.size()-3] != 100*x && container[container.size()-2] != 100*y)
  // {
  //   container.push_back(100*x);
  //   container.push_back(100*y);
  //   container.push_back(current_time);
  // }

  container.push_back(100*x);
  container.push_back(100*y);
  container.push_back(current_time);
}

std::pair<std::vector<double>, std::vector<double>> SlamGMapping::GetPoints(std::vector<double> container)
{
  std::vector<double> container_x;
  std::vector<double> container_y;

  for (int i = 0 ; i < container.size()/3; i++)
  {
    container_x.push_back(container[3*i]);
    container_y.push_back(container[3*i+1]);
  }
  return std::make_pair(container_x, container_y);
}



bool SlamGMapping::SetHypothesisParam()
{
  std::string my_path = log_path + "/hypothesis_config.yaml";
  YAML::Node hypothesis_config = YAML::LoadFile(my_path);
  double weight     = hypothesis_config["weight"].as<double>();


  // adjust the param setting (first tune)
  // double weight_max = 1.8;
  // double weight_min = 0.2;
  // double weight_add = 0.2;

  //compute average error
  float sum_odom = 0;

  for (int i=0; i<AE_best_odom.size(); i++)
    sum_odom += AE_best_odom[i];

  sum_odom = sum_odom/AE_best_odom.size();

  MSE_best_odom_x = MSE_best_odom_x/AE_best_odom.size()*2;
  MSE_best_odom_y = MSE_best_odom_y/AE_best_odom.size()*2;
  MSE_best_odom_sum = MSE_best_odom_sum/AE_best_odom.size();

  std::fstream logFile_hypothesis;

  // Open File
  std::string hypothesis_path  = log_path + "/data/Hypothesis/log_" + std::to_string(weight) + ".txt";

  logFile_hypothesis.open( hypothesis_path, std::ios::app);

  //Write data into log file
  logFile_hypothesis << "weight:"      + std::to_string(weight);
  // logFile_hypothesis << " yawdd:" + std::to_string(std_yawdd_);
  // logFile_hypothesis << " laspx:" + std::to_string(std_laspx_);
  // logFile_hypothesis << " laspy:" + std::to_string(std_laspy_);

  logFile_hypothesis << "---avg:"   + std::to_string(sum_odom);
  logFile_hypothesis << " mseX:"    + std::to_string(MSE_best_odom_x);
  logFile_hypothesis << " mseY:"    + std::to_string(MSE_best_odom_y);
  logFile_hypothesis << " mseS:"    + std::to_string(MSE_best_odom_sum);
  logFile_hypothesis << "\n";
  // close file stream
  logFile_hypothesis.close();

  // if (std_a_ >= a_min && std_a_ < a_max)
  //   plicp_config["std_a_"] = std_a_ + a_add;
  // else 
  // {
  //   plicp_config["std_a_"] = a_min;

  //   if (std_yawdd_ >= yawdd_min && std_yawdd_ < yawdd_max)
  //     plicp_config["std_yawdd_"] = std_yawdd_ + yawdd_add;
  //   else
  //   {
  //     plicp_config["std_yawdd_"] = yawdd_min;

  //     if (std_laspx_ >= laspx_min && std_laspx_ < laspx_max)
  //       plicp_config["std_laspx_"] = std_laspx_ + laspx_add;
  //     else
  //     {
  //       plicp_config["std_laspx_"] = laspx_min;
  //       if (std_laspy_ >= laspy_min && std_laspy_ < laspy_max)
  //         plicp_config["std_laspy_"] = std_laspy_ + laspy_add;
  //       else
  //         return false;
  //     }
  //   }
  // }

  // std::ofstream fout(my_path);
  // YAML::Emitter hypothesis_emitter;
  // hypothesis_emitter << hypothesis_config;
  // fout << hypothesis_emitter.c_str();
  // fout.close();
  return true;
}


void SlamGMapping::SaveResidual()
{
  if (sum_res < residual_sum)
    sum_res++;
    
  ORB_res(count_res, 0) = orb_ukf.y_(0);
  ORB_res(count_res, 1) = orb_ukf.y_(1);

  PLICP_res(count_res, 0) = plicp_ukf.y_(0);
  PLICP_res(count_res, 1) = plicp_ukf.y_(1);

  count_res++;

  if (count_res > 9)
    count_res = 0;

}

void SlamGMapping::HypothesisTesting()
{
  VectorXd orb_avg = VectorXd::Zero(2);
  VectorXd plicp_avg = VectorXd::Zero(2);


  // count the sum of every residual
  for (int i=0; i<sum_res; i++)
  {
    orb_avg(0) += ORB_res(i, 0);
    orb_avg(1) += ORB_res(i, 1);

    plicp_avg(0) += PLICP_res(i, 0);
    plicp_avg(1) += PLICP_res(i, 1);
  }

  // count the average of every residual
  orb_avg(0) = orb_avg(0)/sum_res;
  orb_avg(1) = orb_avg(1)/sum_res;

  plicp_avg(0) = plicp_avg(0)/sum_res;
  plicp_avg(1) = plicp_avg(1)/sum_res;

  VectorXd orb_std = VectorXd::Zero(2);
  VectorXd plicp_std = VectorXd::Zero(2);


  // count the sum of the squares of every residual minus average residual
  for (int i=0; i<sum_res; i++)
  {
    orb_std(0) += (ORB_res(i, 0) - orb_avg(0)) * (ORB_res(i, 0) - orb_avg(0));
    orb_std(1) += (ORB_res(i, 1) - orb_avg(1)) * (ORB_res(i, 1) - orb_avg(1));

    plicp_std(0) += (PLICP_res(i, 0) - plicp_avg(0)) * (PLICP_res(i, 0) - plicp_avg(0));
    plicp_std(1) += (PLICP_res(i, 1) - plicp_avg(1)) * (PLICP_res(i, 1) - plicp_avg(1));
  }

  // compute the Standard Deviation (in this part, I don't use (n-1), I use (n) to instead)
  orb_std(0) = sqrt(orb_std(0)/sum_res);
  orb_std(1) = sqrt(orb_std(1)/sum_res);

  plicp_std(0) = sqrt(plicp_std(0)/sum_res);
  plicp_std(1) = sqrt(plicp_std(1)/sum_res);

  VectorXd orb_z = VectorXd::Zero(2);
  VectorXd plicp_z = VectorXd::Zero(2);

  float sqrt_sum = sqrt(sum_res);

  for (int i=0; i<sum_res; i++)
  {
    orb_z(0) += (ORB_res(i, 0) - orb_avg(0)) / (orb_std(0)/sqrt_sum);
    orb_z(1) += (ORB_res(i, 1) - orb_avg(1)) / (orb_std(1)/sqrt_sum);


    plicp_z(0) += (PLICP_res(i, 0) - plicp_avg(0)) / (plicp_std(0)/sqrt_sum);
    plicp_z(1) += (PLICP_res(i, 1) - plicp_avg(1)) / (plicp_std(1)/sqrt_sum);
  }

  // std::cerr << orb_z(0) << "  " << orb_z(1) << std::endl;
  // std::cerr << plicp_z(0) << "  " << plicp_z(1) << std::endl;

  AdjustWeight(orb_z, plicp_z, 0);
  AdjustWeight(orb_z, plicp_z, 1);

}

void SlamGMapping::AdjustWeight(VectorXd &a, VectorXd &b, int x)
{
  // weight can't lower than 0
  if ( abs(a(x)) >= abs(b(x)) && ORB_weight(x) > 0)
  {
    PLICP_weight(x) += 0.01;
    ORB_weight(x) -= 0.01;
  }
  else if( abs(a(x)) < abs(b(x)) && PLICP_weight(x) > 0)
  {
    PLICP_weight(x) -= 0.01;
    ORB_weight(x) += 0.01;
  }
}
