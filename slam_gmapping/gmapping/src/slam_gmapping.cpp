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
  // need to give last_odom_pose a initialpose by laser pose 
  last_odom_pose = initialPose;
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

  // using PLICP compute
  LDP curr_ldp_scan;
  // change scan type transform to csm type
  LaserScanToLDP(scan, curr_ldp_scan);
  // compute PL-ICP compute the transform
  ScanMatchWithPLICP(curr_ldp_scan, scan->header.stamp);

  GMapping::OrientedPoint icp_pose( last_odom_pose.x + output_.x[0],
                                    last_odom_pose.y + output_.x[1],
                                    last_odom_pose.theta + output_.x[2]);

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

    GMapping::OrientedPoint lcp_current_pose;

    if(addScan(*scan, odom_pose))
    {
      // std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
      // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
      // std::cout << "\naaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa: " << time_used.count() << " s " << std::endl;
      // std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

      ROS_DEBUG("scan processed");
      GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;


      std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
      std::cerr << mpose.x << "  " << mpose.y << "  " << mpose.theta << std::endl;

      ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
      ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
      ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

      // laser_to_map : lidar pose in map frame
      // odom_to_laser : lidar pose in odom frame
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
      last_icp_time_ = scan->header.stamp;
      got_first_scan_ = true;
    }

    // odom_pose : laser pose in odom frame
    // MAYBE : odom_pose is computed by motor motion
    GMapping::OrientedPoint odom_pose;

    GMapping::OrientedPoint lcp_current_pose;

    if (ScanCallback(scan, odom_pose))
    {
      // std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
      // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
      // std::cout << "\naaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa: " << time_used.count() << " s " << std::endl;
      // std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

      ROS_DEBUG("scan processed");

      // std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
      // std::cerr << odom->pose.pose.position.x << " " << odom->pose.pose.position.y << " " << -2 * acos(odom->pose.pose.orientation.w) / 3.14159 * 180.0 << std::endl;

      // std::cerr << odom_pose.x << "  " << odom_pose.y << "  " << odom_pose.theta << std::endl;
      // std::cerr <<  last_odom_pose.x + output_.x[0] << "  " <<  last_odom_pose.y + output_.x[1] << "  " <<  last_odom_pose.theta + output_.x[2] << std::endl;
      // std::cerr << output_.x[0] << "  " << output_.x[1] << "  " << output_.x[2]*180/M_PI << std::endl;
      // std::cerr << "ddddddddddddddddddddddddddddd" << std::endl;

      // std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
      // std::cerr << odom->pose.pose.position.x << " " << odom->pose.pose.position.y << " " << -2 * acos(odom->pose.pose.orientation.w) / 3.14159 * 180.0 << std::endl;
      // std::cerr << lcp_current_pose.x << " " << lcp_current_pose.y << " " << lcp_current_pose.theta << std::endl;
      // std::cerr << odom_pose.x << " " << odom_pose.y << " " << odom_pose.theta << std::endl;


      lcp_current_pose.x = last_odom_pose.x + output_.x[0];
      lcp_current_pose.y = last_odom_pose.y + output_.x[1];
      lcp_current_pose.theta = last_odom_pose.theta + output_.x[2];
      last_odom_pose = odom_pose;


      Precision_PLICP(odom, lcp_current_pose.x, lcp_current_pose.y);


      ROS_DEBUG("new best pose: %.3f %.3f %.3f", lcp_current_pose.x, lcp_current_pose.y, lcp_current_pose.theta);
      ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
      ROS_DEBUG("correction: %.3f %.3f %.3f", output_.x[0], output_.x[1], output_.x[2]);


      // laser_to_map : lidar pose in map frame
      // odom_to_laser : lidar pose in odom frame
      tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, lcp_current_pose.theta), tf::Vector3(lcp_current_pose.x, lcp_current_pose.y, 0.0)).inverse();
      tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

      // map_to_odom_ : compute relationship between odom frame and map frame
      map_to_odom_mutex_.lock();
      map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
      map_to_odom_mutex_.unlock();

      // map update will follow map_update_interval_ this parameter
      if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
      {
        std::cerr << "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz" << std::endl;
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
        last_icp_time_ = scan->header.stamp;
        last_ORB_pose.x = srv.response.x;
        last_ORB_pose.y = srv.response.y;
        last_ORB_pose.theta = srv.response.z;


        // UKF params
        MeasurementPackage meas_init_pose;
        meas_init_pose.raw_measurements_ = VectorXd(2);
        meas_init_pose.raw_measurements_ << 0, 0;

        // meas_init_pose.raw_measurements_ = VectorXd(3);
        // meas_init_pose.raw_measurements_ << 0, 0, 0;
        meas_init_pose.timestamp_ = srv.request.sec * 1000000000 + srv.request.nsec;
        meas_init_pose.sensor_type_ = MeasurementPackage::LASER;

        // init ORBSLAM2 and PLICP parameter
        orb_ukf.SetUKFParam(1);
        plicp_ukf.SetUKFParam(2);

        got_first_scan_ = orb_ukf.ProcessMeasurement(meas_init_pose);
        got_first_scan_ = plicp_ukf.ProcessMeasurement(meas_init_pose);

        got_first_scan_ = true;
      }

      // odom_pose : laser pose in odom frame
      // MAYBE : odom_pose is computed by motor motion
      GMapping::OrientedPoint odom_pose;

      GMapping::OrientedPoint lcp_current_pose;
      GMapping::OrientedPoint ORB_current_change;

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
 
        // std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
        // std::cerr << odom_pose.x << "  " << odom_pose.y << "  " << odom_pose.theta << std::endl;
        // std::cerr <<  last_odom_pose.x + output_.x[0] << "  " <<  last_odom_pose.y + output_.x[1] << "  " <<  last_odom_pose.theta + output_.x[2] << std::endl;
        // std::cerr << output_.x[0] << "  " << output_.x[1] << "  " << output_.x[2] << std::endl;
        // std::cerr << "ddddddddddddddddddddddddddddd" << std::endl;

        /////////     ORB and PLICP　current　change by GMapping::OrientedPoint type   /////////
        lcp_current_pose.x = last_odom_pose.x + output_.x[0];
        lcp_current_pose.y = last_odom_pose.y + output_.x[1];
        lcp_current_pose.theta = last_odom_pose.theta + output_.x[2];
        
        // ORB_current_change.x = srv.response.x - last_ORB_pose.x;
        // ORB_current_change.y = srv.response.y - last_ORB_pose.y;
        // ORB_current_change.theta = srv.response.z - last_ORB_pose.theta;

        last_ORB_pose.x = srv.response.x;
        last_ORB_pose.y = srv.response.y;
        last_ORB_pose.theta = srv.response.z;

        Precision_ORB(odom, srv.response.x, srv.response.y);
        // Precision_PLICP(odom, lcp_current_pose.x,  lcp_current_pose.y);





        // put ORB pose variety and PLICP pose variety into Unscented Kalman Filter
        // ORB pose part
        meas_ORB_pose.raw_measurements_ = VectorXd(2);
        // meas_ORB_pose.raw_measurements_ << ORB_current_change.x, ORB_current_change.y;
        meas_ORB_pose.raw_measurements_ << srv.response.x, srv.response.y;

        // meas_ORB_pose.raw_measurements_ = VectorXd(3);
        // meas_ORB_pose.raw_measurements_ << ORB_current_change.x, ORB_current_change.y, ORB_current_change.theta;

        meas_ORB_pose.timestamp_ = srv.request.sec * 1000000000 + srv.request.nsec;
        meas_ORB_pose.sensor_type_ = MeasurementPackage::LASER;


        // PLICP pose part
        meas_PLICP_pose.raw_measurements_ = VectorXd(2);
        // meas_PLICP_pose.raw_measurements_ << output_.x[0], output_.x[1];
        meas_PLICP_pose.raw_measurements_ << lcp_current_pose.x, lcp_current_pose.y;

        // meas_PLICP_pose.raw_measurements_ = VectorXd(3);
        // meas_PLICP_pose.raw_measurements_ << output_.x[0], output_.x[1], output_.x[2];

        meas_PLICP_pose.timestamp_ = srv.request.sec * 1000000000 + srv.request.nsec;
        meas_PLICP_pose.sensor_type_ = MeasurementPackage::LASER;

        // std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;

        bool flag_orb, flag_plicp;
        flag_orb = orb_ukf.ProcessMeasurement(meas_ORB_pose);
        flag_plicp = plicp_ukf.ProcessMeasurement(meas_PLICP_pose);

        // std::cerr << ORB_current_change.x << "  " << PLICP_weight(0) << std::endl;
        // std::cerr << ORB_current_change.y << "  " << PLICP_weight(1) << std::endl;
        // std::cerr << ORB_current_change.theta << "  " << PLICP_weight(1) << std::endl;

        // std::cerr << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;

        // std::cerr << output_.x[0] << "  " << output_.x[1] << "  " << output_.x[2] << std::endl;
        // std::cerr << ORB_current_change.x << "  " << ORB_current_change.y << "  " << ORB_current_change.theta << std::endl;


        //  std::cerr << srv.response.x << "  " << srv.response.y << "  " << srv.response.z << std::endl;

        if (flag_orb == true && flag_plicp == true)
        {
          // std::cerr << "asasasasasasasasasasasasasasasaasas" << std::endl;

          // save the residual
          SaveResidual();
          HypothesisTesting();

          // std::cerr << ORB_weight(0) << "  " << PLICP_weight(0) << std::endl;
          // std::cerr << ORB_weight(1) << "  " << PLICP_weight(1) << std::endl;

        }

        // best_pose.x = last_odom_pose.x + ORB_weight(0)*orb_ukf.x_[0] + PLICP_weight(0)*plicp_ukf.x_[0];
        // best_pose.y = last_odom_pose.y + ORB_weight(1)*orb_ukf.x_[1] + PLICP_weight(1)*plicp_ukf.x_[1];
        best_pose.x = ORB_weight(0)*orb_ukf.x_[0] + PLICP_weight(0)*plicp_ukf.x_[0];
        best_pose.y = ORB_weight(1)*orb_ukf.x_[1] + PLICP_weight(1)*plicp_ukf.x_[1];
        best_pose.theta = odom_pose.theta;
        // best_pose.theta = last_odom_pose.theta + ORB_weight(2)*orb_ukf.x_[2] + PLICP_weight(2)*plicp_ukf.x_[2];
      
        // best_pose.x = last_odom_pose.x + ORB_weight(0)*ORB_current_change.x + PLICP_weight(0)*output_.x[0];
        // best_pose.y = last_odom_pose.y + ORB_weight(1)*ORB_current_change.y + PLICP_weight(1)*output_.x[1];
        // best_pose.theta = last_odom_pose.theta + ORB_weight(2)*ORB_current_change.theta + PLICP_weight(2)*output_.x[2];

        last_odom_pose = odom_pose;


        ROS_DEBUG("new best pose: %.3f %.3f %.3f", best_pose.x, best_pose.y, lcp_current_pose.theta);
        ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
        ROS_DEBUG("correction: %.3f %.3f %.3f", output_.x[0], output_.x[1], output_.x[2]);

        // laser_to_map : lidar pose in map frame
        // odom_to_laser : lidar pose in odom frame
        tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, best_pose.theta), tf::Vector3(best_pose.x, best_pose.y, 0.0)).inverse();
        tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

        // map_to_odom_ : compute relationship between odom frame and map frame
        map_to_odom_mutex_.lock();
        map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
        map_to_odom_mutex_.unlock();

        // map update will follow map_update_interval_ this parameter
        if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
        {
          std::cerr << "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz" << std::endl;
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

    ldp->estimate[0] = 0.0;
    ldp->estimate[1] = 0.0;
    ldp->estimate[2] = 0.0;

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
        input_.max_angular_correction_deg = 45.0;
        // input_.max_angular_correction_deg = 360.0;

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

    input_.laser_ref = prev_ldp_scan_;
    input_.laser_sens = curr_ldp_scan;

    // 位姿的预测值为0，就是不进行预测
    input_.first_guess[0] = 0;
    input_.first_guess[1] = 0;
    input_.first_guess[2] = 0;

    // 调用csm里的函数进行plicp计算帧间的匹配，输出结果保存在output里
    sm_icp(&input_, &output_);

    // if (output_.valid)
    // {
    //     std::cout << "transfrom: (" << output_.x[0] << ", " << output_.x[1] << ", " 
    //         << output_.x[2] * 180 / M_PI << ")" << std::endl;
    // }
    // else
    // {
    //     std::cout << "not Converged" << std::endl;
    // }

    // 删除prev_ldp_scan_，用curr_ldp_scan进行替代
    ld_free(prev_ldp_scan_);
    prev_ldp_scan_ = curr_ldp_scan;
    last_icp_time_ = time;
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

  double precision = 0.0;

  for (int i=0; i<=AE_PLICP.size(); i++)
  {
    precision += AE_PLICP[i];
  }


  // std::cerr << odom->pose.pose.position.x << " " << odom->pose.pose.position.y << " " << -2 * acos(odom->pose.pose.orientation.w) / 3.14159 * 180.0 << std::endl;

  // std::cerr << "PLICP =========================" << std::endl;
  // std::cerr << precision/AE_PLICP.size() << std::endl;

}
// compute precision of ORB
void SlamGMapping::Precision_ORB(const nav_msgs::Odometry::ConstPtr& odom, double x, double y)
{
  AE_ORBSLAM.push_back( abs(odom->pose.pose.position.x - x ));
  AE_ORBSLAM.push_back( abs(odom->pose.pose.position.y - y ));

  // double precision = 0.0;

  // for (int i=0; i<=AE_ORBSLAM.size(); i++)
  // {
  //   precision += AE_ORBSLAM[i];
  // }
  
  std::cerr << "ORB =========================" << std::endl;
  // std::cerr << precision/AE_ORBSLAM.size() << std::endl;
  std::cerr << AE_ORBSLAM.size() << std::endl;
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

  for (int i=0; i<sum_res; i++)
  {
    orb_avg(0) += ORB_res(i, 0);
    orb_avg(1) += ORB_res(i, 1);

    plicp_avg(0) += PLICP_res(i, 0);
    plicp_avg(1) += PLICP_res(i, 1);
  }

  orb_avg(0) = orb_avg(0)/sum_res;
  orb_avg(1) = orb_avg(1)/sum_res;

  plicp_avg(0) = plicp_avg(0)/sum_res;
  plicp_avg(1) = plicp_avg(1)/sum_res;

  VectorXd orb_std = VectorXd::Zero(2);
  VectorXd plicp_std = VectorXd::Zero(2);

  for (int i=0; i<sum_res; i++)
  {
    orb_std(0) += (ORB_res(i, 0) - orb_avg(0)) * (ORB_res(i, 0) - orb_avg(0));
    orb_std(1) += (ORB_res(i, 1) - orb_avg(1)) * (ORB_res(i, 1) - orb_avg(1));

    plicp_std(0) += (PLICP_res(i, 0) - plicp_avg(0)) * (PLICP_res(i, 0) - plicp_avg(0));
    plicp_std(1) += (PLICP_res(i, 1) - plicp_avg(1)) * (PLICP_res(i, 1) - plicp_avg(1));
  }

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
