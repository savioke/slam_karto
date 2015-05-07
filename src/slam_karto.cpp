/*
 * slam_karto
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

/**

@mainpage karto_gmapping

@htmlinclude manifest.html

*/

#include "ros/ros.h"
#include "ros/console.h"
#include "message_filters/subscriber.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "visualization_msgs/MarkerArray.h"

#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"

#include <open_karto/Mapper.h>

#include <slam_karto/slam_solver.h>
#include <slam_karto/spa_solver.h>

#include <boost/thread.hpp>

#include <string>
#include <map>
#include <vector>

#include <pluginlib/class_loader.h>
#include <slam_karto/localized_range_scan_stamped.h>

#include <std_msgs/Float64.h>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class SlamKarto
{
  public:
    SlamKarto();
    ~SlamKarto();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);

  private:
    bool getOdomPose(karto::Pose2& karto_pose, const ros::Time& t);
    karto::LaserRangeFinder* getLaser(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool addScan(karto::LaserRangeFinder* laser,
                 const sensor_msgs::LaserScan::ConstPtr& scan,
                 karto::Pose2& karto_pose);
    bool updateMap();
    void publishTransform();
    void publishLoop(double transform_publish_period);
    void visLoop(double vis_publish_period);
    void publishGraphVisualization();

    // timing stuff to support calibration
    bool init_time_;
    ros::Time time_offset_;

    // ROS handles
    ros::NodeHandle node_;
    tf::TransformListener tf_;
    tf::TransformBroadcaster* tfB_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    ros::Publisher sst_;
    ros::Publisher marker_publisher_;
    ros::Publisher sstm_;
    ros::Publisher response_publisher_;

    ros::ServiceServer ss_;

    // The map that will be published / send to service callers
    nav_msgs::GetMap::Response map_;

    // Storage for ROS parameters
    std::string odom_frame_;
    std::string map_frame_;
    std::string base_frame_;
    int throttle_scans_;
    ros::Duration map_update_interval_;
    double resolution_;
    boost::mutex map_mutex_;
    boost::mutex map_to_odom_mutex_;

    // ROS parameters copied to karto
    bool use_scan_matching_;
    bool use_scan_barycenter_;
    double min_travel_distance_;
    double min_travel_heading_;
    int scan_buffer_size_;
    double scan_buffer_max_scan_distance_;
    double link_match_min_response_fine_;
    double link_scan_max_distance_;
    double loop_search_max_distance_;
    bool do_loop_closing_;
    int loop_match_min_chain_size_;
    double loop_match_max_variance_coarse_;
    double loop_match_min_response_coarse_;
    double loop_match_min_response_fine_;
    double corr_search_space_dim_;
    double corr_search_space_res_;
    double corr_search_space_smear_dev_;
    double loop_search_space_dim_;
    double loop_search_space_res_;
    double loop_search_space_smear_dev_;
    double dist_var_penalty_;
    double angle_var_penalty_;
    double fine_search_angle_offset_;
    double coarse_search_angle_offset_;
    double coarse_angle_resolution_;
    double minimum_angle_penalty_;
    double minimum_dist_penalty_;
    bool use_response_expansion_;

    std::string solver_type_;
    
    // Karto bookkeeping
    karto::Mapper* mapper_;
    karto::Dataset* dataset_;
    //SpaSolver* solver_;
    pluginlib::ClassLoader<karto::SLAMSolver> solver_loader_;
    boost::shared_ptr<karto::SLAMSolver> solver_;
    std::map<std::string, karto::LaserRangeFinder*> lasers_;
    std::map<std::string, bool> lasers_inverted_;

    // Internal state
    bool got_map_;
    int laser_count_;
    boost::thread* transform_thread_;
    boost::thread* vis_thread_;
    boost::thread* response_thread_;
    tf::Transform map_to_odom_;
    unsigned marker_count_;
    bool inverted_laser_;

    // Cosmetic
    bool got_initial_pose_;
    karto::Pose2 initial_pose_;
    tf::Stamped<tf::Pose> initial_odom_;

};

SlamKarto::SlamKarto() :
        got_map_(false),
        laser_count_(0),
        transform_thread_(NULL),
        vis_thread_(NULL),
        response_thread_(NULL),
        marker_count_(0),
        solver_loader_("slam_karto", "karto::SLAMSolver"),
        tf_(ros::Duration(10000)),
        init_time_(false),
        mapper_(NULL)
{
  got_initial_pose_ = false;
  map_to_odom_.setIdentity();
  // Retrieve parameters
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
 
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";

  ROS_INFO("SLAM map_frame: %s, odom_frame: %s, base_frame: %s", map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());

  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  if(!private_nh_.getParam("resolution", resolution_))
  {
    // Compatibility with slam_gmapping, which uses "delta" to mean
    // resolution
    if(!private_nh_.getParam("delta", resolution_))
      resolution_ = 0.05;
  }
  double transform_publish_period;
  private_nh_.param("transform_publish_period", transform_publish_period, 0.05);
  double vis_publish_period;
  private_nh_.param("vis_publish_period", vis_publish_period, 2.0);
  double response_publish_period;
  private_nh_.param("response_publish_period", response_publish_period, 1.0);

  if(!private_nh_.getParam("solver_type", solver_type_))
    solver_type_="SPA";

  // Set up advertisements and subscriptions
  tfB_ = new tf::TransformBroadcaster();
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamKarto::mapCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 50000);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 50000);
  scan_filter_->registerCallback(boost::bind(&SlamKarto::laserCallback, this, _1));
  marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",1);

  // Publish the most recent loop closure response
  response_publisher_ = node_.advertise<std_msgs::Float64>("loop_closure_response",2);

  // Create a thread to periodically publish the latest map->odom
  // transform; it needs to go out regularly, uninterrupted by potentially
  // long periods of computation in our main loop.
  transform_thread_ = new boost::thread(boost::bind(&SlamKarto::publishLoop, this, transform_publish_period));

  // Initialize Karto structures
  mapper_ = new karto::Mapper();
  dataset_ = new karto::Dataset();
  
  // Karto parameters
  if(private_nh_.getParam("use_scan_matching", use_scan_matching_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter use_scan_matching to %d", use_scan_matching_);
    static_cast<karto::Parameter<bool> *>(mapper_->GetParameterManager()->Get("UseScanMatching"))->SetValue(use_scan_matching_);
  }
  if(private_nh_.getParam("use_scan_barycenter", use_scan_barycenter_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter use_scan_barycenter to %d", use_scan_barycenter_);
    static_cast<karto::Parameter<bool> *>(mapper_->GetParameterManager()->Get("UseScanBarycenter"))->SetValue(use_scan_barycenter_);
  }
  if(private_nh_.getParam("minimum_travel_distance", min_travel_distance_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter MinimumTravelDistance to %f", min_travel_distance_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("MinimumTravelDistance"))->SetValue(min_travel_distance_);
  }
  if(private_nh_.getParam("minimum_travel_heading", min_travel_heading_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter MinimumTravelHeading to %f", min_travel_heading_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("MinimumTravelHeading"))->SetValue(min_travel_heading_);
  }
  if(private_nh_.getParam("scan_buffer_size", scan_buffer_size_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter ScanBufferSize to %d", scan_buffer_size_);
    static_cast<karto::Parameter<int> *>(mapper_->GetParameterManager()->Get("ScanBufferSize"))->SetValue(scan_buffer_size_);
  }
  if(private_nh_.getParam("scan_buffer_max_scan_distance", scan_buffer_max_scan_distance_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter ScanBufferMaximumScanDistance to %f", scan_buffer_max_scan_distance_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("ScanBufferMaximumScanDistance"))->SetValue(scan_buffer_max_scan_distance_);
  }
  if(private_nh_.getParam("link_match_min_response_fine", link_match_min_response_fine_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter LinkMatchMinimumResposeFine to %f", link_match_min_response_fine_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("LinkMatchMinimumResponseFine"))->SetValue(link_match_min_response_fine_);
  }
  if(private_nh_.getParam("link_scan_max_distance", link_scan_max_distance_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter LinkScanMaximumDistance to %f", link_scan_max_distance_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("LinkScanMaximumDistance"))->SetValue(link_scan_max_distance_);
  }
  if(private_nh_.getParam("loop_search_max_distance", loop_search_max_distance_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter LoopSearchMaximumDistance to %f", loop_search_max_distance_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("LoopSearchMaximumDistance"))->SetValue(loop_search_max_distance_);
  }
  if(private_nh_.getParam("do_loop_closing", do_loop_closing_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter DoLoopClosing to %d", do_loop_closing_);
    static_cast<karto::Parameter<bool> *>(mapper_->GetParameterManager()->Get("DoLoopClosing"))->SetValue(do_loop_closing_);
  }
  if(private_nh_.getParam("loop_match_min_chain_size", loop_match_min_chain_size_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter LoopMatchMinimumChainSize to %d", loop_match_min_chain_size_);
    static_cast<karto::Parameter<int> *>(mapper_->GetParameterManager()->Get("LoopMatchMinimumChainSize"))->SetValue(loop_match_min_chain_size_);
  }
  if(private_nh_.getParam("loop_match_max_variance_coarse", loop_match_max_variance_coarse_))
  {
    ROS_DEBUG_NAMED("slam_karto", "Setting karto parameter LoopMatchMaximumVarianceCoarse to %f", loop_match_max_variance_coarse_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("LoopMatchMaximumVarianceCoarse"))->SetValue(loop_match_max_variance_coarse_);
  }
  if(private_nh_.getParam("loop_match_min_response_coarse", loop_match_min_response_coarse_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter LoopMatchMinimumResponseCoarse to %f", loop_match_min_response_coarse_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("LoopMatchMinimumResponseCoarse"))->SetValue(loop_match_min_response_coarse_);
  }
  if(private_nh_.getParam("loop_match_min_response_fine", loop_match_min_response_fine_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter LoopMatchMinimumResponseFine to %f", loop_match_min_response_fine_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("LoopMatchMinimumResponseFine"))->SetValue(loop_match_min_response_fine_);
  }
  if(private_nh_.getParam("corr_search_space_dim", corr_search_space_dim_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter CorrelationSearchSpaceDimension to %f", corr_search_space_dim_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("CorrelationSearchSpaceDimension"))->SetValue(corr_search_space_dim_);
  }
  if(private_nh_.getParam("corr_search_space_res", corr_search_space_res_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter CorrelationSearchSpaceResolution to %f", corr_search_space_res_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("CorrelationSearchSpaceResolution"))->SetValue(corr_search_space_res_);
  }
  if(private_nh_.getParam("corr_search_space_smear_dev", corr_search_space_smear_dev_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter CorrelationSearchSpaceSmearDeviation to %f", corr_search_space_smear_dev_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("CorrelationSearchSpaceSmearDeviation"))->SetValue(corr_search_space_smear_dev_);
  }
  if(private_nh_.getParam("loop_search_space_dim", loop_search_space_dim_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter LoopSearchSpaceDimension to %f", loop_search_space_dim_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("LoopSearchSpaceDimension"))->SetValue(loop_search_space_dim_);
  }
  if(private_nh_.getParam("loop_search_space_res", loop_search_space_res_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter LoopSearchSpaceResolution to %f", loop_search_space_res_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("LoopSearchSpaceResolution"))->SetValue(loop_search_space_res_);
  }
  if(private_nh_.getParam("loop_search_space_smear_dev", loop_search_space_smear_dev_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter LoopSearchSpaceSmearDeviation to %f", loop_search_space_smear_dev_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("LoopSearchSpaceSmearDeviation"))->SetValue(loop_search_space_smear_dev_);
  }
  if(private_nh_.getParam("distance_variance_penalty", dist_var_penalty_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter DistanceVariancePenalty to %f", dist_var_penalty_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("DistanceVariancePenalty"))->SetValue(dist_var_penalty_);
  }
  if(private_nh_.getParam("angle_variance_penalty", angle_var_penalty_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter AngleVariancePenalty to %f", angle_var_penalty_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("AngleVariancePenalty"))->SetValue(angle_var_penalty_);
  }
  if(private_nh_.getParam("fine_search_angle_offset", fine_search_angle_offset_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter FineSearchAngleOffset to %f", fine_search_angle_offset_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("FineSearchAngleOffset"))->SetValue(fine_search_angle_offset_);
  }
  if(private_nh_.getParam("coarse_search_angle_offset", coarse_search_angle_offset_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter CoarseSearchAngleOffset to %f", coarse_search_angle_offset_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("CoarseSearchAngleOffset"))->SetValue(coarse_search_angle_offset_);
  }
  if(private_nh_.getParam("coarse_angle_resolution", coarse_angle_resolution_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter CoarseAngleResolution to %f", coarse_angle_resolution_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("CoarseAngleResolution"))->SetValue(coarse_angle_resolution_);
  }
  if(private_nh_.getParam("minimum_angle_penalty", minimum_angle_penalty_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter MinimumAnglePenalty to %f", minimum_angle_penalty_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("MinimumAnglePenalty"))->SetValue(minimum_angle_penalty_);
  }
  if(private_nh_.getParam("minimum_dist_penalty", minimum_dist_penalty_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter MinimumDistancePenalty to %f", minimum_dist_penalty_);
    static_cast<karto::Parameter<double> *>(mapper_->GetParameterManager()->Get("MinimumDistancePenalty"))->SetValue(minimum_dist_penalty_);
  }
  if(private_nh_.getParam("use_response_expansion", use_response_expansion_))
  {
    ROS_DEBUG_NAMED("slam_karto","Setting karto parameter UseResponseExpansion to %d", use_response_expansion_);
    static_cast<karto::Parameter<bool> *>(mapper_->GetParameterManager()->Get("UseResponseExpansion"))->SetValue(use_response_expansion_);
  }

  // Set solver to be used in loop closure

  if(solver_type_ == "SPA")
  {
    try
    {
      solver_ = solver_loader_.createInstance("karto_plugins::SPASolver");
      ROS_INFO("Loaded SPA solver plugin");
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }
  }
  else if(solver_type_ == "G2O")
  {
    try
    {
      solver_ = solver_loader_.createInstance("karto_plugins::G2OSolver");
      ROS_INFO("Loaded G2O solver plugin");
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }
  }
  else if(solver_type_ == "MAXMIX")
  {
    try
    {
      solver_ = solver_loader_.createInstance("karto_plugins::VertigoMaxMixSolver");
      ROS_INFO("Loaded Vertigo MaxMix solver plugin");
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }
  }
  else if(solver_type_ == "SWITCHABLE")
  {
    try
    {
      solver_ = solver_loader_.createInstance("karto_plugins::VertigoSwitchableSolver");
      ROS_INFO("Loaded Vertigo Switchable solver plugin");
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }
  }
  mapper_->SetScanSolver(solver_.get());
  solver_->setFrameId(map_frame_);
  
  vis_thread_ = new boost::thread(boost::bind(&SlamKarto::visLoop, this, vis_publish_period));
}

SlamKarto::~SlamKarto()
{
  if(response_thread_)
  {
    response_thread_->join();
    delete response_thread_;
  }
  if(transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
  if(vis_thread_)
  {
    vis_thread_->join();
    delete vis_thread_;
  }
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
  if (solver_)
    delete solver_.get();
  if (mapper_)
    delete mapper_;
  if (dataset_)
    delete dataset_;
  // TODO: delete the pointers in the lasers_ map; not sure whether or not
  // I'm supposed to do that.
}

void
SlamKarto::publishLoop(double transform_publish_period)
{
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok())
  {
    publishTransform();
    r.sleep();
  }
}

void
SlamKarto::publishTransform()
{
  boost::mutex::scoped_lock(map_to_odom_mutex_);
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(0.05);
  tfB_->sendTransform(tf::StampedTransform (map_to_odom_, ros::Time::now(), map_frame_, odom_frame_));
}

karto::LaserRangeFinder*
SlamKarto::getLaser(const sensor_msgs::LaserScan::ConstPtr& scan)
{

  // Check whether we know about this laser yet
  if(lasers_.find(scan->header.frame_id) == lasers_.end())
  {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try
    {
      tf_.transformPose(base_frame_, ident, laser_pose);
    }
    catch(tf::TransformException e)
    {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
	       e.what());
      return NULL;
    }

    double yaw = tf::getYaw(laser_pose.getRotation());
   
    ROS_DEBUG_NAMED("slam_karto","laser %s's pose wrt base: %.3f %.3f %.3f",
	     scan->header.frame_id.c_str(),
	     laser_pose.getOrigin().x(),
	     laser_pose.getOrigin().y(),
	     yaw);
    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, scan->angle_min);
    tf::Stamped<tf::Quaternion> min_q(q, scan->header.stamp,
                                      scan->header.frame_id);
    q.setRPY(0.0, 0.0, scan->angle_max);
    tf::Stamped<tf::Quaternion> max_q(q, scan->header.stamp,
                                      scan->header.frame_id);
    try
    {
      tf_.transformQuaternion(base_frame_, min_q, min_q);
      tf_.transformQuaternion(base_frame_, max_q, max_q);
    }
    catch(tf::TransformException& e)
    {
      ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
               e.what());
      return NULL;
    }

    double angle_min = tf::getYaw(min_q);
    double angle_max = tf::getYaw(max_q);
    bool inverse =  lasers_inverted_[scan->header.frame_id] = angle_max < angle_min;
    if (inverse)
      ROS_DEBUG_NAMED("slam_karto","laser is mounted upside-down");


    // Create a laser range finder device and copy in data from the first
    // scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder* laser = 
      karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, karto::Name(name));
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(),
				      laser_pose.getOrigin().y(),
				      yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);

    // TODO: expose this, and many other parameters
    //laser_->SetRangeThreshold(12.0);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;

    // Add it to the dataset, which seems to be necessary
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

bool
SlamKarto::getOdomPose(karto::Pose2& karto_pose, const ros::Time& t)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                           tf::Vector3(0,0,0)), t, base_frame_);
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  if(!got_initial_pose_)
  {
    got_initial_pose_ = true;
    initial_pose_ = karto::Pose2(odom_pose.getOrigin().x(),
                       odom_pose.getOrigin().y(),
                       yaw);
    initial_odom_ = odom_pose;

  }

  karto_pose = 
          karto::Pose2(odom_pose.getOrigin().x(),
                       odom_pose.getOrigin().y(),
                       yaw) - initial_pose_;

  double x = karto_pose.GetX();
  double y = karto_pose.GetY();
  double th = initial_pose_.GetHeading();

  double x_dash = x*cos(th) + y*sin(th);
  double y_dash = -x*sin(th)+y*cos(th);

  karto_pose.SetX(x_dash);
  karto_pose.SetY(y_dash);
  karto_pose.SetHeading(yaw-th);

  return true;
}

/* Publish the graph */
void SlamKarto::publishGraphVisualization()
{
 visualization_msgs::MarkerArray marray;
 solver_->publishGraphVisualization(marray); 
 marker_publisher_.publish(marray);
}

void
SlamKarto::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0,0);

  // Check whether we know about this laser yet
  karto::LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN("Failed to create laser device for %s; discarding scan",
	     scan->header.frame_id.c_str());
    return;
  }

  karto::Pose2 odom_pose;
  if(addScan(laser, scan, odom_pose))
  {
    ROS_DEBUG_NAMED("slam_karto","added scan at pose: %.3f %.3f %.3f", 
              odom_pose.GetX(),
              odom_pose.GetY(),
              odom_pose.GetHeading());

    if(!got_map_ || 
       (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      if(updateMap())
      {
        last_map_update = scan->header.stamp;
        got_map_ = true;
        ROS_DEBUG("Updated the map");
      }
    }
  }
}

bool
SlamKarto::updateMap()
{
  boost::mutex::scoped_lock(map_mutex_);

  karto::OccupancyGrid* occ_grid = 
          karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);

  if(!occ_grid)
    return false;

  if(!got_map_) {
    map_.map.info.resolution = resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

  if(map_.map.info.width != (unsigned int) width || 
     map_.map.info.height != (unsigned int) height ||
     map_.map.info.origin.position.x != offset.GetX() ||
     map_.map.info.origin.position.y != offset.GetY())
  {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }
 
  for (kt_int32s y=0; y<height; y++)
  {
    for (kt_int32s x=0; x<width; x++) 
    {
      // Getting the value at position x,y
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value)
      {
        case karto::GridStates_Unknown:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
          break;
        case karto::GridStates_Occupied:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
          break;
        case karto::GridStates_Free:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
          break;
        default:
          ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
          break;
      }
    }
  }

  // Set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);

  delete occ_grid;

  return true;
}

bool
SlamKarto::addScan(karto::LaserRangeFinder* laser,
		   const sensor_msgs::LaserScan::ConstPtr& scan, 
                   karto::Pose2& karto_pose)
{
  if(!getOdomPose(karto_pose, scan->header.stamp))
     return false;
  
  // Create a vector of doubles for karto
  std::vector<kt_double> readings;

  if (lasers_inverted_[scan->header.frame_id]) {
    for(std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin();
      it != scan->ranges.rend();
      ++it)
    {
      readings.push_back(*it);
    }
  } else {
    for(std::vector<float>::const_iterator it = scan->ranges.begin();
      it != scan->ranges.end();
      ++it)
    {
      readings.push_back(*it);
    }
  }
  
  // create localized range scan
 
  if(!init_time_)
  {
    time_offset_ = scan->header.stamp;
    init_time_ = true;
  }
  double tStamp = (scan->header.stamp-time_offset_).toNSec();
  
  karto::LocalizedRangeScan* range_scan = 
    new karto::LocalizedRangeScanStamped(laser->GetName(), readings, tStamp/1e9);
  
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  // Add the localized range scan to the mapper
  bool processed;
  if((processed = mapper_->Process(range_scan)))
  {
    //std::cout << "Pose: " << range_scan->GetOdometricPose() << " Corrected Pose: " << range_scan->GetCorrectedPose() << std::endl;
    
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();

    // Compute the map->odom transform
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      tf_.transformPose(odom_frame_,tf::Stamped<tf::Pose> (tf::Transform(tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()), tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0)).inverse(), scan->header.stamp, base_frame_),odom_to_map);
    }
    catch(tf::TransformException e)
    {
      ROS_ERROR("Transform from base_link to odom failed\n");
      odom_to_map.setIdentity();
    }

    map_to_odom_mutex_.lock();
    map_to_odom_ = tf::Transform(tf::Quaternion( odom_to_map.getRotation() ),
                                 tf::Point(      odom_to_map.getOrigin() ) ).inverse();
    map_to_odom_mutex_.unlock();

    // Add the localized range scan to the dataset (for memory management)
    dataset_->Add(range_scan);
  }
  else
    delete range_scan;

  return processed;
}

bool 
SlamKarto::mapCallback(nav_msgs::GetMap::Request  &req,
                       nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock(map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

void
SlamKarto::visLoop(double vis_publish_period)
{
  if(vis_publish_period == 0)
    return;

  ros::Rate r(1.0 / vis_publish_period);
  while(ros::ok())
  {
    publishGraphVisualization();
    r.sleep();
  }
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_karto");

  SlamKarto kn;

  ros::spin();

  return 0;
}
