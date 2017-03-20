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
#include <std_srvs/Empty.h>

#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"

#include <open_karto/Mapper.h>

#include <slam_karto/spa_solver.h>
#include <slam_karto/spa_graph_visualizer.h>

#include <boost/thread.hpp>

#include <string>
#include <map>
#include <vector>

#include <pluginlib/class_loader.h>

// Dataset serialization
#include <fstream>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class SlamKarto
{
  public:
    SlamKarto();
    ~SlamKarto();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void setOldMapToBaseLinkCallback(const geometry_msgs::Pose::ConstPtr& pose);
    void setOldMapToOldOdomCallback(const geometry_msgs::Pose::ConstPtr& pose);
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);
    bool rebuildGraphCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool saveDatasetCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

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
    void initializeMapper(ros::NodeHandle& private_nh,boost::shared_ptr<karto::ScanSolver> solver, karto::Mapper* mapper);
    bool offsetOdometricPosesForNewMapFrame();
    tf::Transform kartoPose2ToTfTransform(const karto::Pose2& karto_pose);
    karto::Pose2 tfTransformToKartoPose2(const tf::Transform& transform);
    std::string getDatasetSummary();

    // ROS handles
    ros::NodeHandle node_,private_nh_;
    tf::TransformListener tf_;
    tf::TransformBroadcaster* tfB_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    ros::Publisher sst_;
    ros::Publisher marker_publisher_;
    ros::Publisher sstm_;
    ros::Subscriber set_old_map_to_base_link_sub_;
    ros::Subscriber set_old_map_to_old_odom_sub_;
    ros::ServiceServer ss_, remove_vertices_ss_, rebuild_graph_ss_, save_dataset_ss_;

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
    
    std::string solver_type_;
    std::string visualizer_type_;

    // Karto bookkeeping
    karto::Mapper* mapper_;
    karto::Dataset* dataset_;
    pluginlib::ClassLoader<karto::ScanSolver> solver_loader_;
    pluginlib::ClassLoader<karto::GraphVisualizer> visualizer_loader_;
    boost::shared_ptr<karto::ScanSolver> solver_;
    boost::shared_ptr<karto::GraphVisualizer> visualizer_;
    std::map<std::string, karto::LaserRangeFinder*> lasers_;
    std::map<std::string, bool> lasers_inverted_;

    // Internal state
    bool got_map_;
    int laser_count_;
    boost::thread* transform_thread_;
    boost::thread* vis_thread_;
    tf::Transform map_to_odom_;
    bool inverted_laser_;
    karto::Pose2 old_map_to_base_link_;
    karto::Pose2 old_map_to_old_odom_;
    bool old_map_to_base_link_set_;
    bool old_map_to_old_odom_set_;

    std::string input_dataset_file_;
    std::string output_dataset_file_;

    bool dont_add_new_nodes_before_graph_is_restored_;
    double minimum_travel_distance_;
    double minimum_travel_heading_;
};

// You can retrieve the list below with : 
// cat open_karto/include/open_karto/Karto.h | grep '^ \+class.*' | cut -d: -f1 | cut -d';' -f1 | sed 's/KARTO_EXPORT //g' | sed 's/^ \+class //g' | sed 's/^/BOOST_CLASS_EXPORT(karto::/g' | sed 's/ *$/)/g' | sort | uniq
BOOST_CLASS_EXPORT(karto::AbstractParameter)
BOOST_CLASS_EXPORT(karto::BoundingBox2)
BOOST_CLASS_EXPORT(karto::CustomData)
BOOST_CLASS_EXPORT(karto::Dataset)
BOOST_CLASS_EXPORT(karto::DatasetInfo)
BOOST_CLASS_EXPORT(karto::LaserRangeFinder)
BOOST_CLASS_EXPORT(karto::LaserRangeScan)
BOOST_CLASS_EXPORT(karto::LocalizedRangeScan)
BOOST_CLASS_EXPORT(karto::Name)
BOOST_CLASS_EXPORT(karto::Object)
BOOST_CLASS_EXPORT(karto::Parameter<karto::Pose2>)
BOOST_CLASS_EXPORT(karto::Parameter<kt_double>)
BOOST_CLASS_EXPORT(karto::Parameter<std::string>)
BOOST_CLASS_EXPORT(karto::Parameter<kt_int32u>)
BOOST_CLASS_EXPORT(karto::Parameter<kt_int32s>)
BOOST_CLASS_EXPORT(karto::Parameter<kt_bool>)
BOOST_CLASS_EXPORT(karto::ParameterEnum)
BOOST_CLASS_EXPORT(karto::ParameterManager)
BOOST_CLASS_EXPORT(karto::Pose2)
BOOST_CLASS_EXPORT(karto::Sensor)
BOOST_CLASS_EXPORT(karto::SensorData)
BOOST_CLASS_EXPORT(karto::Vector2<kt_double>)
BOOST_CLASS_EXPORT(karto::Vector2<kt_int32s>)

BOOST_CLASS_EXPORT(karto::ObjectVector)
typedef std::map<karto::Name, karto::Sensor*> KartoNameSensorMap;
BOOST_CLASS_EXPORT(KartoNameSensorMap)
BOOST_CLASS_EXPORT(karto::ParameterVector)
typedef std::map<std::string, karto::AbstractParameter*> KartoStringAbstractParameterMap;
BOOST_CLASS_EXPORT(KartoStringAbstractParameterMap)
typedef std::map<std::string, kt_int32s> EnumMap;
BOOST_CLASS_EXPORT(EnumMap)
BOOST_CLASS_EXPORT(karto::CustomDataVector)
BOOST_CLASS_EXPORT(karto::PointVectorDouble)

SlamKarto::SlamKarto() :
        got_map_(false),
        laser_count_(0),
        transform_thread_(NULL),
        vis_thread_(NULL),
        solver_loader_("slam_karto", "karto::ScanSolver"),
        visualizer_loader_("slam_karto", "karto::GraphVisualizer"),
        mapper_(NULL),
        dataset_(NULL),
	old_map_to_base_link_set_(false),
	old_map_to_old_odom_set_(false),
	dont_add_new_nodes_before_graph_is_restored_(false)
{
  map_to_odom_.setIdentity();
  // Retrieve parameters
  private_nh_ = ros::NodeHandle("~");
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
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

  if (!private_nh_.getParam("solver_type", solver_type_))
    solver_type_ = "SPASolver";

  if (!private_nh_.getParam("visualizer_type", visualizer_type_))
    visualizer_type_ = "SPAGraphVisualizer";

  private_nh_.getParam("input_dataset_file", input_dataset_file_);
  private_nh_.getParam("output_dataset_file", output_dataset_file_);
  private_nh_.getParam("dont_add_new_nodes_before_graph_is_restored", dont_add_new_nodes_before_graph_is_restored_);

  // Set up advertisements and subscriptions
  tfB_ = new tf::TransformBroadcaster();
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamKarto::mapCallback, this);
  rebuild_graph_ss_ = node_.advertiseService("rebuild_graph", &SlamKarto::rebuildGraphCallback, this);
  save_dataset_ss_ = node_.advertiseService("save_dataset", &SlamKarto::saveDatasetCallback, this);
  set_old_map_to_base_link_sub_ = node_.subscribe("/set_old_map_to_base_link", 1, &SlamKarto::setOldMapToBaseLinkCallback, this);
  set_old_map_to_old_odom_sub_ = node_.subscribe("/set_old_map_to_old_odom", 1, &SlamKarto::setOldMapToOldOdomCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamKarto::laserCallback, this, _1));
  marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",1);

  // Create a thread to periodically publish the latest map->odom
  // transform; it needs to go out regularly, uninterrupted by potentially
  // long periods of computation in our main loop.
  transform_thread_ = new boost::thread(boost::bind(&SlamKarto::publishLoop, this, transform_publish_period));

  // Initialize Karto structures
  mapper_ = new karto::Mapper();
  dataset_ = new karto::Dataset();

  // Set solver to be used in loop closure
  std::stringstream solver_plugin_stream;
  solver_plugin_stream << "karto_plugins::" << solver_type_;
  try
  {
    solver_ = solver_loader_.createInstance(solver_plugin_stream.str());
    ROS_INFO_STREAM("Loaded " << solver_plugin_stream.str() << " solver plugin");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM(
        "The solver plugin " << solver_plugin_stream.str() << " failed to load for some reason. Error: " << ex.what()
            << ". We will load the default solver plugin (SPA).");
    solver_ = boost::shared_ptr<karto_plugins::SPASolver>(new karto_plugins::SPASolver());
  }

  // Initialize karto mapper
  initializeMapper(private_nh_, solver_, mapper_);


  // Set visualizer for the graph
  std::stringstream visualizer_plugin_stream;
  visualizer_plugin_stream << "karto_plugins::" << visualizer_type_;
  try
  {
    visualizer_ = visualizer_loader_.createInstance(visualizer_plugin_stream.str());
    ROS_INFO_STREAM("Loaded " << visualizer_plugin_stream.str() << " visualizer plugin");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM(
        "The visualizer plugin " << visualizer_plugin_stream.str() << " failed to load for some reason. Error: " << ex.what()
            << ". We will load the default visualizer plugin (SPA).");
    visualizer_ = boost::shared_ptr<karto_plugins::SPAGraphVisualizer>(new karto_plugins::SPAGraphVisualizer());
  }
  visualizer_->initialize(solver_);
  visualizer_->setFrameId(map_frame_);

  vis_thread_ = new boost::thread(boost::bind(&SlamKarto::visLoop, this, vis_publish_period));
}

SlamKarto::~SlamKarto()
{
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
  if (mapper_)
    delete mapper_;
  if (dataset_)
    delete dataset_;
  visualizer_.reset();
  solver_.reset();
  // TODO: delete the pointers in the lasers_ map; not sure whether or not
  // I'm supposed to do that.
}

void SlamKarto::initializeMapper(ros::NodeHandle& private_nh,boost::shared_ptr<karto::ScanSolver> solver,karto::Mapper* mapper){

  // Setting General Parameters from the Parameter Server
  bool use_scan_matching;
  if(private_nh.getParam("use_scan_matching", use_scan_matching))
    mapper->setParamUseScanMatching(use_scan_matching);
  
  bool use_scan_barycenter;
  if(private_nh.getParam("use_scan_barycenter", use_scan_barycenter))
    mapper->setParamUseScanBarycenter(use_scan_barycenter);

  if(private_nh.getParam("minimum_travel_distance", minimum_travel_distance_))
    mapper->setParamMinimumTravelDistance(minimum_travel_distance_);

  if(private_nh.getParam("minimum_travel_heading", minimum_travel_heading_))
    mapper->setParamMinimumTravelHeading(minimum_travel_heading_);

  int scan_buffer_size;
  if(private_nh.getParam("scan_buffer_size", scan_buffer_size))
    mapper->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance;
  if(private_nh.getParam("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance))
    mapper->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  double link_match_minimum_response_fine;
  if(private_nh.getParam("link_match_minimum_response_fine", link_match_minimum_response_fine))
    mapper->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

  double link_scan_maximum_distance;
  if(private_nh.getParam("link_scan_maximum_distance", link_scan_maximum_distance))
    mapper->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  double loop_search_maximum_distance;
  if(private_nh.getParam("loop_search_maximum_distance", loop_search_maximum_distance))
    mapper->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  bool do_loop_closing;
  if(private_nh.getParam("do_loop_closing", do_loop_closing))
    mapper->setParamDoLoopClosing(do_loop_closing);

  int loop_match_minimum_chain_size;
  if(private_nh.getParam("loop_match_minimum_chain_size", loop_match_minimum_chain_size))
    mapper->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  double loop_match_maximum_variance_coarse;
  if(private_nh.getParam("loop_match_maximum_variance_coarse", loop_match_maximum_variance_coarse))
    mapper->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse;
  if(private_nh.getParam("loop_match_minimum_response_coarse", loop_match_minimum_response_coarse))
    mapper->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine;
  if(private_nh.getParam("loop_match_minimum_response_fine", loop_match_minimum_response_fine))
    mapper->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

  // Setting Correlation Parameters from the Parameter Server

  double correlation_search_space_dimension;
  if(private_nh.getParam("correlation_search_space_dimension", correlation_search_space_dimension))
    mapper->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  double correlation_search_space_resolution;
  if(private_nh.getParam("correlation_search_space_resolution", correlation_search_space_resolution))
    mapper->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  double correlation_search_space_smear_deviation;
  if(private_nh.getParam("correlation_search_space_smear_deviation", correlation_search_space_smear_deviation))
    mapper->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter Server

  double loop_search_space_dimension;
  if(private_nh.getParam("loop_search_space_dimension", loop_search_space_dimension))
    mapper->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution;
  if(private_nh.getParam("loop_search_space_resolution", loop_search_space_resolution))
    mapper->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation;
  if(private_nh.getParam("loop_search_space_smear_deviation", loop_search_space_smear_deviation))
    mapper->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters from the Parameter Server

  double distance_variance_penalty;
  if(private_nh.getParam("distance_variance_penalty", distance_variance_penalty))
    mapper->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty;
  if(private_nh.getParam("angle_variance_penalty", angle_variance_penalty))
    mapper->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset;
  if(private_nh.getParam("fine_search_angle_offset", fine_search_angle_offset))
    mapper->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset;
  if(private_nh.getParam("coarse_search_angle_offset", coarse_search_angle_offset))
    mapper->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution;
  if(private_nh.getParam("coarse_angle_resolution", coarse_angle_resolution))
    mapper->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty;
  if(private_nh.getParam("minimum_angle_penalty", minimum_angle_penalty))
    mapper->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty;
  if(private_nh.getParam("minimum_distance_penalty", minimum_distance_penalty))
    mapper->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion;
  if(private_nh.getParam("use_response_expansion", use_response_expansion))
    mapper->setParamUseResponseExpansion(use_response_expansion);

  mapper->SetScanSolver(solver.get());
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

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
	     scan->header.frame_id.c_str(),
	     laser_pose.getOrigin().x(),
	     laser_pose.getOrigin().y(),
	     yaw);
    // To account for lasers that are mounted upside-down,
    // we create a point 1m above the laser and transform it into the laser frame
    // if the point's z-value is <=0, it is upside-down

    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan->header.stamp, base_frame_);

    try
    {
      tf_.transformPoint(scan->header.frame_id, up, up);
      ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN("Unable to determine orientation of laser: %s", e.what());
      return NULL;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
    if (inverse)
      ROS_INFO("laser is mounted upside-down");


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
    tf_.transformPose(odom_frame_, ident, odom_pose); // odom -> base_link transform
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  karto_pose = 
          karto::Pose2(odom_pose.getOrigin().x(),
                       odom_pose.getOrigin().y(),
                       yaw);
  return true;
}

void
SlamKarto::publishGraphVisualization()
{
  visualization_msgs::MarkerArray marray = visualizer_->createVisualizationMarkers();
  marker_publisher_.publish(marray);
}

void
SlamKarto::setOldMapToBaseLinkCallback(const geometry_msgs::Pose::ConstPtr& pose){
  ROS_INFO_STREAM("Received Old map -> /base_link transform");
	old_map_to_base_link_.SetX(pose->position.x);
	old_map_to_base_link_.SetY(pose->position.y);
  	double yaw = tf::getYaw(pose->orientation);
	old_map_to_base_link_.SetHeading(yaw);
	old_map_to_base_link_set_=true;
}

void
SlamKarto::setOldMapToOldOdomCallback(const geometry_msgs::Pose::ConstPtr& pose){
  ROS_INFO_STREAM("Received Old map -> Old odom transform");
	old_map_to_old_odom_.SetX(pose->position.x);
	old_map_to_old_odom_.SetY(pose->position.y);
  	double yaw = tf::getYaw(pose->orientation);
	old_map_to_old_odom_.SetHeading(yaw);
	old_map_to_old_odom_set_=true;
}

void
SlamKarto::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if(dont_add_new_nodes_before_graph_is_restored_ && dataset_->GetObjects().size() == 0){
    return;
  }

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
    ROS_DEBUG("added scan at pose: %.3f %.3f %.3f", 
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
  karto::LocalizedRangeScan* range_scan = 
    new karto::LocalizedRangeScan(laser->GetName(), readings);
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  // Add the localized range scan to the mapper
  bool processed;
  if((processed = mapper_->Process(range_scan)))
  {
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();

    // Compute the map->odom transform
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      tf_.transformPose(odom_frame_,tf::Stamped<tf::Pose> (tf::Transform(tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()),
                                                                    tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0)).inverse(),
                                                                    scan->header.stamp, base_frame_),odom_to_map);
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

tf::Transform SlamKarto::kartoPose2ToTfTransform(const karto::Pose2& karto_pose){
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(karto_pose.GetPosition().GetX(),karto_pose.GetPosition().GetY(),0.0));
  transform.setRotation(tf::createQuaternionFromYaw(karto_pose.GetHeading()));
  return transform;
}

karto::Pose2 SlamKarto::tfTransformToKartoPose2(const tf::Transform& transform){
  return karto::Pose2(transform.getOrigin()[0],transform.getOrigin()[1],tf::getYaw(transform.getRotation()));
}

bool
SlamKarto::saveDatasetCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  std::ofstream ofs(output_dataset_file_.c_str());
  {
    boost::archive::text_oarchive oa(ofs);
    oa << dataset_;
  }

  map_to_odom_mutex_.lock();
  ROS_INFO_STREAM("map_to_odom_.getRotation(): " << map_to_odom_.getRotation() << " map_to_odom_.getOrigin(): " << map_to_odom_.getOrigin());
  map_to_odom_mutex_.unlock();
  return true;
}

bool SlamKarto::offsetOdometricPosesForNewMapFrame(){

  if(!old_map_to_base_link_set_ || !old_map_to_old_odom_set_){
	ROS_ERROR("Some initial transforms have not been specified. Cannot offset odometric poses of nodes for new map frame");
	return false;
  }

  // Get the robot's odometric pose ( /odom -> /base_link )
  ros::Time now = ros::Time::now();
  tf::Stamped<tf::Transform> current_base_wrt_odom;
  try{
    tf_.waitForTransform(odom_frame_, base_frame_,
                              now, ros::Duration(1.0));
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                           tf::Vector3(0,0,0)), now, base_frame_);
    tf_.transformPose(odom_frame_, ident, current_base_wrt_odom);
  }
  catch(tf::TransformException e)
  {
    ROS_ERROR("Failed to compute odom pose, cannot offset odometric poses for new map frame! (%s)", e.what());
    return false;
  }
  
  // Now we need to convert the odometric poses of the old /odom frame (/odom_old -> /node)
  // into the new /odom frame (/odom -> /node). For this we go through 3 steps : 
  // 1) compose with current /odom -> /base_link
  // 2) compose with inverse of /map_old -> /base_link
  // 3) compose with /map_old -> /odom_old
  // That gives us :( /odom -> /base_link + /base_link -> /map_old + /map_old -> /odom_old ) + /odom_old -> /node
  // == /odom -> /odom_old + /odom_old -> /node == /odom -> /node
  for(int i = 0; i < dataset_->GetObjects().size();i++){
    if (dynamic_cast<karto::LocalizedRangeScan*>(dataset_->GetObjects()[i])){
      karto::LocalizedRangeScan* pScan = dynamic_cast<karto::LocalizedRangeScan*>(dataset_->GetObjects()[i]);
      karto::Pose2 old_odometric_pose = pScan->GetOdometricPose();
      karto::Pose2 old_corrected_pose = pScan->GetCorrectedPose();
      karto::Pose2 new_odometric_pose = old_odometric_pose;
      ROS_INFO_STREAM("Odometric pose as read from file: " << old_odometric_pose);
      
      // We convert the karto::Pose2 into tf::Transform, which are more intuitive to use for composing transformations
      tf::Transform tf_new_odometric_pose = kartoPose2ToTfTransform(new_odometric_pose);
      tf::Transform tf_old_map_to_base_link = kartoPose2ToTfTransform(old_map_to_base_link_);
      tf::Transform tf_old_map_to_old_odom = kartoPose2ToTfTransform(old_map_to_old_odom_);

      // Step 1)
      tf_new_odometric_pose = tf_new_odometric_pose * current_base_wrt_odom;
      new_odometric_pose = tfTransformToKartoPose2(tf_new_odometric_pose);
      ROS_INFO_STREAM("Odometric pose after composing with current_base_wrt_odom: " << new_odometric_pose);

      // Step 2)
      tf_new_odometric_pose= tf_old_map_to_base_link.inverse() * tf_new_odometric_pose;
      new_odometric_pose = tfTransformToKartoPose2(tf_new_odometric_pose);
      ROS_INFO_STREAM("Odometric pose after composing with INV of old_map_to_base_link: " << new_odometric_pose);

      // Step 3)
      tf_new_odometric_pose =  tf_new_odometric_pose * tf_old_map_to_old_odom;
      new_odometric_pose = tfTransformToKartoPose2(tf_new_odometric_pose);
      ROS_INFO_STREAM("Odometric pose after composing with old_map_to_old_odom: " << new_odometric_pose);

      // Convert back to karto::Pose2
      new_odometric_pose = tfTransformToKartoPose2(tf_new_odometric_pose);
      ROS_INFO_STREAM("Odometric pose after applying offset: " << new_odometric_pose);
      pScan->SetOdometricPose(new_odometric_pose);
      
      // Now we apply the same offset to the corrected pose
      karto::Transform lastTransform(old_odometric_pose, new_odometric_pose);
      karto::Pose2 new_corrected_pose = lastTransform.TransformPose(old_corrected_pose);
      pScan->SetCorrectedPose(new_corrected_pose);

      // And set the odometric pose to be the corrected pose
      pScan->SetOdometricPose(new_corrected_pose);
      
     // pScan->SetCorrectedPose(new_odometric_pose); // Cancel corrections
    }
  }

  return true;
}

std::string SlamKarto::getDatasetSummary(){
  std::stringstream ss;
  int scans=0;
  int sensors=0;
  int others=0;
  for(int i = 0; i < dataset_->GetObjects().size();i++){
    if (dynamic_cast<karto::LocalizedRangeScan*>(dataset_->GetObjects()[i])){
      scans++;
    }else if(dynamic_cast<karto::LaserRangeFinder*>(dataset_->GetObjects()[i])){
      sensors++;
    }else{
      others++;
    }
  }
  ss << "Dataset contains " << scans <<" karto::LocalizedRangeScan, "<< sensors<< " karto::LaserRangeFinder, and " << others <<" unknown objects.";
  return ss.str();
}

bool
SlamKarto::rebuildGraphCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){

  // Print summary of dataset before the operation
  ROS_DEBUG_STREAM(getDatasetSummary());

  // Clear all the objects containing the graph state
  mapper_->Reset();
  solver_->Clear();
  dataset_->Clear();
  lasers_.clear();

  // Load the dataset from the input file
  {
    std::ifstream ifs(input_dataset_file_.c_str());
    boost::archive::text_iarchive ia(ifs);
    ia >> dataset_;
  }

  // Transform the odometric poses of the nodes from the old odom frame
  // to the new one (when karto is launched, /odom = /map)
  offsetOdometricPosesForNewMapFrame();

  ROS_DEBUG_STREAM(getDatasetSummary());

  // Reinitialize the mapper
  mapper_ = new karto::Mapper();
  initializeMapper(private_nh_,solver_,mapper_);

  // We disable the minimum travel distance/heading
  // as we reload the nodes of the graph, we re-enable them after
  mapper_->setParamMinimumTravelDistance(0.0);
  mapper_->setParamMinimumTravelHeading(0.0);

  // Re-populate the list of sensors
  int sensor_count = 0;
  try{
    for(int i = 0; i < dataset_->GetObjects().size();i++){
      if(dynamic_cast<karto::LaserRangeFinder*>(dataset_->GetObjects()[i])){
        karto::LaserRangeFinder* pLaser = dynamic_cast<karto::LaserRangeFinder*>(dataset_->GetObjects()[i]);
        lasers_.insert(std::map<std::string, karto::LaserRangeFinder*>::value_type(pLaser->GetName().GetName(), pLaser));
        sensor_count+=1;
      }
    }
  }catch(karto::Exception& e){
    ROS_ERROR_STREAM("Failed to re-populate the list of sensors while rebuilding the graph!: " << e);
    return false;
  }
  if(sensor_count == 0){
    ROS_ERROR_STREAM("No sensors were found in the input dataset file! Cannot restore graph");
    return false;
  }

  // Then make the mapper re-process the scans one by one
  try{
    for(int i = 0; i < dataset_->GetObjects().size();i++){
      if (dynamic_cast<karto::LocalizedRangeScan*>(dataset_->GetObjects()[i])){
        karto::LocalizedRangeScan* pScan = dynamic_cast<karto::LocalizedRangeScan*>(dataset_->GetObjects()[i]);
//        pScan->SetCorrectedPose(pScan->GetOdometricPose()); // Reset corrections
        if(!mapper_->Process(pScan)){
          ROS_ERROR("Failed to re-add scan to mapper!");
        }
      }
    }
  }catch(karto::Exception& e){
    ROS_ERROR_STREAM("Failed to re-process scans from the dataset file through the mapper. Cannot rebuild graph: " << e);
    return false;
  }

  // Now that we have re-processed the scans, we can re-enable
  // the minimum travel distance/heading
  mapper_->setParamMinimumTravelDistance(minimum_travel_distance_);
  mapper_->setParamMinimumTravelHeading(minimum_travel_heading_);

  // Update the occupancy grid
  if(updateMap())
  {
    got_map_ = true;
    ROS_DEBUG("Updated the map");
  }
  return true;
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
  if (vis_publish_period == 0)
    return;

  ros::Rate r(1.0 / vis_publish_period);
  while (ros::ok()) {
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
