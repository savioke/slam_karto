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

#include <slam_karto/spa_solver.h>
#include <slam_karto/spa_graph_visualizer.h>

#include <boost/thread.hpp>

#include <string>
#include <map>
#include <vector>
#include <locale> // for std::isupper, std::tolower

#include <pluginlib/class_loader.h>

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

    // ROS handles
    ros::NodeHandle node_;
    tf::TransformListener tf_;
    tf::TransformBroadcaster* tfB_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    ros::Publisher sst_;
    ros::Publisher marker_publisher_;
    ros::Publisher sstm_;
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
};

SlamKarto::SlamKarto() :
        got_map_(false),
        laser_count_(0),
        transform_thread_(NULL),
        vis_thread_(NULL),
        solver_loader_("slam_karto", "karto::ScanSolver"),
        visualizer_loader_("slam_karto", "karto::GraphVisualizer"),
        mapper_(NULL)
{
  map_to_odom_.setIdentity();
  // Retrieve parameters
  ros::NodeHandle private_nh_("~");
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

  // Set up advertisements and subscriptions
  tfB_ = new tf::TransformBroadcaster();
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamKarto::mapCallback, this);
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

  // Read karto parameters from ROS if they are present:
  // For each karto parameter name, if that parameter is set in ROS,
  // read the parameter and set it in karto.
  const karto::ParameterVector& mapper_parameters = mapper_->GetParameters();
  for (karto::ParameterVector::const_iterator iter = mapper_parameters.begin();
         iter != mapper_parameters.end(); ++iter) {
    std::string karto_parameter_name = (*iter)->GetName();
    // Karto parameter names are in CamelCase, ROS parameters are in under_scores,
    // so we have to convert.
    std::string ros_parameter_name;
    for (std::string::iterator str_it = karto_parameter_name.begin(); str_it != karto_parameter_name.end(); ++ str_it) {
      if (str_it != karto_parameter_name.begin() && std::isupper(*str_it)) {
        ros_parameter_name += '_';
      }
      ros_parameter_name += std::tolower(*str_it);
    }
    XmlRpc::XmlRpcValue ros_parameter_value;
    if (private_nh_.getParam(ros_parameter_name, ros_parameter_value)) {
      switch (ros_parameter_value.getType()) {
        case XmlRpc::XmlRpcValue::TypeBoolean:  mapper_->SetParameter(karto_parameter_name, static_cast<bool>(ros_parameter_value)); break;
        case XmlRpc::XmlRpcValue::TypeInt:      mapper_->SetParameter(karto_parameter_name, static_cast<int>(ros_parameter_value)); break;
        case XmlRpc::XmlRpcValue::TypeDouble:   mapper_->SetParameter(karto_parameter_name, static_cast<double>(ros_parameter_value)); break;
        case XmlRpc::XmlRpcValue::TypeString:   mapper_->SetParameter(karto_parameter_name, static_cast<std::string>(ros_parameter_value)); break;
        default: ROS_WARN_STREAM("Cannot set " << ros_parameter_name << ": Unsupported parameter type. Leaving default: "
                     << mapper_->GetParameter(karto_parameter_name)->GetValueAsString()); continue;
      }
      ROS_INFO_STREAM("Set " << ros_parameter_name << " (" << karto_parameter_name << " in karto) to "
          << mapper_->GetParameter(karto_parameter_name)->GetValueAsString());
    } else {
      ROS_INFO_STREAM("No ROS parameter " << ros_parameter_name << " set, using karto default: "
          << mapper_->GetParameter(karto_parameter_name)->GetValueAsString());
    }
  }

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
  mapper_->SetScanSolver(solver_.get());

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
    tf_.transformPose(odom_frame_, ident, odom_pose);
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
    //std::cout << "Pose: " << range_scan->GetOdometricPose() << " Corrected Pose: " << range_scan->GetCorrectedPose() << std::endl;
    
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
