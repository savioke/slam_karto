/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <slam_karto/spa_solver.h>
#include <open_karto/Karto.h>

#include "ros/console.h"

#include<pluginlib/class_list_macros.h>


namespace karto_plugins {

SPASolver::SPASolver() : marker_count_(0)
{

}

SPASolver::~SPASolver()
{

}

void SPASolver::Clear()
{
  corrections.clear();
}

const karto::ScanSolver::IdPoseVector& SPASolver::GetCorrections() const
{
  return corrections;
}

void SPASolver::Compute()
{
  corrections.clear();

  typedef std::vector<sba::Node2d, Eigen::aligned_allocator<sba::Node2d> > NodeVector;

  ROS_INFO("Calling doSPA for loop closure");
  m_Spa.doSPA(40);
  ROS_INFO("Finished doSPA for loop closure");
  NodeVector nodes = m_Spa.getNodes();
  forEach(NodeVector, &nodes)
  {
    karto::Pose2 pose(iter->trans(0), iter->trans(1), iter->arot);
    corrections.push_back(std::make_pair(iter->nodeId, pose));
  }
}

void SPASolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
{
  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  Eigen::Vector3d vector(pose.GetX(), pose.GetY(), pose.GetHeading());
  m_Spa.addNode(vector, pVertex->GetObject()->GetUniqueId());
}

void SPASolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
{
  karto::LocalizedRangeScan* pSource = pEdge->GetSource()->GetObject();
  karto::LocalizedRangeScan* pTarget = pEdge->GetTarget()->GetObject();
  karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());

  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  Eigen::Vector3d mean(diff.GetX(), diff.GetY(), diff.GetHeading());

  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix<double,3,3> m;
  m(0,0) = precisionMatrix(0,0);
  m(0,1) = m(1,0) = precisionMatrix(0,1);
  m(0,2) = m(2,0) = precisionMatrix(0,2);
  m(1,1) = precisionMatrix(1,1);
  m(1,2) = m(2,1) = precisionMatrix(1,2);
  m(2,2) = precisionMatrix(2,2);

  m_Spa.addConstraint(pSource->GetUniqueId(), pTarget->GetUniqueId(), mean, m);
}

void SPASolver::publishGraphVisualization(visualization_msgs::MarkerArray &marray)
{
  std::vector<float> graph;
  getGraph(graph);

  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "karto";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0);

  visualization_msgs::Marker edge;
  edge.header.frame_id = "map";
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;

  m.action = visualization_msgs::Marker::ADD;
  uint id = 0;
  for (uint i=0; i<graph.size()/2; i++) 
  {
    m.id = id;
    m.pose.position.x = graph[2*i];
    m.pose.position.y = graph[2*i+1];
    marray.markers.push_back(visualization_msgs::Marker(m));
    id++;

    if(i>0)
    {
      edge.points.clear();

      geometry_msgs::Point p;
      p.x = graph[2*(i-1)];
      p.y = graph[2*(i-1)+1];
      edge.points.push_back(p);
      p.x = graph[2*i];
      p.y = graph[2*i+1];
      edge.points.push_back(p);
      edge.id = id;

      marray.markers.push_back(visualization_msgs::Marker(edge));
      id++;
    }
  }

  m.action = visualization_msgs::Marker::DELETE;
  for (; id < marker_count_; id++) 
  {
    m.id = id;
    marray.markers.push_back(visualization_msgs::Marker(m));
  }

  marker_count_ = marray.markers.size();
}

} //namespace karto_plugins

PLUGINLIB_EXPORT_CLASS(karto_plugins::SPASolver, karto::SLAMSolver)
