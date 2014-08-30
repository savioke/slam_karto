/* Extends ScanSolver and forces everything to have a graph publisher */
#ifndef SLAM_SOLVER_H
#define SLAM_SOLVER_H

#include <open_karto/Mapper.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>

namespace karto {

class SLAMSolver : public karto::ScanSolver
{
  public:
    virtual void publishGraphVisualization(visualization_msgs::MarkerArray &marray)=0;
};

};

#endif
