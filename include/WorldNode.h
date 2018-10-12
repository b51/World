/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: WorldNode.h
*
*          Created On: Mon 03 Sep 2018 03:36:29 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef IKID_WORLD_NODE_H_
#define IKID_WORLD_NODE_H_

#include <stdio.h>
#include <iostream>
#include <chrono>

#include <ros/ros.h>

#include "Utils.h"

#include "WorldOptions.h"
#include "LandmarkData.h"
#include "ParticleFilter.h"

namespace IKid
{
namespace World
{

class WorldNode
{
public:
  WorldNode(const WorldOptions& options);
  ~WorldNode();
  void Init();
  void Run();

public:
  ::ros::NodeHandle node_handle_;

private:
  WorldOptions options_;

  std::unique_ptr<ParticleFilter> particle_filter_;

  Rigid2d current_pose_;
  Eigen::Vector2d ball_;
  Eigen::Vector2d goal_;
  Eigen::Vector2d spot_;
};

} // namespace World
} // namespace IKid

#endif
