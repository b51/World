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

#include "WorldOptions.h"
#include "LandmarkData.h"
#include "Modeling.h"

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

public:
  ::ros::NodeHandle node_handle_;

private:
  WorldOptions options_;
};

} // namespace World
} // namespace IKid

#endif
