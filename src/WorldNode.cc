/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: WorldNode.cc
*
*          Created On: Mon 03 Sep 2018 03:33:11 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "WorldNode.h"

namespace IKid
{
namespace World
{

WorldNode::WorldNode(const WorldOptions& options)
    : options_(options)
{
  LOG(INFO) << "log_level                   : " << options.log_level;
}

WorldNode::~WorldNode()
{
}

void WorldNode::Init()
{
  /* Open shms */
  int ret;
  ret = vision_shm_open();
  if (ret < 0)
    LOG(ERROR) << "Vision shm open falied.";
  else
    LOG(INFO) << "Vision shm opend.";

  ret = world_shm_open();
  if (ret < 0)
    LOG(ERROR) << "World shm open falied.";
  else
    LOG(INFO) << "World shm opend.";

  Modeling modeling(options_.modeling_options);
  modeling.Init();
}

} // namespace World
} // namespace IKid
