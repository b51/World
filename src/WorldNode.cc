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
  world_shm_close();
}

void WorldNode::Init()
{
  /* Init world shm */
  int ret = world_shm_open();
  if (ret < 0)
    LOG(ERROR) << "Could not open world shm";
  else
    LOG(INFO) << "World shm opened";

  for (auto const& world_key : world_keys)
  {
    LOG(INFO) << "Setting up world key: "
        << world_key.first << "[" << world_key.second.size << "]";
    int nval = world_key.second.size;

    double* pr = world_shm_set_ptr(world_key.first, nval);
    for (int i = 0; i < nval; i++)
    {
      *(pr + i) = 0.;   // Init all shm data to 0.
    }
  }

  Modeling modeling(options_.modeling_options);
  modeling.Init();
}

} // namespace World
} // namespace IKid
