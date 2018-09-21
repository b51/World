/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: Modeling.cc
*
*          Created On: Mon 03 Sep 2018 07:52:34 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "Modeling.h"

namespace IKid
{
namespace World
{
using Rigid2d = cartographer::transform::Rigid2d;

Modeling::Modeling(const ModelingOptions& options)
    : options_(options)
{
}

Modeling::~Modeling()
{
}

void Modeling::Init()
{
  ParticleFilter pf(options_.particle_filter_options);
  Rigid2d init_pose({0., 0.}, 0.);

  pf.InitWithKnownPose(init_pose);

  Rigid2d movement({0.5, 0.5}, 0.5);
  pf.UpdateAction(movement);
  pf.UpdateObservation();

  double* pr = world_shm_get_ptr(pose_key);

  //TODO need remove
  for (int i = 0; i < world_keys[pose_key].size; i++)
    LOG(INFO) << "get position " << i << " : " << *(pr+i);
}

} // namespace World
} // namespace IKid
