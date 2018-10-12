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

  /* Particle filter init */
  particle_filter_ = cartographer::common::make_unique<ParticleFilter>(
      options_.particle_filter_options);

  Rigid2d init_pose({0., 0.}, 0.);

  particle_filter_->InitWithKnownPose(init_pose);

  Rigid2d movement({0.5, 0.5}, 0.5);
  particle_filter_->UpdateAction(movement);
  particle_filter_->UpdateObservation();

  //TODO need remove
  double* pr = world_shm_get_ptr(pose_key);
  for (int i = 0; i < world_keys[pose_key].size; i++)
    LOG(INFO) << "get position " << i << " : " << *(pr+i);
}

void WorldNode::Run()
{
  while (node_handle_.ok())
  {
    double current_time = GetTime();
    bool ball_detected = false;
    // First, get all landmark information to update particles

    Rigid2d last_pose = particle_filter_->GetCurrentPose();
    for (const auto& vision_key : vision_keys)
    {
      double* vcm_pr = vision_shm_get_ptr(vision_key.first);
      if (*vcm_pr)
      {
        if (vision_key.first == ball_detection_key)
        {
          ball_detected = true;
          continue;
        }
        Eigen::Vector3d position(*(vcm_pr+1), *(vcm_pr+2), *(vcm_pr+3));

        if (vision_key.first == goal_detection_key)
        {
          // Use last_pose to calculate this post position in global frame
          // to get rid of wrong detection.
        }
      }
    }

    // Second, get current pose to calculate ball position in global frame
    Rigid2d cur_pose = particle_filter_->GetCurrentPose();
    if (ball_detected)
    {
      double *wcm_pr = world_shm_set_ptr(ball_global_key, world_keys[ball_global_key].size);
      *wcm_pr = current_time;

      ball_ = cur_pose * position.head<2>();
      *(wcm_pr+1) = ball_.x();
      *(wcm_pr+2) = ball_.y();
      *(wcm_pr+3) = position.z();
    }
  }
}

} // namespace World
} // namespace IKid
