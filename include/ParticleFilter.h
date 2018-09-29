/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: ParticleFilter.h
*
*          Created On: Tue 04 Sep 2018 12:05:05 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef IKID_WORLD_PARTICLE_FILTER_H_
#define IKID_WORLD_PARTICLE_FILTER_H_

#include <vector>

#include "ceres/ceres.h"

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/transform/rigid_transform.h"

#include "WorldOptions.h"
#include "WorldUtils.h"
#include "shm_util.h"

namespace IKid
{
namespace World
{
using Rigid2d = cartographer::transform::Rigid2d;

/**
 *  This function used for minimize resudual of landmark observation pose and true pose.
 *
 *  @param[in] scaling_factor each landmark has its own weight, eg. goal post weights higher.
 *  @param[in] observe_pose landmark pose in robot frame.
 *  @param[in] landmark_pose landmark true pose in world frame.
 */
class LandmarkObservationCostFunction
{
public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
        const double scaling_factor
      , const Eigen::Vector2d& observe_pose
      , const Eigen::Vector2d& landmark_pose)
  {
    return new ceres::AutoDiffCostFunction<LandmarkObservationCostFunction
                                         , 2 /* residuals */
                                         , 3 /* pose variables */>(
                       new LandmarkObservationCostFunction(scaling_factor
                                                         , observe_pose
                                                         , landmark_pose));
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const
  {
    Eigen::Matrix<T, 2, 1> robot_point(pose[0], pose[1]);
    Eigen::Rotation2D<T> robot_orientation(pose[2]);
    Eigen::Matrix<T, 2, 1> observe_in_world = robot_orientation * observe_pose_ + robot_point;

    residual[0] = scaling_factor_ * (observe_in_world[0] - landmark_pose_[0]);
    residual[1] = scaling_factor_ * (observe_in_world[1] - landmark_pose_[1]);
    return true;
  }

private:
  explicit LandmarkObservationCostFunction(
        const double scaling_factor
      , const Eigen::Vector2d& observe_pose
      , const Eigen::Vector2d& landmark_pose)
      : scaling_factor_(scaling_factor)
      , observe_pose_(observe_pose)
      , landmark_pose_(landmark_pose)
  {}
  LandmarkObservationCostFunction(const LandmarkObservationCostFunction&) = delete;
  LandmarkObservationCostFunction& operator=(
      const LandmarkObservationCostFunction&) = delete;

  const double scaling_factor_;
  const Eigen::Vector2d observe_pose_;
  const Eigen::Vector2d landmark_pose_;
};

struct Sample
{
  Sample() : weight(0.), clustered(false) {}
  Sample(const Rigid2d& p, const double w)
      : pose(p), weight(w), clustered(false) {}

  Rigid2d pose;
  double weight;
  bool clustered;
};

typedef std::shared_ptr<Sample> SamplePtr;
typedef std::shared_ptr<const Sample> SampleConstPtr;
typedef std::vector<SamplePtr> Samples;

class ParticleFilter
{
public:
  ParticleFilter(const ParticleFilterOptions& options);
  ~ParticleFilter();

  void Init();

  /**
   *  Will generate samples with gaussian noise
   *
   *  @param[in] init_pose
   */
  void InitWithKnownPose(const Rigid2d& init_pose);

  /**
   *  Update samples pose with movement
   *
   *  @param[in] action movement of robot
   */
  void UpdateAction(const Rigid2d& action);

  /**
   *  Update samples weight with observations
   *
   *  TODO @param[in] observations from vision node, with pose
   */
  void UpdateObservation();

  /**
   *  Resample samples when weights change, come from Probabilistic Robotics(pg.258)
   */
  void UpdateResample();

  Rigid2d GetCurrentPose() { return current_pose_; }

private:
  /**
   *  Cluster samples to grids and get max weight cluster
   */
  void ParticleCluster();

  /**
   *  Loop cluster samples
   */
  void AddParticleToCluster(std::vector<SampleConstPtr>& samples, const int* key);

  /**
   *  Set current pose to shm
   */
  void UpdateShm();

public:

private:
  ParticleFilterOptions options_;

  Samples samples_;
  std::vector<std::vector<SampleConstPtr> > clusters_;

  Rigid2d current_pose_;
  double weight_slow_;
  double weight_fast_;
};

}
} // namespace IKid

#endif
