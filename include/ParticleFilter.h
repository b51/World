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

#include "WorldUtils.h"

namespace IKid
{
namespace World
{
using Rigid2d = cartographer::transform::Rigid2d;

struct ParticleFilterOptions
{
  int particles_num;
  double gaussian_noise_mean;
  double gaussian_noise_var;
};

inline ParticleFilterOptions CreateParticleFilterOptions(
    cartographer::common::LuaParameterDictionary* const lua_parameter_dictionary)
{
  ParticleFilterOptions options;
  options.particles_num = lua_parameter_dictionary->GetInt("particles_num");
  options.gaussian_noise_mean = lua_parameter_dictionary->GetDouble("gaussian_noise_mean");
  options.gaussian_noise_var = lua_parameter_dictionary->GetDouble("gaussian_noise_var");
  return options;
}

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
  void InitWithKnownPose(const Rigid2d& init_pose);
  void UpdateAction(const Rigid2d& action);
  void UpdateObservation();
  void UpdateResample();
  void AddParticleToCluster(std::vector<SampleConstPtr>& samples, const int* key);

private:
  void ParticleCluster();

public:
  Rigid2d current_pose_;

private:
  ParticleFilterOptions options_;

  Samples samples_;
  std::vector<std::vector<SampleConstPtr> > clusters_;

  // weight_slow, weight_fast, (Prob Robot p258)
  double weight_slow_;
  double weight_fast_;
};

}
} // namespace IKid

#endif
