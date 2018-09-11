/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: ParticleFilter.cc
*
*          Created On: Tue 04 Sep 2018 11:31:48 AM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "ParticleFilter.h"

namespace IKid
{
namespace World
{
using Rigid2d = cartographer::transform::Rigid2d;

ParticleFilter::ParticleFilter(const ParticleFilterOptions& options)
    : options_(options)
{
  samples_.reserve(options_.particles_num);
}

ParticleFilter::~ParticleFilter()
{
}

void ParticleFilter::Init()
{
}

void ParticleFilter::InitWithKnownPose(const Rigid2d& init_pose)
{
  double noise_mean = options_.gaussian_noise_mean;
  double noise_var = options_.gaussian_noise_var;

  for (int i = 0; i < options_.particles_num; i++)
  {
    Rigid2d noise({RandomInGaussian(noise_mean, noise_var)
                   , RandomInGaussian(noise_mean, noise_var)}
                 , RandomInGaussian(noise_mean, noise_var));

    samples_.emplace_back(std::make_shared<Sample>(noise * init_pose, 1./options_.particles_num));
  }
}

void ParticleFilter::UpdateAction(const Rigid2d& action)
{
  double dist = action.translation().norm();

  for (auto& sample: samples_)
  {
    Rigid2d noise({RandomInMirrorRange(dist * 0.1), RandomInMirrorRange(dist * 0.1)}
                 , RandomInMirrorRange(action.rotation().angle() * 0.1));

    sample->pose = action * sample->pose;
    sample->pose = noise * sample->pose;
    LOG(WARNING) << sample->pose;
  }
}

//void ParticleFilter::UpdateObservation(Observations observations)
void ParticleFilter::UpdateObservation()
{
  //TODO: Parameters for test, need add to config.lua
  double rsigma = 0.5;
  double asigma = 5 * M_PI / 180;

  double translation_weight = 0.7;
  double rotation_weight = 0.3;

  double alpha_slow = 0.001;
  double alpha_fast = 0.1;

  for (auto& sample: samples_)
  {
    // TODO: Add known landmarks and unknown landmarks residual block
    //       should use ceres for all samples or only the best location?
    LOG(WARNING) << sample->pose;
    
    // For known landmarks, i for landmarks num
    // scaling_factor for known and unknown landmarks weight
    double v[3] = {sample->pose.translation().x(), sample->pose.translation().y(),
                   sample->pose.rotation().angle()};

    ceres::Problem problem;
    ceres::Solver::Options options;
    options.num_threads = 1;
    ceres::Solver::Summary summary;

    for (int i = 0; i < 3; i++)
    {
      Eigen::Vector2d observe_pose(4.3, 0);
      Eigen::Vector2d real_pose(4.5, 1.5);

      problem.AddResidualBlock(
          LandmarkObservationCostFunction::CreateAutoDiffCostFunction(
              0.1, observe_pose, real_pose)
          , nullptr, v);
    }

    ceres::Solve(options, &problem, &summary);

    Rigid2d new_pose({v[0], v[1]}, v[2]);

    Rigid2d dist = sample->pose.inverse() * new_pose;

    double error = translation_weight * exp(-dist.translation().squaredNorm() / (rsigma * rsigma))
        + rotation_weight * exp(-(dist.rotation().angle() * dist.rotation().angle()) / (asigma * asigma));

    sample->weight *= error;
  }

  double weight_avg = 0.;
  double weight_sum = 0.;
  for (int i = 0; i < samples_.size(); i++)
    weight_avg += samples_[i]->weight / options_.particles_num;

  weight_sum = weight_avg * options_.particles_num;

  // Normalize weights
  for (auto& sample : samples_)
    sample->weight /= weight_sum;

  if (weight_slow_ == 0.)
    weight_slow_ = weight_avg;
  else
    weight_slow_ += alpha_slow * (weight_avg - weight_slow_);

  if (weight_fast_ == 0.)
    weight_fast_ = weight_avg;
  else
    weight_fast_ = alpha_fast * (weight_avg - weight_fast_);

  UpdateResample();
  ParticleCluster();
}

void ParticleFilter::UpdateResample()
{
  double x_length = 4.5;
  double y_length = 3.0;
  double orientation_range = M_PI;

  double thresthold = std::max(0., 1. - weight_fast_ / weight_slow_);

  if (thresthold == 0.)
    return;

  double weight_avg = 0.;
  double weight_sum = 0.;
  for (auto& sample : samples_)
  {
    if (RandomInRange(0., 1.) < thresthold)
    {
      double x = RandomInMirrorRange(x_length);
      double y = RandomInMirrorRange(y_length);
      double a = RandomInMirrorRange(orientation_range);
      sample->pose = Rigid2d({x, y}, a);
      sample->weight = 1 / options_.particles_num;
    }
    weight_avg += sample->weight / options_.particles_num;
  }
  weight_sum = weight_avg * options_.particles_num;

  for (auto& sample : samples_)
  {
    sample->weight /= weight_sum;
    sample->clustered = false;
  }

  if (thresthold > 0.)
  {
    weight_fast_ = 0.;
    weight_slow_ = 0.;
  }
}

/**
 *  Cluster particles into 0.5 x 0.5 x 10° grid size
 *  with keys.
 */
void ParticleFilter::ParticleCluster()
{
  double grid_size = 0.5;
  double grid_angle = 10 * M_PI / 180.;

  int cluster_id = 0;
  clusters_.clear();

  for (auto& sample : samples_)
  {
    int key[3];
    if (sample->clustered)
      continue;

    sample->clustered = true;
    clusters_.emplace_back(std::vector<SampleConstPtr>{sample});

    key[0] = floor(sample->pose.translation().x() / grid_size);
    key[1] = floor(sample->pose.translation().y() / grid_size);
    key[2] = floor(sample->pose.rotation().angle() / grid_angle);

    AddParticleToCluster(clusters_[cluster_id++], key);
  }

  double max_weight = 0.;
  int max_weight_index = -1;
  for (int i = 0; i < clusters_.size(); i++)
  {
    double weight = 0.;
    double mean_x, mean_y, mean_cos, mean_sin;
    Eigen::Matrix<double, 3, 3> covariance = Eigen::Matrix<double, 3, 3>::Zero();

    for (auto sample : clusters_[i])
    {
      Eigen::Vector3d pose(sample->pose.translation().x(), sample->pose.translation().y(),
                           sample->pose.rotation().angle());

      weight += sample->weight;
      mean_x += sample->weight * pose.x();
      mean_y += sample->weight * pose.y();
      mean_cos += sample->weight * cos(pose.z());
      mean_sin += sample->weight * sin(pose.z());

      covariance = sample->weight * pose * pose.transpose();
    }
    Rigid2d mean_pose({mean_x / weight, mean_y / weight}, atan2(mean_sin, mean_cos));

    LOG(INFO) << "mean_pose: " << mean_pose << " weight: " << weight << "\n" << covariance;

    if (weight > max_weight)
    {
      max_weight = weight;
      max_weight_index = i;
      current_pose_ = mean_pose;
    }
  }
  LOG(INFO) << "current_pose_: " << current_pose_ << " max_weight: " << max_weight;
}

void ParticleFilter::AddParticleToCluster(std::vector<SampleConstPtr>& cluster, const int* key)
{
  int pose_key[3];
  double grid_size = 0.5;
  double grid_angle = 10 * M_PI / 180.;

  for (auto& sample : samples_)
  {
    if (sample->clustered)
      continue;

    pose_key[0] = floor(sample->pose.translation().x() / grid_size);
    pose_key[1] = floor(sample->pose.translation().y() / grid_size);
    pose_key[2] = floor(sample->pose.rotation().angle() / grid_angle);

    // new_key will generated from key[i]+-1
    for (int i = 0; i < 3 * 3 * 3; i++)
    {
      int new_key[3];
      new_key[0] = key[0] + (i / 9) - 1;
      new_key[1] = key[1] + ((i % 9) / 3) - 1;
      new_key[2] = key[2] + ((i % 9) % 3) - 1;

      if (new_key[0] == pose_key[0] and new_key[1] == pose_key[1]
          and new_key[2] == pose_key[2])
      {
        sample->clustered = true;
        cluster.emplace_back(sample);
        AddParticleToCluster(cluster, pose_key);
        break;
      }
    }
  }
}

} // namespace World
} // namespace IKid
