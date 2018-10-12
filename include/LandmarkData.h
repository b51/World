/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: LandmarkData.h
*
*          Created On: Mon 03 Sep 2018 04:08:33 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef IKID_WORKD_LANDMARK_DATA_H_
#define IKID_WORKD_LANDMARK_DATA_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"

namespace IKid
{
namespace World
{
using Pose2d = Eigen::Matrix<double, 2, 1>;
namespace carto = cartographer;

const Pose2d kGoalPostOpponentLeft(4.5, 0.75);
const Pose2d kGoalPostOpponentRight(4.5, -0.75);
const Pose2d kGoalPostKeepLeft(-4.5, -0.75);
const Pose2d kGoalPostKeepRight(-4.5, 0.75);

struct LabeldPoint
{
  int label;
  Eigen::Vector2d point;
};

} // namespace World
} // namespace IKid

#endif
