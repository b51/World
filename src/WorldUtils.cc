/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: WorldUtils.cc
*
*          Created On: Tue 04 Sep 2018 03:06:16 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "WorldUtils.h"

namespace IKid
{
namespace World
{

double RandomInRange(const double start, const double end)
{
  auto noise =
      std::bind(std::uniform_real_distribution<double>{start, end}
              , std::mt19937(std::random_device{}()));

  return noise();
}

double RandomInMirrorRange(const double range)
{
  return range * RandomInRange(-1.0, 1.0);
}

double RandomInGaussian(const double mean, const double var)
{
  auto noise =
      std::bind(std::normal_distribution<double>{mean, var}
              , std::mt19937(std::random_device{}()));
  return noise();
}

} // namespace World
} // namespace IKid
