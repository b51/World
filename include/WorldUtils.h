/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: WorldUtils.h
*
*          Created On: Tue 04 Sep 2018 02:57:03 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include <iostream>
#include <math.h>
#include <random>
#include <functional>

namespace IKid
{
namespace World
{

double RandomInRange(const double start, const double end);

double RandomInMirrorRange(const double range);

double RandomInGaussian(const double mean, const double var);

} // namespace World
} // namespace IKid
