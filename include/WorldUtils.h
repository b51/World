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

/**
 *  Will return data in (start, end) with uniform distribution.
 *
 *  @param[in] start
 *  @param[in] end
 */
double RandomInRange(const double start, const double end);

/**
 *  Will return data in (-range, range) with uniform distribution.
 *
 *  @param[in] range
 */
double RandomInMirrorRange(const double range);

/**
 *  Will return in gaussian distribution with (mean, var).
 *
 *  @param[in] mean gaussian distribution mean
 *  @param[in] var gaussian distribution var
 */
double RandomInGaussian(const double mean, const double var);

} // namespace World
} // namespace IKid
