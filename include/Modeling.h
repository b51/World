/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: Modeling.h
*
*          Created On: Mon 03 Sep 2018 08:12:06 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef IKID_WORLD_MODELING_H_
#define IKID_WORLD_MODELING_H_

#include <math.h>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/transform/rigid_transform.h"

#include "ParticleFilter.h"

namespace IKid
{
namespace World
{
using Rigid2d = cartographer::transform::Rigid2d;

struct ModelingOptions
{
  double ball_diameter;
  ParticleFilterOptions particle_filter_options;
};

inline ModelingOptions CreateModelingOptions(
    cartographer::common::LuaParameterDictionary* const lua_parameter_dictionary)
{
  ModelingOptions options;
  options.particle_filter_options =
      CreateParticleFilterOptions(lua_parameter_dictionary->GetDictionary("particle_filter").get());
  return options;
}

class Modeling
{
public:
  Modeling(const ModelingOptions& options);
  ~Modeling();

  void Init();

public:

private:
  ModelingOptions options_;
  Rigid2d current_pose_;

};

} // namespace World
} // namespace IKid

#endif
