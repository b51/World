/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: WorldOptions.h
*
*          Created On: Mon 03 Sep 2018 03:40:38 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef IKID_WORLD_OPTIONS_H_
#define IKID_WORLD_OPTIONS_H_

#include <string>
#include <vector>
#include <tuple>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"

namespace IKid
{
namespace World
{
namespace carto = cartographer;

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

struct WorldOptions
{
  int log_level;

  ParticleFilterOptions particle_filter_options;
};

inline WorldOptions CreateWorldOptions(carto::common::LuaParameterDictionary* const
                                        lua_parameter_dictionary)
{
  WorldOptions options;
  options.log_level = lua_parameter_dictionary->GetInt("log_level");
  options.particle_filter_options =
      CreateParticleFilterOptions(lua_parameter_dictionary->GetDictionary("particle_filter").get());
  return options;
}

inline WorldOptions LoadOptions(const std::string& configuration_directory
                              , const std::string& configuration_basename)
{
  auto file_resolver = carto::common::make_unique<
      carto::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  carto::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return CreateWorldOptions(&lua_parameter_dictionary);
}

} // namespace World
} // namespace IKid

#endif
