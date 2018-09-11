--########################################################################
--
--              Author: b51
--                Mail: b51live@gmail.com
--            FileName: WorldConfig.lua
--
--          Created On: Wed Jul 18 16:01:15 2018
--     Licensed under The MIT License [see LICENSE for details]
--
--########################################################################

PARTICLE_FILTER =
{
  particles_num = 50,
  gaussian_noise_mean = 0.,
  gaussian_noise_var = 0.01,
}

MODELING =
{
  particle_filter = PARTICLE_FILTER,
}

world_options =
{
  log_level = 0,
  modeling = MODELING,
}

return world_options;
