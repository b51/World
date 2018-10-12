/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: WorldNodeMain.cc
*
*          Created On: Mon 03 Sep 2018 03:14:11 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "WorldNode.h"

DEFINE_string(configuration_directory, "",
                  "First directory in which configuration files are searched, "
                  "second is always the Cartographer installation to allow "
                  "including files from there.");
DEFINE_string(configuration_basename, "",
                  "Basename, i.e. not containing any directory prefix, of the "
                  "configuration file.");

using namespace IKid;
using namespace World;

int main(int argc, char* argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
        << "-configuration_directory is missing.";

  CHECK(!FLAGS_configuration_basename.empty())
        << "-configuration_basename is missing.";

  WorldOptions options =
        LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  FLAGS_minloglevel = options.log_level;
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  ::ros::init(argc, argv, "WorldNode");
  ::ros::start();

  WorldNode world_node(options);

  world_node.Init();
  world_node.Run();

  ::ros::spin();
  ::ros::shutdown();
}
