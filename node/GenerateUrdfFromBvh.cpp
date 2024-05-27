/* Author: Masaki Murooka */

#include <MocapRosUtils/BvhConverter.h>

#include <fstream>

using namespace MocapRosUtils;

int main(int argc, char ** argv)
{
  if(argc != 4)
  {
    std::cout << "Usage:\n  " << argv[0] << " <bvh_file_path> <urdf_file_path> <urdf_robot_name>" << std::endl;
    return 1;
  }
  std::string bvh_filename = argv[1];
  std::string urdf_filename = argv[2];
  std::string robot_name = argv[3];

  BvhConverter::Configuration bvh_converter_config;
  bvh_converter_config.print_level = 1;
  bvh_converter_config.pos_scale = 1e-2;
  bvh_converter_config.urdf_cylinder_radius_list = {{"Spine", 0.03}, {"Spine1", 0.03}, {"Head", 0.1}};
  bvh_converter_config.disable_end_link_visual = true;

  BvhConverter bvh_converter(bvh_filename, bvh_converter_config);

  ROS_INFO_STREAM("Dump URDF file: " << urdf_filename);
  std::ofstream ofs(urdf_filename);
  ofs << bvh_converter.convertToUrdf(robot_name);

  return 0;
}
