/* Author: Masaki Murooka */

#include <MocapRosUtils/BvhConverter.h>

using namespace MocapRosUtils;

int main(int argc, char ** argv)
{
  if(argc != 3)
  {
    std::cout << "Usage:\n  " << argv[0] << " <bvh_file_path> <rosbag_file_path>" << std::endl;
    return 1;
  }
  std::string bvh_filename = argv[1];
  std::string rosbag_filename = argv[2];

  BvhConverter::Configuration bvh_converter_config;
  bvh_converter_config.print_level = 0;
  bvh_converter_config.pos_scale = 1e-2;
  bvh_converter_config.urdf_cylinder_radius_list = {{"Spine", 0.03}, {"Spine1", 0.03}, {"Head", 0.1}};
  bvh_converter_config.disable_end_link_visual = false;

  BvhConverter bvh_converter(bvh_filename, bvh_converter_config);

  ros::Time::init();
  bvh_converter.joint_traj_data_.dumpRosbag(rosbag_filename);

  return 0;
}
