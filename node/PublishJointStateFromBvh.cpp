/* Author: Masaki Murooka */

#include <MocapRosUtils/BvhConverter.h>

using namespace MocapRosUtils;

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "publish_joint_state_from_bvh");
  ros::NodeHandle nh;

  BvhConverter::Configuration bvh_converter_config;
  bvh_converter_config.print_level = 0;
  bvh_converter_config.pos_scale = 1e-2;
  bvh_converter_config.urdf_cylinder_radius_list = {{"Spine", 0.03}, {"Spine1", 0.03}, {"Head", 0.1}};
  bvh_converter_config.disable_end_link_visual = false;

  std::string bvh_filename;
  nh.getParam("bvh_filename", bvh_filename);

  BvhConverter bvh_converter(bvh_filename, bvh_converter_config);
  nh.setParam("robot_description", bvh_converter.convertToUrdf("BvhRobot"));

  bvh_converter.joint_traj_data_.publishJointState(true);

  return 0;
}
