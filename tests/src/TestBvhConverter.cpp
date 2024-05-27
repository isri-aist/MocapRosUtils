/* Author: Masaki Murooka */

#include <MocapRosUtils/BvhConverter.h>

#include <ros/package.h>

#include <fstream>
#include <gtest/gtest.h>

using namespace MocapRosUtils;

TEST(TestBvhConverter, Test1)
{
  // Instantiate BvhConverter and load BVH file
  std::string bvh_filename = ros::package::getPath("mocap_ros_utils") + "/data/sample_walk.bvh";
  BvhConverter::Configuration bvh_converter_config;
  bvh_converter_config.print_level = 2;
  BvhConverter bvh_converter(bvh_filename, bvh_converter_config);
  EXPECT_EQ(156, bvh_converter.joint_traj_data_.names_.size());

  // Dump URDF file
  std::string urdf_str = bvh_converter.convertToUrdf("TestBvhConverter");
  EXPECT_TRUE(urdf_str.size() > 0);

  // Dump rosbag file
  ros::Time::init();
  bvh_converter.joint_traj_data_.dumpRosbag("/tmp/TestBvhConverter.bag");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
