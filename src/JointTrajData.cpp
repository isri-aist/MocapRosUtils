/* Author: Masaki Murooka */

#include <MocapRosUtils/JointTrajData.h>

#include <ros/ros.h>

#include <rosbag/bag.h>

using namespace MocapRosUtils;

sensor_msgs::JointState JointTrajData::makeJointStateMsg(int frame_idx) const
{
  sensor_msgs::JointState js_msg;

  js_msg.header.stamp = ros::Time::now();
  js_msg.header.frame_id = "world";
  js_msg.name = names_;
  js_msg.position.resize(names_.size());
  for(size_t joint_idx = 0; joint_idx < names_.size(); joint_idx++)
  {
    js_msg.position[joint_idx] = pos_list_(frame_idx, joint_idx);
  }

  return js_msg;
}

void JointTrajData::publishJointState(bool loop) const
{
  // Setup publisher
  ros::NodeHandle nh;
  ros::Publisher js_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);

  // Publish loop
  ros::Rate rate(1.0 / dt_);
  while(ros::ok())
  {
    for(int frame_idx = 0; frame_idx < pos_list_.rows(); frame_idx++)
    {
      js_pub.publish(makeJointStateMsg(frame_idx));

      ros::spinOnce();
      rate.sleep();

      if(!ros::ok())
      {
        break;
      }
    }
    if(!loop)
    {
      break;
    }
  }
}

void JointTrajData::dumpRosbag(const std::string & rosbag_filename) const
{
  rosbag::Bag bag;
  bag.open(rosbag_filename, rosbag::bagmode::Write);

  for(int frame_idx = 0; frame_idx < pos_list_.rows(); frame_idx++)
  {
    bag.write("joint_states", ros::Time::now(), makeJointStateMsg(frame_idx));
  }

  bag.close();
}
