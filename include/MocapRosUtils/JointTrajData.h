/* Author: Masaki Murooka */

#pragma once

#include <Eigen/Dense>

#include <sensor_msgs/JointState.h>

namespace MocapRosUtils
{
/** \brief Joint trajectory data. */
class JointTrajData
{
public:
  /** \brief Constructor. */
  JointTrajData() {}

  /** \brief Get number of steps. */
  inline int stepNum() const
  {
    return static_cast<int>(pos_list_.rows());
  }

  /** \brief Make joint state message.
      \param frame_idx frame index
   */
  sensor_msgs::JointState makeJointStateMsg(int frame_idx) const;

  /** \brief Publish joint state message.
      \param loop whether to loop infinitely
   */
  void publishJointState(bool loop = false) const;

  /** \brief Dump joint state message to a rosbag file.
      \param rosbag_filename Path of the rosbag file
   */
  void dumpRosbag(const std::string & rosbag_filename) const;

public:
  //! Sampling period [sec]
  double dt_ = 0;

  //! Joint name list
  std::vector<std::string> names_;

  /** \brief Joint position trajectory [m], [rad]

      A single row represents data for all joints at a specific time.
      A single column represents all the time data for a specific joint.
  */
  Eigen::MatrixXd pos_list_;
};
} // namespace MocapRosUtils
