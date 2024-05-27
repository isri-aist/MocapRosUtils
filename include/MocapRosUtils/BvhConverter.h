/* Author: Masaki Murooka */

#pragma once

#include <RBDyn/parsers/urdf.h>

#include <MocapRosUtils/JointTrajData.h>

#include <ros/ros.h>

namespace MocapRosUtils
{
/** \brief Convert BVH channel to RBDyn joint type. */
rbd::Joint::OldType bvhChannelToRbdJointType(const std::string & str);

/** \brief Joint in BVH file. */
struct BvhJoint
{
  //! Name
  std::string name;

  //! Offset
  Eigen::Vector3d offset;

  //! RBDyn joint list
  std::vector<rbd::Joint> rbd_joint_list;

  //! Child BVH joints
  std::vector<std::shared_ptr<BvhJoint>> child_joints;

  //! Whether the joint is end
  bool is_end;

  /** \brief Constructor.
      \param _name name
      \param _is_end whether the joint is end
  */
  BvhJoint(const std::string & _name, bool _is_end) : name(_name), is_end(_is_end) {}
};

/** \brief Converter of BVH file. */
class BvhConverter
{
public:
  /** \brief Configuration. */
  struct Configuration
  {
    //! Print level (0: no print, 1: print only important, 2: print verbose, 3: print very verbose)
    int print_level = 1;

    //! Position scale
    double pos_scale = 1.0;

    //! Scale factor of cylinder radius in URDF file
    double urdf_cylinder_radius_scale_factor = 0.1;

    //! List of user-specified cylinder radius in URDF file [m]
    std::map<std::string, double> urdf_cylinder_radius_list;

    //! Whether to disable visual of end link
    bool disable_end_link_visual = false;

    /** \brief Constructor. */
    Configuration() {}
  };

public:
  /** \brief Constructor.
      \param filename BVH file name
      \param config configuration
  */
  BvhConverter(const std::string & filename, const Configuration & config = Configuration());

  /** \brief Convert BVH hierarchy to URDF.
      \param robot_name URDF robot name
  */
  std::string convertToUrdf(const std::string & robot_name) const;

protected:
  bool parse(const std::string & filename);

  bool parseBvhHierarchy(const std::vector<std::string> & hierarchy_str_list);

  std::shared_ptr<BvhJoint> makeBvhJoint(std::vector<std::string>::const_iterator & it, bool is_end);

  void addRbdBody(const std::shared_ptr<BvhJoint> & bvh_joint,
                  const std::string & parent_name,
                  rbd::MultiBodyGraph & mbg) const;

  void setRbdVisual(rbd::parsers::ParserResult & rbd_parser_result) const;

public:
  //! Configuration
  Configuration config_;

  //! BVH joint list
  std::vector<std::shared_ptr<BvhJoint>> bvh_joint_list_;

  //! Joint trajectory data
  JointTrajData joint_traj_data_;
};
} // namespace MocapRosUtils
