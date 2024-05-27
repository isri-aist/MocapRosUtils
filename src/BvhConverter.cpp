/* Author: Masaki Murooka */

#include <MocapRosUtils/BvhConverter.h>
#include <MocapRosUtils/StringUtils.h>

#include <algorithm>
#include <fstream>

using namespace MocapRosUtils;

namespace
{
Eigen::Matrix3d rotationMatrixFromZAxis(const Eigen::Vector3d & z_axis)
{
  Eigen::Matrix3d mat;
  mat.row(2) = z_axis.normalized();
  mat.row(0) = mat.row(2).cross(Eigen::Vector3d::UnitX());
  if(mat.row(0).norm() < 1e-3)
  {
    mat.row(0) = mat.row(2).cross(Eigen::Vector3d::UnitY());
  }
  mat.row(0).normalize();
  mat.row(1) = mat.row(2).cross(mat.row(0)).normalized();
  return mat;
}
} // namespace

rbd::Joint::OldType MocapRosUtils::bvhChannelToRbdJointType(const std::string & str)
{
  if(str == "Xposition")
  {
    return rbd::Joint::OldType::PrismX;
  }
  else if(str == "Yposition")
  {
    return rbd::Joint::OldType::PrismY;
  }
  else if(str == "Zposition")
  {
    return rbd::Joint::OldType::PrismZ;
  }
  else if(str == "Xrotation")
  {
    return rbd::Joint::OldType::RevX;
  }
  else if(str == "Yrotation")
  {
    return rbd::Joint::OldType::RevY;
  }
  else if(str == "Zrotation")
  {
    return rbd::Joint::OldType::RevZ;
  }
  else
  {
    throw std::runtime_error("[bvhChannelToRbdJointType] Invalid BVH channel: " + str);
  }
}

BvhConverter::BvhConverter(const std::string & filename, const Configuration & config) : config_(config)
{
  parse(filename);
}

bool BvhConverter::parse(const std::string & filename)
{
  if(config_.print_level >= 1)
  {
    ROS_INFO_STREAM("[BvhConverter::parse] Load BVH file: " << filename);
  }

  // Open file
  std::ifstream ifs(filename);
  if(!ifs)
  {
    ROS_ERROR_STREAM("[BvhConverter::parse] Cannot open the file: " << filename);
    return false;
  }

  std::string line_str;
  std::string content_phase = ""; // "HIERARCHY" or "MOTION"
  std::vector<std::string> hierarchy_str_list;
  int frame_num = 0;
  int frame_idx = 0;
  joint_traj_data_.pos_list_.resize(0, 0);
  while(std::getline(ifs, line_str, '\n'))
  {
    // Treat newline code for Windows (i.e., CRLF)
    if(line_str.back() == '\r')
    {
      line_str.pop_back();
    }

    // Skip empty line
    if(line_str.empty())
    {
      continue;
    }

    // Replace tab with space
    std::replace(std::begin(line_str), std::end(line_str), '\t', ' ');

    // Update content_phase
    if(line_str == "HIERARCHY")
    {
      if(content_phase != "")
      {
        ROS_ERROR_STREAM("[BvhConverter::parse] HIERARCHY must be first, but current content_phase is {}"
                         << content_phase);
        return false;
      }
      content_phase = "HIERARCHY";
      continue;
    }
    else if(line_str == "MOTION")
    {
      if(content_phase != "HIERARCHY")
      {
        ROS_ERROR_STREAM("[BvhConverter::parse] MOTION must come after HIERARCHY, but current content_phase is "
                         << content_phase);
        return false;
      }
      content_phase = "MOTION";
      continue;
    }
    if(content_phase == "")
    {
      ROS_ERROR_STREAM("[BvhConverter::parse] content_phase is empty.");
      return false;
    }

    // Store each content
    if(content_phase == "HIERARCHY")
    {
      splitStr(line_str, ' ', std::back_inserter(hierarchy_str_list), true);
    }
    else // if(content_phase == "MOTION")
    {
      std::vector<std::string> motion_str_list = splitStr(line_str, ' ', true);
      if(motion_str_list.size() > 1 && motion_str_list[0] == "Frames:")
      {
        frame_num = std::stoi(motion_str_list[1]);
      }
      else if(motion_str_list.size() > 2 && motion_str_list[0] == "Frame" && motion_str_list[1] == "Time:")
      {
        joint_traj_data_.dt_ = std::stod(motion_str_list[2]);
      }
      else
      {
        if(joint_traj_data_.pos_list_.size() == 0)
        {
          joint_traj_data_.pos_list_.resize(frame_num, motion_str_list.size());
        }
        else
        {
          if(joint_traj_data_.pos_list_.cols() != static_cast<int>(motion_str_list.size()))
          {
            ROS_ERROR_STREAM("[BvhConverter::parse] Motion dimensions change: " << joint_traj_data_.pos_list_.cols()
                                                                                << " != " << motion_str_list.size());
          }
        }
        for(int col_idx = 0; col_idx < joint_traj_data_.pos_list_.cols(); col_idx++)
        {
          joint_traj_data_.pos_list_(frame_idx, col_idx) = std::stod(motion_str_list[col_idx]);
        }
        frame_idx++;
      }
    }
  }

  // Parse hierarchy
  parseBvhHierarchy(hierarchy_str_list);
  if(joint_traj_data_.pos_list_.cols() != static_cast<int>(joint_traj_data_.names_.size()))
  {
    ROS_ERROR_STREAM("[BvhConverter::parse] Hierarchy and motion dimensions are inconsistent: "
                     << joint_traj_data_.pos_list_.cols() << " != " << joint_traj_data_.names_.size());
  }

  // Convert motion unit
  for(int col_idx = 0; col_idx < joint_traj_data_.pos_list_.cols(); col_idx++)
  {
    if(joint_traj_data_.names_[col_idx].find("rotation") != std::string::npos)
    {
      joint_traj_data_.pos_list_.col(col_idx) *= M_PI / 180.0;
    }
    else if(joint_traj_data_.names_[col_idx].find("position") != std::string::npos)
    {
      joint_traj_data_.pos_list_.col(col_idx) *= config_.pos_scale;
    }
  }

  return true;
}

bool BvhConverter::parseBvhHierarchy(const std::vector<std::string> & hierarchy_str_list)
{
  joint_traj_data_.names_.clear();

  std::vector<std::string>::const_iterator it = hierarchy_str_list.begin();

  // Check first element
  if(*it != "ROOT")
  {
    ROS_ERROR_STREAM("[BvhConverter::parseBvhHierarchy] First element of hierarchy_str_list must be \"ROOT\".");
    return false;
  }
  it++;

  makeBvhJoint(it, false);

  if(it != hierarchy_str_list.end())
  {
    ROS_ERROR_STREAM("[BvhConverter::parseBvhHierarchy] Iterator should be end, but next value is \"" << *it << "\".");
    return false;
  }

  if(config_.print_level >= 2)
  {
    ROS_INFO_STREAM(
        "[BvhConverter::parseBvhHierarchy] joint_traj_data_.names_: (num: " << joint_traj_data_.names_.size() << ")");
    for(const auto & rbd_joint_name : joint_traj_data_.names_)
    {
      ROS_INFO_STREAM("[BvhConverter::parseBvhHierarchy]   - " << rbd_joint_name);
    }
  }

  return true;
}

std::shared_ptr<BvhJoint> BvhConverter::makeBvhJoint(std::vector<std::string>::const_iterator & it, bool is_end)
{
  if(config_.print_level >= 3)
  {
    ROS_INFO_STREAM("[BvhConverter::makeBvhJoint] - Make BvhJoint " << *it);
  }
  const auto bvh_joint = std::make_shared<BvhJoint>(*it, is_end); // Not reference
  bvh_joint_list_.push_back(bvh_joint);
  it++;

  if(*it != "{")
  {
    ROS_ERROR_STREAM("[BvhConverter::makeBvhJoint] Expect \"{\" but \"" << *it << "\"");
  }
  it++;

  while(true)
  {
    if(*it == "OFFSET")
    {
      if(config_.print_level >= 3)
      {
        ROS_INFO_STREAM("[BvhConverter::makeBvhJoint]   - Set " << bvh_joint->name << "->offset");
      }

      it++;
      for(int i = 0; i < 3; i++)
      {
        bvh_joint->offset[i] = config_.pos_scale * std::stod(*it);
        it++;
      }
    }
    else if(*it == "CHANNELS")
    {
      if(config_.print_level >= 3)
      {
        ROS_INFO_STREAM("[BvhConverter::makeBvhJoint]   - Set " << bvh_joint->name << "->channels");
      }

      it++;
      int num_channels = std::stoi(*it);
      it++;
      for(int i = 0; i < num_channels; i++)
      {
        std::string rbd_joint_name = bvh_joint->name + "_" + *it;
        bvh_joint->rbd_joint_list.push_back(rbd::Joint(bvhChannelToRbdJointType(*it), true, rbd_joint_name));
        joint_traj_data_.names_.push_back(rbd_joint_name);
        it++;
      }
    }
    else if(*it == "JOINT")
    {
      it++;
      bvh_joint->child_joints.push_back(makeBvhJoint(it, false));
    }
    else if(*it == "END" || *it == "End")
    {
      it++;
      bvh_joint->child_joints.push_back(makeBvhJoint(it, true));
    }
    else if(*it == "}")
    {
      if(config_.print_level >= 3)
      {
        ROS_INFO_STREAM("[BvhConverter::makeBvhJoint] - End BvhJoint " << bvh_joint->name);
      }

      it++;
      return bvh_joint;
    }
    else
    {
      ROS_ERROR_STREAM("[BvhConverter::makeBvhJoint]  Unexpect string \"" << *it << "\"");
    }
  }
}

std::string BvhConverter::convertToUrdf(const std::string & robot_name) const
{
  // Set MultiBodyGraph
  rbd::MultiBodyGraph mbg;

  // Add virtual root link
  std::string root_link_name = "base_link";
  rbd::Body rbd_body_root(0.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), root_link_name);
  mbg.addBody(rbd_body_root);

  // Add each link
  addRbdBody(bvh_joint_list_.front(), root_link_name, mbg);

  // Set MultiBody and MultiBodyConfig
  rbd::MultiBody mb = mbg.makeMultiBody(root_link_name, true);
  rbd::MultiBodyConfig mbc(mb);
  mbc.zero(mb);

  // Set Limits
  rbd::parsers::Limits limits;
  for(const auto & joint_name : joint_traj_data_.names_)
  {
    limits.lower[joint_name] = std::vector<double>{-1e2};
    limits.upper[joint_name] = std::vector<double>{1e2};
    limits.velocity[joint_name] = std::vector<double>{1e2};
    limits.torque[joint_name] = std::vector<double>{1e2};
  }

  // Make ParserResult
  rbd::parsers::ParserResult rbd_parser_result;
  rbd_parser_result.mb = mb;
  rbd_parser_result.mbc = mbc;
  rbd_parser_result.mbg = mbg;
  rbd_parser_result.limits = limits;
  rbd_parser_result.name = robot_name;
  setRbdVisual(rbd_parser_result);

  return rbd::parsers::to_urdf(rbd_parser_result);
}

void BvhConverter::addRbdBody(const std::shared_ptr<BvhJoint> & bvh_joint,
                              const std::string & parent_name,
                              rbd::MultiBodyGraph & mbg) const
{
  std::string parent_body_name = parent_name;
  for(size_t i = 0; i < bvh_joint->rbd_joint_list.size(); i++)
  {
    // Add joint
    const auto & rbd_joint = bvh_joint->rbd_joint_list[i];
    mbg.addJoint(rbd_joint);

    // Add body
    bool is_last = (i == bvh_joint->rbd_joint_list.size() - 1);
    std::string body_name = (is_last ? bvh_joint->name : rbd_joint.name());
    rbd::Body rbd_body = (is_last ? rbd::Body(1.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(), body_name)
                                  : rbd::Body(0.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), body_name));
    mbg.addBody(rbd_body);

    // Connect bodies with joint
    sva::PTransformd rel_trans = (i == 0 ? sva::PTransformd(bvh_joint->offset) : sva::PTransformd::Identity());
    mbg.linkBodies(parent_body_name, rel_trans, body_name, sva::PTransformd::Identity(), rbd_joint.name());
    parent_body_name = body_name;
  }

  for(const auto & child_bvh_joint : bvh_joint->child_joints)
  {
    addRbdBody(child_bvh_joint, bvh_joint->name, mbg);
  }
}

void BvhConverter::setRbdVisual(rbd::parsers::ParserResult & rbd_parser_result) const
{
  for(const auto & bvh_joint : bvh_joint_list_)
  {
    std::vector<rbd::parsers::Visual> rbd_visual_list;
    for(const auto & child_bvh_joint : bvh_joint->child_joints)
    {
      if(config_.disable_end_link_visual && child_bvh_joint->is_end)
      {
        continue;
      }

      constexpr double visual_length_thre = 1e-3; // [m]
      if(child_bvh_joint->offset.norm() < visual_length_thre)
      {
        continue;
      }

      rbd::parsers::Material::Color color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;

      rbd::parsers::Visual rbd_visual_cylinder;
      rbd_visual_cylinder.name = bvh_joint->name;
      rbd_visual_cylinder.origin.rotation() = rotationMatrixFromZAxis(child_bvh_joint->offset).transpose();
      rbd_visual_cylinder.origin.translation() = 0.5 * child_bvh_joint->offset;
      rbd_visual_cylinder.geometry.type = rbd::parsers::Geometry::CYLINDER;
      rbd::parsers::Geometry::Cylinder cylinder;
      if(config_.urdf_cylinder_radius_list.count(child_bvh_joint->name))
      {
        cylinder.radius =
            std::min(config_.urdf_cylinder_radius_list.at(child_bvh_joint->name), 0.5 * child_bvh_joint->offset.norm());
      }
      else
      {
        cylinder.radius = config_.urdf_cylinder_radius_scale_factor * child_bvh_joint->offset.norm();
      }
      cylinder.length = child_bvh_joint->offset.norm() - 2 * cylinder.radius;
      rbd_visual_cylinder.geometry.data = cylinder;
      rbd_visual_cylinder.material.type = rbd::parsers::Material::Type::COLOR;
      rbd_visual_cylinder.material.data = color;
      rbd_visual_list.push_back(rbd_visual_cylinder);

      for(int sphere_idx = 0; sphere_idx < 2; sphere_idx++)
      {
        rbd::parsers::Visual rbd_visual_sphere;
        rbd_visual_sphere.name = bvh_joint->name;
        rbd_visual_sphere.origin.rotation().setIdentity();
        if(sphere_idx == 0)
        {
          rbd_visual_sphere.origin.translation() = cylinder.radius * child_bvh_joint->offset.normalized();
        }
        else
        {
          rbd_visual_sphere.origin.translation() =
              (child_bvh_joint->offset.norm() - cylinder.radius) * child_bvh_joint->offset.normalized();
        }
        rbd_visual_sphere.geometry.type = rbd::parsers::Geometry::SPHERE;
        rbd::parsers::Geometry::Sphere sphere;
        sphere.radius = cylinder.radius;
        rbd_visual_sphere.geometry.data = sphere;
        rbd_visual_sphere.material.type = rbd::parsers::Material::Type::COLOR;
        rbd_visual_sphere.material.data = color;
        rbd_visual_list.push_back(rbd_visual_sphere);
      }
    }

    rbd_parser_result.visual.emplace(bvh_joint->name, rbd_visual_list);
  }
}
