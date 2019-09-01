/*
 *  OptiMo
 *
 *  Copyright (c) 2018 National Institute of Advanced Industrial Science and Technology (AIST)
 *  Authors of this file: Yuki Koyama <koyama.y@aist.go.jp>
 *
 *  This program is dual-licensed; You may use it under either LGPLv3 or
 *  our commercial license. See the LICENSE files for details.
 *
 */

#ifndef JOINT_H
#define JOINT_H

#include <memory>
#include <Eigen/Geometry>
#include "variable.h"
#include "curve.h"
#include "item.h"

class IkHandler;

class Joint final : public std::enable_shared_from_this<Joint>, public Item
{
public:
    /// Constructor for building from scratch
    Joint(std::shared_ptr<Joint> parent = nullptr,
          const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
          const Eigen::Vector3d& rotation = Eigen::Vector3d::Zero(),
          const std::string& name = "root") :
    parent_(parent),
    offset_(offset),
    rot_x_(rotation(0)),
    rot_y_(rotation(1)),
    rot_z_(rotation(2))
    {
        name_ = name;
        SetPhysicalParameters();
    }

    /// Constructor for building from JSON file
    Joint(std::shared_ptr<Joint> parent,
          const Eigen::Vector3d& offset,
          const Variable& rot_x,
          const Variable& rot_y,
          const Variable& rot_z,
          const std::string& name) :
    parent_(parent),
    offset_(offset),
    rot_x_(rot_x),
    rot_y_(rot_y),
    rot_z_(rot_z)
    {
        name_ = name;
        SetPhysicalParameters();
    }
    
    /// Get the joint angle at the specified time in the form of Euler XYZ rotation.
    /// \param t Time
    ///
    /// The angle is controlled by either FK or IK.
    Eigen::Vector3d GetEulerAngle(double t) const;
    
    Eigen::Matrix4d GetTransformationRelativeToParent(double t) const;
    Eigen::Matrix4d GetTransformationRelativeToWorld(double t) const;

    /// Get the affine transformation of the joint relative to the parent joint coordinates.
    /// \param t Time
    Eigen::Affine3d GetAffineRelativeToParent(double t) const;
    /// Get the affine transformation of the joint relative the world coordinates.
    /// \param t Time
    Eigen::Affine3d GetAffineRelativeToWorld(double t) const;
    
    // Draw in 3D view
    void Draw(double t) const;
    void Draw(int t) const { Draw(static_cast<double>(t)); }
    
    // Pre-defined relationship
    std::vector<std::shared_ptr<Joint>> children_;
    std::weak_ptr<Joint> parent_;
    
    // Generate a new joint, add it to the list, and return its reference
    // Used for the initialization only
    std::shared_ptr<Joint> AddChild(const Eigen::Vector3d& offset, const Eigen::Vector3d& rotation, const std::string& name);
    std::shared_ptr<Joint> AddChild(std::shared_ptr<Joint> child);
    
    /// Described in the parent joint space
    Eigen::Vector3d offset_;
    
    // Time-variant parameters (rotation described by Euler XYZ)
    /// x value of Euler XYZ rotation
    Variable rot_x_;
    /// y value of Euler XYZ rotation
    Variable rot_y_;
    /// z value of Euler XYZ rotation
    Variable rot_z_;

    // Utility functions
    std::vector<Variable*> GetVariablePointers() { return std::vector<Variable*>{ &rot_x_, &rot_y_, &rot_z_ }; }
    bool IsKeyframed() const { return rot_x_.IsKeyframed() || rot_y_.IsKeyframed() || rot_z_.IsKeyframed(); }

    // For IK handling
    /// If this is true, it means that the joint is controlled by IK instead of FK.
    bool IsControlledByIk() const { return !associated_ik_handler_.expired(); }
    std::weak_ptr<IkHandler> associated_ik_handler_;
    
    /// Physical property described in the parent joint space
    Eigen::Vector3d center_of_mass_position_parent_;
    /// Physical property described in the parent joint space
    Eigen::Matrix3d inertia_at_center_parent_;
    double mass_;
    
    Eigen::Vector3d GetCenterOfMassInWorld(double t) const;
    Eigen::Matrix3d GetOrientationInWorld(double t) const;
    Eigen::Affine3d GetConfigurationOfRigidBodyInWorld(double t) const;
    
    /// Calculates the position of this joint at time t in the world coordinates.
    /// \param t Time.
    Eigen::Vector3d GetPosition(double t) const;
    
private:
    
    struct Color
    {
        const Eigen::Vector3d joint_unselected {0.70, 0.70, 0.70};
        const Eigen::Vector3d joint_selected   {0.60, 0.00, 0.20};
        const Eigen::Vector3d bone_unselected  {0.50, 0.50, 0.50};
        const Eigen::Vector3d bone_selected    {0.50, 0.50, 0.50};
        const Eigen::Vector3d joint_ik_selected{0.10, 0.10, 0.60};
    } color_;
    
    void SetPhysicalParameters();
};

#endif // JOINT_H
