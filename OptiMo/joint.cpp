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

#include "joint.h"
#include <three-dim-util/opengl2/draw-functions.hpp>
#include <three-dim-util/opengl2/gl.hpp>
#include <three-dim-util/opengl2/gl-wrappers.hpp>
#include "core.h"
#include "ikhandler.h"

// 2017/09/04:
// The original implementation was based on Euler ZYX, and it worked correctly.
// However, as the default in Maya is Euler XYZ, I had to change it to Euler XYZ.
// In Euler XYZ, the rotation matrix is like R = R_z * R_y * R_x.

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::AngleAxisd;
using Eigen::Affine3d;

namespace
{
    Core& core = Core::GetInstance();
}

Vector3d Joint::GetEulerAngle(double t) const
{
    if (IsControlledByIk())
    {
        // Calling inverse kinematics for every IK joint may look redundant but it effectively uses cache so this is not that bad
        const std::vector<Eigen::Vector3d> euler_angles = associated_ik_handler_.lock()->ComputeInverseKinematics(t);
        const int index = associated_ik_handler_.lock()->GetIndex(shared_from_this());
        return euler_angles[index];
    }
    else
    {
        const double rot_x_t = rot_x_.GetValue(t);
        const double rot_y_t = rot_y_.GetValue(t);
        const double rot_z_t = rot_z_.GetValue(t);
        return Vector3d(rot_x_t, rot_y_t, rot_z_t);
    }
}

Matrix4d Joint::GetTransformationRelativeToParent(double t) const
{
    return GetAffineRelativeToParent(t).matrix();
}

Matrix4d Joint::GetTransformationRelativeToWorld(double t) const
{
    return GetAffineRelativeToWorld(t).matrix();
}

Affine3d Joint::GetAffineRelativeToParent(double t) const
{
    // Note: affine = trans * rotate_z * rotate_y * rotate_x
    const Vector3d euler_angle = GetEulerAngle(t);
    Affine3d affine = Affine3d::Identity();
    affine.translate(offset_);
    affine.rotate(AngleAxisd(euler_angle(2), Vector3d::UnitZ()));
    affine.rotate(AngleAxisd(euler_angle(1), Vector3d::UnitY()));
    affine.rotate(AngleAxisd(euler_angle(0), Vector3d::UnitX()));
    return affine;
}

Affine3d Joint::GetAffineRelativeToWorld(double t) const
{
    if (parent_.expired())
    {
        return GetAffineRelativeToParent(t);
    }
    return parent_.lock()->GetAffineRelativeToWorld(t) * GetAffineRelativeToParent(t);
}

Eigen::Vector3d Joint::GetPosition(double t) const
{
    return GetAffineRelativeToWorld(t).translation();
}

void Joint::SetPhysicalParameters()
{
    // Heuristic definition of physical properties
    // Assuming a rectangular with the size of 10 cm * 10 cm * {offset.norm()} m and the unit density
    
    const double x_body = 0.1;
    const double y_body = offset_.norm();
    const double z_body = 0.1;
    
    if (parent_.expired())
    {
        mass_ = 0.0;
        center_of_mass_position_parent_ = Vector3d::Zero();
        inertia_at_center_parent_ = Matrix3d::Zero();
    }
    else
    {
        mass_ = 1000.0 * x_body * z_body * offset_.norm();
        
        center_of_mass_position_parent_ = 0.5 * offset_;
        
        // R rotates the offset vector direction to the y axis
        const Matrix3d R = Eigen::Quaterniond::FromTwoVectors(offset_.normalized(), Vector3d::UnitY()).toRotationMatrix();
        
        // I_body is the inertia tensor of a rectangular shape that aligns to the y axis
        const Matrix3d I_body = (mass_ / 12.0) * Vector3d(y_body * y_body + z_body * z_body, x_body * x_body + z_body * z_body, x_body * x_body + y_body * y_body).asDiagonal();
        
        inertia_at_center_parent_ = R * I_body * R.transpose();
    }
}

Vector3d Joint::GetCenterOfMassInWorld(double t) const
{
    if (parent_.expired())
    {
        return offset_ + center_of_mass_position_parent_;
    }
    const Affine3d T = parent_.lock()->GetAffineRelativeToWorld(t);
    return T * center_of_mass_position_parent_;
}

Matrix3d Joint::GetOrientationInWorld(double t) const
{
    if (parent_.expired())
    {
        return GetTransformationRelativeToParent(t).block<3, 3>(0, 0);
    }
    const Matrix4d T = parent_.lock()->GetTransformationRelativeToWorld(t);
    return T.block<3, 3>(0, 0);
}

Eigen::Affine3d Joint::GetConfigurationOfRigidBodyInWorld(double t) const
{
    if (parent_.expired())
    {
        return GetAffineRelativeToParent(t);
    }
    const Affine3d T = parent_.lock()->GetAffineRelativeToWorld(t);
    auto a = Affine3d::Identity();
    a.translate(center_of_mass_position_parent_);
    return T * a;
}

void Joint::Draw(double t) const
{
    threedimutil::gl()->glMatrixMode(GL_MODELVIEW);
    threedimutil::gl()->glPushMatrix();
    
    if (!parent_.expired())
    {
        const double radius = core.drawing_scale_ * 0.02;
        threedimutil::color_3d(is_selected_ ? color_.bone_selected : color_.bone_unselected);
        threedimutil::draw_cylinder(radius, offset_, Vector3d::Zero());
    }
    
    const Affine3d mat = GetAffineRelativeToParent(t);
    threedimutil::gl()->glMultMatrixd(mat.data());
    
    if (IsControlledByIk() && associated_ik_handler_.lock()->is_selected_)
    {
        threedimutil::color_3d(color_.joint_ik_selected);
    }
    else
    {
        threedimutil::color_3d(is_selected_ ? color_.joint_selected : color_.joint_unselected);
    }
    const double cube_size = core.drawing_scale_ * 0.10;
    threedimutil::draw_cube(cube_size);
    
    for (const auto& child : children_)
    {
        child->Draw(t);
    }
    
    threedimutil::gl()->glPopMatrix();
}

std::shared_ptr<Joint> Joint::AddChild(const Vector3d& offset, const Vector3d& rotation, const std::string& name)
{
    children_.push_back(std::make_shared<Joint>(shared_from_this(), offset, rotation, name));
    return children_[children_.size() - 1];
}

std::shared_ptr<Joint> Joint::AddChild(std::shared_ptr<Joint> child)
{
    children_.push_back(child);
    return children_[children_.size() - 1];
}
