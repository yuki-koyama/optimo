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

#ifndef IKHANDLER_H
#define IKHANDLER_H

#include <memory>
#include <Eigen/Core>
#include "variable.h"
#include "item.h"
#include "joint.h"

namespace RigidBodyDynamics
{
    struct Model;
}

/// \brief Class for managing Inverse Kinematics for a single kinematic chain
class IkHandler : public std::enable_shared_from_this<IkHandler>, public Item
{
public:
    IkHandler(const std::string& name)
    {
        name_ = name;
    }
    
    /// Initialize this IK handler using the specified joints.
    /// This method should be called right after instantiated.
    /// \param ik_root_joint The uppermost joint that will be controlled by IK
    /// \param end_effector_joint The joint that acts as an end effector
    void Initialize(std::shared_ptr<Joint> ik_root_joint, std::shared_ptr<Joint> end_effector_joint);
    
    // Time-variant parameters
    /// x value of the target position in the world coordinates
    Variable pos_x_;
    /// y value of the target position in the world coordinates
    Variable pos_y_;
    /// z value of the target position in the world coordinates
    Variable pos_z_;
    
    using EulerAngle = Eigen::Vector3d;

    /// Compute Inverse Kinematics (IK) by using the damped least square method.
    /// \param t Time
    /// \param use_cache Whether use cache to improve performance or not
    ///
    /// This method is not marked as const to enable the cache function.
    /// It immediately returns the cached result if the input time is the same as the previous call; otherwise, it computes Inverse Kinematics and stores the result as cache.
    std::vector<EulerAngle> ComputeInverseKinematics(double t, bool use_cache = true);
    
    int GetIndex(std::shared_ptr<const Joint> joint) const
    {
        for (int i = 0; i < static_cast<int>(target_joints_.size()); ++ i)
        {
            if (target_joints_[i] == joint) return i;
        }
        assert(false);
        return 0;
    }
    
    /// Add a new keyframe at the specified time
    /// \param t Time
    /// \param position Position of the IK handle
    /// \param smoothness Smoothness of the resulting Bezier curve
    /// This is a utility method to easily keyframe the IK handle motion.
    void AddKeyframe(int t, const Eigen::Vector3d& position, double smoothness = 1.0);
    
    // Utility functions
    std::vector<Variable*> GetVariablePointers() { return std::vector<Variable*>{ &pos_x_, &pos_y_, &pos_z_ }; }

    /// Visualize the IK handle.
    /// \param t Time
    void Draw(double t) const;
    
    const std::string& GetRootName() const { return ik_root_joint_->GetName(); }
    const std::string& GetEndEffectorName() const { return end_effector_joint_->GetName(); }
    
    const std::vector<std::shared_ptr<Joint>>& GetTargetJoints() const { return target_joints_; }
    
    /// \param t Time
    Eigen::Vector3d GetPosition(double t) const;
   
private:
    std::vector<EulerAngle> cached_result;
    double cached_parameter;
    
    std::shared_ptr<Joint> ik_root_joint_;
    std::shared_ptr<Joint> end_effector_joint_;
    
    // From the root joint to the end effector joint
    std::vector<std::shared_ptr<Joint>> target_joints_;
    
    std::shared_ptr<RigidBodyDynamics::Model> model_;
    unsigned end_effector_joint_id_;
    
    void InitializeKinematicsModel();
};

#endif // IKHANDLER_H
