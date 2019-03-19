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

#include "ikhandler.h"
#include <thread>
#include <rbdl/rbdl.h>
#include <three-dim-util/opengl2/gl.hpp>
#include <three-dim-util/opengl2/draw-functions.hpp>
#include "core.h"
#include "joint.h"

#define THREAD_SAFE

/////////////////////////////////////////////////////////////////
// Important note on the Euler angle representation:
// Our system uses the Euler XYZ representation, in which a point
// (or a vector) is rotated around the x axis first, then y axis,
// and finally z axis. That is, the rotation matrix is calculated
// by R = R_z * R_y * R_x. This is the same as Maya's default. However,
// this does not work correctly with RBDL Inverse Kinematics; When
// I specify "EulerAngleXYZ" and calculate IK, RBDL returns incorrect
// results. On the other hand, when I specify "EulerAngleZYX" and
// then swap the x element and z element (like as in MATLAB), the
// result looks correct. I think this is a bug of RBDL, because Inverse
// Dynamics seems to work correctly with "EulerAngleXYZ".
/////////////////////////////////////////////////////////////////

namespace RBD = RigidBodyDynamics;
using Vec3 = RBD::Math::Vector3d;
using VecN = RBD::Math::VectorNd;
using Mat3 = RBD::Math::Matrix3d;

namespace
{
    Core& core = Core::GetInstance();
    
    std::vector<std::shared_ptr<Joint>> RetrieveTargetJoints(std::shared_ptr<Joint> ik_root_joint, std::shared_ptr<Joint> end_effector_joint)
    {
        std::list<std::shared_ptr<Joint>> joint_list;
        std::shared_ptr<Joint> joint = end_effector_joint;
        while (true)
        {
            joint_list.push_front(joint);
            
            if (joint == ik_root_joint)
            {
                break;
            }
            else
            {
                assert(!joint->parent_.expired());
                joint = joint->parent_.lock();
            }
        }
        std::vector<std::shared_ptr<Joint>> target_joints;
        for (auto j : joint_list)
        {
            target_joints.push_back(j);
        }
        return target_joints;
    }
    
    inline Eigen::Vector3d SwapXZ(const Eigen::Vector3d& vec)
    {
        return Eigen::Vector3d(vec(2), vec(1), vec(0));
    }
}

void IkHandler::Initialize(std::shared_ptr<Joint> ik_root_joint, std::shared_ptr<Joint> end_effector_joint)
{
    ik_root_joint_ = ik_root_joint;
    end_effector_joint_ = end_effector_joint;
    
    target_joints_ = RetrieveTargetJoints(ik_root_joint, end_effector_joint);
    
    for (auto joint : target_joints_)
    {
        if (joint == end_effector_joint_) continue;

        // Each joint should be associated to at most one IK handler
        assert(joint->associated_ik_handler_.expired());
        
        // Register the handler
        joint->associated_ik_handler_ = shared_from_this();
        
        // Reset FK keys
        joint->rot_x_.curve_ = Curve();
        joint->rot_y_.curve_ = Curve();
        joint->rot_z_.curve_ = Curve();
    }

    InitializeKinematicsModel();
}

std::vector<IkHandler::EulerAngle> IkHandler::ComputeInverseKinematics(double t, bool use_cache)
{
#ifdef THREAD_SAFE
    // For thread safe
    static std::mutex local_mutex;
#endif
    
    // Return the cached result if available (otherwise, compute inverse kinematics)
    if (use_cache)
    {
#ifdef THREAD_SAFE
        local_mutex.lock();
#endif
        const auto copy = cached_result;
        const bool flag = (cached_result.size() == model_->dof_count / 3) && (std::abs(t - cached_parameter) < 1e-12);
#ifdef THREAD_SAFE
        local_mutex.unlock();
#endif
        if (flag) return copy;
    }
    
    const Eigen::Affine3d affine = ik_root_joint_->parent_.expired() ? Eigen::Affine3d::Identity() : ik_root_joint_->parent_.lock()->GetAffineRelativeToWorld(t);
    
    const Eigen::Vector3d target_position_in_world(pos_x_.GetValue(t), pos_y_.GetValue(t), pos_z_.GetValue(t));
    const Eigen::Vector3d target_position_in_ik_root = affine.inverse() * target_position_in_world;

    VecN q_init   = VecN::Zero(model_->dof_count);
    VecN q_result = VecN::Zero(model_->dof_count);
    
    const Vec3 end_point = end_effector_joint_->offset_;
    
    // It is necessary to set lambda to a large value (> 0.5) for handling unreachable cases
    const double tolerance       = 1e-06;
    const double lambda          = 0.40;
    const int    max_evaluations = 20;
    
#ifdef THREAD_SAFE
    RBD::Model copied_model = *model_;
    RBD::InverseKinematics(copied_model, q_init, std::vector<unsigned>{ end_effector_joint_id_ }, std::vector<Vec3>{ end_point }, std::vector<Vec3>{ target_position_in_ik_root }, q_result, tolerance, lambda, max_evaluations);
#else
    RBD::InverseKinematics(*model_, q_init, std::vector<unsigned>{ end_effector_joint_id_ }, std::vector<Vec3>{ end_point }, std::vector<Vec3>{ target_position_in_ik_root }, q_result, tolerance, lambda, max_evaluations);
#endif

    const int n_ik_controlled_joints = model_->dof_count / 3;
    std::vector<EulerAngle> result(n_ik_controlled_joints);
    for (int i = 0; i < n_ik_controlled_joints; ++ i)
    {
        result[i] = SwapXZ(EulerAngle(q_result.segment<3>(i * 3))); // It seems that X and Z are swapped in the ZYX mode (like as MATLAB)
    }
    
    // Store the result as cache
#ifdef THREAD_SAFE
    local_mutex.lock();
#endif
    cached_result = result;
    cached_parameter = t;
#ifdef THREAD_SAFE
    local_mutex.unlock();
#endif
    
    return result;
}

void IkHandler::InitializeKinematicsModel()
{
    model_ = std::make_shared<RBD::Model>();

    unsigned parent_id = 0;
    for (int i = 0; i < static_cast<int>(target_joints_.size() - 1); ++ i)
    {
        std::shared_ptr<Joint> j = target_joints_[i + 1];
        
        const Vec3&  center_of_mass = j->center_of_mass_position_parent_;
        const Mat3&  inertia        = j->inertia_at_center_parent_;
        const double mass           = j->mass_;

        assert(j->parent_.lock() == target_joints_[i]);
        const Vec3 offset = j->parent_.lock()->offset_;

        const RBD::Body  rbd_body  = RBD::Body(mass, center_of_mass, inertia);
        const RBD::Joint rbd_joint = RBD::Joint(RBD::JointTypeEulerZYX); // I am not sure why XYZ does not work correctly

        parent_id = model_->AddBody(parent_id, RBD::Math::Xtrans(offset), rbd_joint, rbd_body);
    }
    end_effector_joint_id_ = parent_id;
}

void IkHandler::Draw(double t) const
{
    const Eigen::Vector3d target_position_in_world = GetPosition(t);

    const double radius = core.drawing_scale_ * 0.10;
    threedimutil::gl()->glColor3d(0.1, 0.1, 0.6);
    threedimutil::draw_sphere(radius, target_position_in_world);
}

void IkHandler::AddKeyframe(int t, const Eigen::Vector3d &position, double smoothness)
{
    pos_x_.curve_.AddControlPoint(ControlPoint(static_cast<double>(t), position(0), 0.0, - smoothness, smoothness));
    pos_y_.curve_.AddControlPoint(ControlPoint(static_cast<double>(t), position(1), 0.0, - smoothness, smoothness));
    pos_z_.curve_.AddControlPoint(ControlPoint(static_cast<double>(t), position(2), 0.0, - smoothness, smoothness));
}

Eigen::Vector3d IkHandler::GetPosition(double t) const
{
    return Eigen::Vector3d(pos_x_.GetValue(t), pos_y_.GetValue(t), pos_z_.GetValue(t));
}
