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

#include "core.h"
#include <rbdl/rbdl.h>
#include <parallel-util.hpp>

namespace RBD = RigidBodyDynamics;
using Vec3 = RBD::Math::Vector3d;
using VecN = RBD::Math::VectorNd;
using Mat3 = RBD::Math::Matrix3d;

// #define SECOND_ORDER_ACCURATE
#define MULTI_THREAD
#define THREAD_SAFE

namespace
{
    /// \param[out] results Results. The memory should be allocated beforehand.
    template<typename T>
    void CalculateFirstCentral(double dx, const std::vector<T>& values, std::vector<T>* results)
    {
        assert(values.size() > 3);
        assert(values.size() == results->size());
        const int n = static_cast<int>(values.size());

#ifdef SECOND_ORDER_ACCURATE
        // Non boundary
        for (int i = 1; i < n - 1; ++ i)
        {
            results->at(i) = (values[i + 1] - values[i - 1]) / (2.0 * dx);
        }
        
        // Boundary (by forward/backward difference)
        results->at(0)     = (values[1] - values[0]) / dx;
        results->at(n - 1) = (values[n - 1] - values[n - 2]) / dx;
#else
        // Non boundary
        for (int i = 2; i < n - 2; ++ i)
        {
            results->at(i) = (- values[i + 2] + 8.0 * values[i + 1] - 8.0 * values[i - 1] + values[i - 2]) / (12.0 * dx);
        }

        // Boundary
        for (int i : { 1, n - 2 })
        {
            results->at(i) = (values[i + 1] - values[i - 1]) / (2.0 * dx);
        }
        results->at(0)     = (values[1] - values[0]) / dx;
        results->at(n - 1) = (values[n - 1] - values[n - 2]) / dx;
#endif
    }

    /// \param[out] results Results. The memory should be allocated beforehand.
    template<typename T>
    void CalculateSecondCentral(double dx, const std::vector<T>& values, std::vector<T>* results)
    {
        assert(values.size() > 3);
        assert(values.size() == results->size());
        const int n = static_cast<int>(values.size());

#ifdef SECOND_ORDER_ACCURATE
        // Non boundary
        for (int i = 1; i < n - 1; ++ i)
        {
            results->at(i) = (values[i + 1] - 2.0 * values[i] + values[i - 1]) / (dx * dx);
        }
        
        // Boundary (by forward/backward difference)
        results->at(0)     = (values[2] - 2.0 * values[1] + values[0]) / (2.0 * dx * dx);
        results->at(n - 1) = (values[n - 1] - 2.0 * values[n - 2] + values[n - 3]) / (2.0 * dx * dx);
#else
        // Non boundary
        for (int i = 2; i < n - 2; ++ i)
        {
            results->at(i) = (- values[i + 2] + 16.0 * values[i + 1] - 30.0 * values[i] + 16.0 * values[i - 1] - values[i - 2]) / (12.0 * dx * dx);
        }

        // Boundary (by forward/backward difference)
        for (int i : { 1, n - 2 })
        {
            results->at(i) = (values[i + 1] - 2.0 * values[i] + values[i - 1]) / (dx * dx);
        }
        results->at(0)     = (values[2] - 2.0 * values[1] + values[0]) / (2.0 * dx * dx);
        results->at(n - 1) = (values[n - 1] - 2.0 * values[n - 2] + values[n - 3]) / (2.0 * dx * dx);
#endif
    }

    double CalculateFirstCentral(double v_prev, double v_succ, double eps = 0.10)
    {
        return (v_succ - v_prev) / (2.0 * eps);
    }
    
    double CalculateSecondCentral(double v, double v_prev, double v_succ, double eps = 0.10)
    {
        return (v_succ - 2.0 * v + v_prev) / (eps * eps);
    }
}

void Core::SetStates(const std::list<std::shared_ptr<Joint>>& joints,
                     double min_t,
                     double max_t,
                     double dt,
                     int n_dof,
                     std::vector<double>* time_sequence,
                     std::vector<VecN>* q,
                     std::vector<VecN>* q_dot,
                     std::vector<VecN>* q_ddot,
                     std::vector<VecN>* p)
{
    // Determine sampling points
    time_sequence->clear();
    time_sequence->reserve(1 + static_cast<int>((max_t - min_t) / dt));
    for (double t = min_t; t <= max_t; t += dt) time_sequence->push_back(t);
    const int n_samples = time_sequence->size();
    
    // Allocate vectors
    q     ->resize(n_samples, VecN::Zero(n_dof));
    q_dot ->resize(n_samples, VecN::Zero(n_dof));
    q_ddot->resize(n_samples, VecN::Zero(n_dof));
    if (p != nullptr) p->resize(n_samples, VecN::Zero(joints.size() * 3));
    
    // Compute FK/IK for q (and p if necessary)
    auto lambda = [q, &joints, time_sequence, p](int time_index)
    {
        int joint_index = 0;
        for (const auto& joint : joints)
        {
            if (joint->parent_.expired())
            {
                assert(joint_index == 0);
                (*q)[time_index].segment(0, 3) = joint->offset_;
            }
            else
            {
                const Eigen::Vector3d euler_angle = joint->GetEulerAngle((*time_sequence)[time_index]);
                for (int j : { 0, 1, 2 })
                {
                    (*q)[time_index](joint_index * 3 + j) = euler_angle(j);
                }
            }
            if (p != nullptr)
            {
                (*p)[time_index].segment<3>(3 * joint_index) = joint->GetAffineRelativeToWorld((*time_sequence)[time_index]).translation();
            }
            ++ joint_index;
        }
    };
#ifdef MULTI_THREAD
    parallelutil::parallel_for(n_samples, lambda);
#else
    parallelutil::parallel_for(n_samples, lambda, 1);
#endif
    
    // Compute Differentiation
    CalculateFirstCentral(dt, (*q), q_dot);
    CalculateSecondCentral(dt, (*q), q_ddot);
}

void Core::SetCurrentState(const std::list<std::shared_ptr<Joint>>& joints,
                           double t,
                           double dt,
                           VecN* q,
                           VecN* q_dot,
                           VecN* q_ddot)
{
    int i = 0;
    for (const auto& joint : joints)
    {
        if (joint->parent_.expired())
        {
            assert(i == 0);
            q->segment(0, 3) = joint->offset_;
        }
        else
        {
            const Eigen::Vector3d euler_angle      = joint->GetEulerAngle(t);
            const Eigen::Vector3d euler_angle_prev = joint->GetEulerAngle(t - dt);
            const Eigen::Vector3d euler_angle_succ = joint->GetEulerAngle(t + dt);
            
            for (int j : { 0, 1, 2 })
            {
                (*q)     (i * 3 + j) = euler_angle(j);
                (*q_dot) (i * 3 + j) = CalculateFirstCentral(euler_angle_prev(j), euler_angle_succ(j), dt);
                (*q_ddot)(i * 3 + j) = CalculateSecondCentral(euler_angle(j), euler_angle_prev(j), euler_angle_succ(j), dt);
            }
        }
        ++ i;
    }
}

void Core::InitializeDynamicsModel()
{
    assert(object_ != nullptr);
    
    const auto joints = object_->GetJoints();
    
    model_ = std::make_shared<RBD::Model>();
    
    // Add a (virtual) root joint
    const Vec3& root_offset = object_->GetRootJoint()->offset_;
    const RBD::Body  root_body  = RBD::Body(0.0, Vec3(0.0, 0.0, 0.0), Vec3(1.0, 1.0, 1.0));
    const RBD::Joint root_joint = RBD::Joint(RBD::JointTypeTranslationXYZ);
    const int root_id = model_->AddBody(0, RBD::Math::Xtrans(root_offset), root_joint, root_body);
    
    std::map<int, int> id_map_from_object_to_model;
    id_map_from_object_to_model[0] = root_id;
    
    int object_joint_id = 0;
    for (auto j : joints)
    {
        // Ignore the root joint because it is already added
        if (j->parent_.expired())
        {
            assert(object_joint_id == 0);
            ++ object_joint_id;
            continue;
        }
        
        const Vec3&  center_of_mass = j->center_of_mass_position_parent_;
        const Mat3&  inertia        = j->inertia_at_center_parent_;
        const double mass           = j->mass_;
        
        const RBD::Body  body  = RBD::Body(mass, center_of_mass, inertia);
        const RBD::Joint joint = RBD::Joint(RBD::JointTypeEulerXYZ);
        
        const Vec3 offset = j->parent_.lock()->offset_;
        
        // Find the parent from the list
        const int parent_object_joint_id = [&]()
        {
            int i = 0;
            for (const auto& tmp : joints)
            {
                if (j->parent_.lock() == tmp) return i;
                else ++ i;
            }
            assert(false);
            return i;
        }();
        
        // Add the new joint to the model
        const int parent_model_joint_id = id_map_from_object_to_model[parent_object_joint_id];
        const int model_joint_id = model_->AddBody(parent_model_joint_id, RBD::Math::Xtrans(offset), joint, body);
        id_map_from_object_to_model[object_joint_id] = model_joint_id;
        
        ++ object_joint_id;
    }
    
    assert(model_->dof_count == joints.size() * 3);
    
    // Define gravity
    model_->gravity = 0.0 * Vec3(0.0, - 9.8, 0.0);
}

Eigen::VectorXd Core::CalculateTorques(double t) const
{
    assert(object_ != nullptr);
    assert(model_  != nullptr);
    
    const double dt = 0.20;
    
    const auto joints = object_->GetJoints();
    
    VecN q      = VecN::Zero(model_->dof_count);
    VecN q_dot  = VecN::Zero(model_->dof_count);
    VecN q_ddot = VecN::Zero(model_->dof_count);
    VecN tau    = VecN::Zero(model_->dof_count);
    
    SetCurrentState(joints, t, dt, &q, &q_dot, &q_ddot);
    
    // Compute inverse dynamics
#ifdef THREAD_SAFE
    RBD::Model copied_model = *model_;
    RBD::InverseDynamics(copied_model, q, q_dot, q_ddot, tau);
#else
    RBD::InverseDynamics(*model_, q, q_dot, q_ddot, tau);
#endif
    
    return tau;
}

double Core::CalculateCost(double min_t, double max_t, double dt)
{
    assert(object_ != nullptr);
    assert(model_  != nullptr);
    
    const auto joints = object_->GetJoints();
    
    std::vector<double> time_sequence;
    std::vector<VecN> q;
    std::vector<VecN> q_dot;
    std::vector<VecN> q_ddot;
    SetStates(joints, min_t, max_t, dt, model_->dof_count, &time_sequence, &q, &q_dot, &q_ddot);

    return CalculateCost(min_t, max_t, dt, time_sequence, q, q_dot, q_ddot);
}

double Core::CalculateCost(double /* min_t */, double /* max_t */, double dt, const std::vector<double>& time_sequence, const std::vector<VecN> &q, const std::vector<VecN> &q_dot, const std::vector<VecN> &q_ddot)
{
    assert(object_ != nullptr);
    assert(model_  != nullptr);
    
    const int n_samples = q.size();
    
    double cost = 0.0;
    
    std::vector<VecN> tau(n_samples, VecN::Zero(model_->dof_count));
    std::vector<double> local_cost(n_samples);
    std::vector<double> local_weight(n_samples);
    
    // Calculate the time-varying weights
    for (int i = 0; i < n_samples; ++ i) local_weight[i] = GetCostWeight(time_sequence[i]);

    // Note: This process is not thread-safe
    auto lambda = [&](int i)
    {
        // Compute inverse dynamics
        // Warning: this part is not thread-safe.
        RBD::InverseDynamics(*model_, q[i], q_dot[i], q_ddot[i], tau[i]);
        
        // Note: Need to ignore the translational force on the root joint
        local_cost[i] = tau[i].segment(3, model_->dof_count - 3).squaredNorm() * dt;

        // Update the cost
        cost += local_weight[i] * local_cost[i];
    };
    
    // Note: It was not very effective to parallelize this for loop
    for (int i = 0; i < n_samples; ++ i) { lambda(i); }
    
    // Set the cache
    optimization_cache_mutex_.lock();
    optimization_cache_ = OptimizationCache{ time_sequence, q, q_dot, q_ddot, tau, local_cost };
    optimization_cache_mutex_.unlock();
    
    return cost;
}

double Core::CalculateFullObjective(double min_t, double max_t, const Eigen::VectorXd &x, const Eigen::VectorXd &x_original)
{
    // Define a constant value
    const double dt = 0.20;
    
    const Eigen::VectorXd weights_parameter = CalculateParameterWeights();
    
    // Check the cache
    // Note: Once made, this cache is expected to be available until the original motion is updated.
    if (original_motion_cache_ == nullptr || !original_motion_cache_->x_original.isApprox(x_original))
    {
        // If the cache is not available, calculate and store it
        object_->SetDescriptor(x_original);
        std::vector<double> time_sequence;
        std::vector<VecN> q;
        std::vector<VecN> q_dot;
        std::vector<VecN> q_ddot;
        std::vector<VecN> p;
        SetStates(object_->GetJoints(), min_t, max_t, dt, model_->dof_count, &time_sequence, &q, &q_dot, &q_ddot, &p);
        original_motion_cache_ = std::make_shared<OriginalMotionCache>(OriginalMotionCache{ p, x_original });
    }
    
    // Set the new parameter to the object
    object_->SetDescriptor(x);

    // Calculate current states
    std::vector<double> time_sequence;
    std::vector<VecN> q;
    std::vector<VecN> q_dot;
    std::vector<VecN> q_ddot;
    std::vector<VecN> p;
    SetStates(object_->GetJoints(), min_t, max_t, dt, model_->dof_count, &time_sequence, &q, &q_dot, &q_ddot, &p);

    // Calculate the motion cost
    const double C = CalculateCost(min_t, max_t, dt, time_sequence, q, q_dot, q_ddot);

    // Regularization in parameter space
    const double D_param = (x - x_original).transpose() * weights_parameter.asDiagonal() * (x - x_original);
   
    auto calculate_difference_of_trajectories = [&]()
    {
        double diff = 0.0;
        for (int i = 0; i < static_cast<int>(p.size()); ++ i)
        {
            diff += weight_trajectory_ * (p[i] - original_motion_cache_->p_original[i]).squaredNorm() * dt;
        }
        return diff;
    };
    
    // Regularization in trajectory space
    const double D_traj = calculate_difference_of_trajectories();
    
    return C + D_param + D_traj;
}

