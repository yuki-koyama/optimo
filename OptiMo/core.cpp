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
#include <iostream>
#include <Eigen/Core>

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;

using Vec3 = Vector3d;

Core::Core()
{
    object_ = std::make_shared<Object>();
    
    // Humanoid
    {
        auto joint_root   = object_->GetRootJoint();
        auto joint_spine  = joint_root  ->AddChild(Vec3(  0.0, 0.2, 0.0), Vec3::Zero(), "spine");
        auto joint_neck   = joint_spine ->AddChild(Vec3(  0.0, 0.5, 0.0), Vec3::Zero(), "neck");
        auto joint_head   = joint_neck  ->AddChild(Vec3(  0.0, 0.5, 0.0), Vec3::Zero(), "head");
        auto joint_r_shld = joint_spine ->AddChild(Vec3(- 0.2, 0.5, 0.0), Vec3(0.0, 0.0, + M_PI / 6.0), "right_shoulder");
        auto joint_r_arm  = joint_r_shld->AddChild(Vec3(- 0.5, 0.0, 0.0), Vec3::Zero(), "right_arm");
        auto joint_r_hand = joint_r_arm ->AddChild(Vec3(- 0.5, 0.0, 0.0), Vec3::Zero(), "right_hand");
        auto joint_l_shld = joint_spine ->AddChild(Vec3(+ 0.2, 0.5, 0.0), Vec3(0.0, 0.0, - M_PI / 6.0), "left_shoulder");
        auto joint_l_arm  = joint_l_shld->AddChild(Vec3(+ 0.5, 0.0, 0.0), Vec3(0.0, - 0.1 * M_PI, 0.0), "left_arm");
        auto joint_l_hand = joint_l_arm ->AddChild(Vec3(+ 0.5, 0.0, 0.0), Vector3d::Zero(), "left_hand");
        
        auto joint_l_uleg = joint_root  ->AddChild(Vec3(+ 0.15, - 0.10,   0.00), Vector3d::Zero(), "left_upleg");
        auto joint_l_leg  = joint_l_uleg->AddChild(Vec3(  0.03, - 0.50,   0.05), Vector3d::Zero(), "left_leg");
        auto joint_l_foot = joint_l_leg ->AddChild(Vec3(  0.00, - 0.50, - 0.05), Vector3d::Zero(), "left_foot");
        auto joint_l_toe  = joint_l_foot->AddChild(Vec3(  0.00, - 0.05,   0.15), Vector3d::Zero(), "left_toe");
        
        auto joint_r_uleg = joint_root  ->AddChild(Vec3(- 0.15, - 0.10,   0.00), Vector3d::Zero(), "right_upleg");
        auto joint_r_leg  = joint_r_uleg->AddChild(Vec3(- 0.03, - 0.50,   0.05), Vector3d::Zero(), "right_leg");
        auto joint_r_foot = joint_r_leg ->AddChild(Vec3(  0.00, - 0.50, - 0.05), Vector3d::Zero(), "right_foot");
        auto joint_r_toe  = joint_r_foot->AddChild(Vec3(  0.00, - 0.05,   0.15), Vector3d::Zero(), "right_toe");
        
        joint_spine ->rot_z_.curve_.AddControlPoint( 4, + 0.0 * M_PI);
        joint_spine ->rot_z_.curve_.AddControlPoint(10, + 0.1 * M_PI);
        joint_spine ->rot_z_.curve_.AddControlPoint(28, + 0.1 * M_PI);
        joint_spine ->rot_z_.curve_.AddControlPoint(36, + 0.0 * M_PI);
        joint_r_shld->rot_z_.curve_.AddControlPoint( 4, + M_PI / 4.0);
        joint_r_shld->rot_z_.curve_.AddControlPoint(10, + M_PI / 3.0);
        joint_r_shld->rot_z_.curve_.AddControlPoint(28, + M_PI / 3.0);
        joint_r_shld->rot_z_.curve_.AddControlPoint(36, + M_PI / 4.0);
        joint_l_arm ->rot_z_.curve_.AddControlPoint( 4, + 0.0 * M_PI);
        joint_l_arm ->rot_z_.curve_.AddControlPoint(12, + M_PI / 6.0);
        joint_l_arm ->rot_z_.curve_.AddControlPoint(20, + M_PI / 8.0);
        joint_l_arm ->rot_z_.curve_.AddControlPoint(28, + M_PI / 6.0);
        joint_l_arm ->rot_z_.curve_.AddControlPoint(36, + 0.0 * M_PI);
        joint_l_shld->rot_z_.curve_.AddControlPoint( 4, - M_PI / 4.0);
        joint_l_shld->rot_z_.curve_.AddControlPoint(12, + 0.4 * M_PI);
        joint_l_shld->rot_z_.curve_.AddControlPoint(20, + 0.0 * M_PI);
        joint_l_shld->rot_z_.curve_.AddControlPoint(26, + 0.4 * M_PI);
        joint_l_shld->rot_z_.curve_.AddControlPoint(36, - M_PI / 4.0);
        joint_r_shld->rot_y_.curve_.AddControlPoint( 4, + 0.0 * M_PI);
        joint_r_shld->rot_y_.curve_.AddControlPoint(10, - 0.1 * M_PI);
        joint_r_shld->rot_y_.curve_.AddControlPoint(28, - 0.1 * M_PI);
        joint_r_shld->rot_y_.curve_.AddControlPoint(36, + 0.0 * M_PI);
        joint_r_arm ->rot_y_.curve_.AddControlPoint( 4, + 0.1 * M_PI);
        joint_r_arm ->rot_y_.curve_.AddControlPoint(10, + 0.2 * M_PI);
        joint_r_arm ->rot_y_.curve_.AddControlPoint(28, + 0.2 * M_PI);
        joint_r_arm ->rot_y_.curve_.AddControlPoint(36, + 0.1 * M_PI);
        
        joint_root->offset_ = Vector3d(0.0, - 0.6, 0.0);
    }
    
    // Specify initial weights and freezing DoFs
    const int dim = 5;
    const int n = static_cast<int>(object_->GetDescriptor().rows()) / dim;
    freezed_dofs_ = std::vector<bool>(n * dim, false);
    for (int i = 0; i < n; ++ i)
    {
        freezed_dofs_[i * dim + 0] = true; // t
        freezed_dofs_[i * dim + 1] = true; // v
    }
    per_item_weights_ = std::vector<double>(object_->GetItems().size(), 0.0);
    weight_trajectory_ = 0.0;
    
    // Register the parameters as the original motion
    UpdateOriginalParameters(object_->GetDescriptor());
    
    // Create RBD::Model for Dynamics
    InitializeDynamicsModel();
    
    PushHistory();
    std::cout << "Cost: " << CalculateCost(min_frame_ - 1.0, max_frame_ + 1.0, 0.20) << std::endl;
    
    // Set max cost and max torque for visualization
    UpdateMaximumTorqueAndCost();
}

void Core::UpdateMaximumTorqueAndCost()
{
    const OptimizationCache cache = GetOptimizationCache();
    const int n_samples = cache.GetSize();
    max_torque_ = 0.0;
    max_cost_   = 0.0;
    for (int i = 0; i < n_samples; ++ i)
    {
        max_torque_ = std::max(max_torque_, cache.tau[i].norm());
        max_cost_   = std::max(max_cost_, cache.local_cost_sequence[i]);
    }
}

void Core::UpdateFreezedDofsFromSelection()
{
    const int dim = 5;
    const int n = static_cast<int>(object_->GetDescriptor().rows()) / dim;
    
    // Renew the instance
    freezed_dofs_ = std::vector<bool>(n * dim, true);
    
    // Release the constraints for unselected joints
    auto items = object_->GetItems();
    int control_point_index = 0;
    for (auto item : items)
    {
        int index = control_point_index;
        
        for (Variable* var : item->GetVariablePointers())
        {
            control_point_index += var->GetNumOfControlPoints();
        }
        
        if (item->is_selected_)
        {
            for (int i = index; i < control_point_index; ++ i)
            {
                freezed_dofs_[i * dim + 2] = false;
                freezed_dofs_[i * dim + 3] = false;
                freezed_dofs_[i * dim + 4] = false;
            }
        }
    }
}

void Core::UpdateGlobalWeight(double weight)
{
    weight_trajectory_ = weight;
}

void Core::UpdatePerItemWeight(double weight, const std::string& item_name)
{
    int index = object_->GetItemIndexByName(item_name);
    per_item_weights_[index] = weight;
}

Eigen::VectorXd Core::CalculateParameterWeights() const
{
    const int dim = 5;
    const int n   = object_->GetDescriptor().rows() / dim;
    Eigen::VectorXd weights = Eigen::VectorXd::Constant(dim * n, weight_trajectory_);
    
    auto items = object_->GetItems();
    int item_index = 0;
    int control_point_index = 0;
    for (auto item : items)
    {
        for (Variable* var : item->GetVariablePointers())
        {
            const int n_points = var->GetNumOfControlPoints();
            weights.segment(control_point_index * dim, n_points * dim) += Eigen::VectorXd::Constant(n_points * dim, per_item_weights_[item_index]);
            control_point_index += n_points;
        }
        item_index ++;
    }
    
    return weights;
}

void Core::UpdateOriginalParameters(const VecN &x_original)
{
    x_original_ = x_original;
    original_motion_cache_ = nullptr;
    std::cout << "x_original is updated." << std::endl;
}

double Core::GetCostWeight(double t) const
{
    double weight = 1.0;
    for (const auto& kernel : kernels_)
    {
        weight -= kernel.GetValue(t);
    }
    return std::max(weight, 0.0);
}

