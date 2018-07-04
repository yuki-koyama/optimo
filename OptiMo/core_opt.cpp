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
#include <Eigen/Sparse>
#include <nlopt-util.hpp>

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
typedef Eigen::SparseMatrix<double> SparseMatrixd;

namespace
{
    Core& core = Core::GetInstance();
    
    inline VectorXd ConvertVec2Eigen(const std::vector<double>& x)
    {
        return Eigen::Map<const VectorXd>(&x[0], x.size());
    }
    
    struct Data
    {
        double min_t;
        double max_t;
        VectorXd x_original; // #{full dofs}
        std::function<VectorXd(const VectorXd&)> select; // #{full dofs} => #{selected dofs}
        std::function<VectorXd(const VectorXd&)> revert; // #{selected dofs} => #{full dofs}
    };
    
    // Note: This function modifies the object and does not restore it.
    // Note: This function is compatible with nlopt::vfunc.
    double EvaluateObjective(const std::vector<double>& x, std::vector<double>& /*grad*/, void* data)
    {
        const Data* d = static_cast<Data*>(data);
        const double min_t = d->min_t;
        const double max_t = d->max_t;
        const VectorXd& x_original = d->x_original;
        const VectorXd eigen_x = d->revert(ConvertVec2Eigen(x));

        if (core.stop_optimization_)
        {
            core.stop_optimization_ = false;
            throw nlopt::forced_stop();
        }
        
        return core.CalculateFullObjective(min_t, max_t, eigen_x, x_original);
    }
}

void Core::PerformOptimization(bool explicit_termination)
{
    stop_optimization_ = false;

    // The number of parameters per control handle
    constexpr int dim = 5;
    
    constexpr double epsilon = 1e-06;

    // Retrive x_initial
    // Note: x_initial and x_original can be different.
    VectorXd x_initial = object_->GetDescriptor();
    const int n = static_cast<int>(x_initial.rows() / dim);
    
    // Retrieve x_original
    const VectorXd x_original = this->RetrieveOriginalParameters();
    assert(x_initial.rows() == x_original.rows());
    
    // Set upper & lower bounds
    VectorXd upper = x_initial + VectorXd::Constant(n * dim, + 6.0);
    VectorXd lower = x_initial + VectorXd::Constant(n * dim, - 6.0);
    for (int i = 0; i < n; ++ i)
    {
        // For handles
        upper(i * dim + 2) = + M_PI * 0.4;
        lower(i * dim + 2) = - M_PI * 0.4;
        upper(i * dim + 3) = - epsilon;
        lower(i * dim + 4) = + epsilon;
    }
    
    // Preprocess the initial parameters into the bound space
    for (int i = 0; i < n * dim; ++ i)
    {
        x_initial(i) = std::max(std::min(x_initial(i), upper(i)), lower(i));
    }
    
    // Prepare matrices for handling the freezing of dofs
    int number_of_full_dofs = n * dim;
    int number_of_free_dofs = 0;
    for (auto b : freezed_dofs_)
    {
        if (!b) { ++ number_of_free_dofs; }
    }
    SparseMatrixd X(number_of_free_dofs, number_of_full_dofs);
    SparseMatrixd Y(number_of_full_dofs, number_of_full_dofs);
    int index = 0;
    for (int i = 0; i < static_cast<int>(freezed_dofs_.size()); ++ i)
    {
        if (!freezed_dofs_[i])
        {
            X.insert(index ++, i) = 1.0;
        }
        else
        {
            Y.insert(i, i) = 1.0;
        }
    }
    auto sel = [&X](const VectorXd& x) { return X * x; };
    auto rev = [&X, &Y, &x_initial](const VectorXd& x) { return X.transpose() * x + Y * x_initial; };
    
    // Prepare data
    Data data;
    data.min_t = min_frame_ - 2.0;
    data.max_t = max_frame_ + 2.0;
    data.x_original = x_original;
    data.select = sel;
    data.revert = rev;
    
    // Specify the quality of optimization
    const int max_evaluations = explicit_termination ? 20000 : 2000;
    
    const double relative_function_tolerance  = explicit_termination ? 1e-24 : 1e-07;
    const double relative_parameter_tolerance = explicit_termination ? 1e-24 : 1e-07;

    const double initial_step_scale = 0.06;

    // Perform optimization using nlopt
    const auto result = nloptutil::compute(sel(x_initial),
                                           sel(upper),
                                           sel(lower),
                                           EvaluateObjective,
                                           &data,
                                           nlopt::LN_BOBYQA,
                                           max_evaluations,
                                           relative_function_tolerance,
                                           relative_parameter_tolerance,
                                           false,
                                           true,
                                           initial_step_scale);
    const auto x_optimal = rev(result);
    
    // Apply the result of the optimization
    object_->SetDescriptor(x_optimal);
}
