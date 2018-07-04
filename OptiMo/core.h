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

#ifndef core_hpp
#define core_hpp

#include <vector>
#include <memory>
#include <mutex>
#include <three-dim-util/camera.hpp>
#include "object.h"

namespace RigidBodyDynamics
{
    struct Model;
}

/// Gaussian kernel: amp * exp(- (x - mu)^2 / (2 * sigma^2))
struct GaussianKernel
{
    double amp;
    double mu;
    double sigma;
    double GetValue(double x) const { return amp * std::exp(- (x - mu) * (x - mu) / (2.0 * sigma * sigma)); }
};

class Core
{
public:
    Core();
    
    static Core& GetInstance()
    {
        static Core core;
        return core;
    }
    
    int min_frame_         = 1;
    int max_frame_         = 40;
    int current_frame_     = 1;
    int frames_per_second_ = 24;
    
    threedimutil::Camera camera_;
    
    /// The relative scale of 3D icon objects.
    /// This member can be manipulated via user interface.
    double drawing_scale_ = 1.0;
    
    bool is_playing_               = false;
    bool show_cost_visualization_  = true;
    bool show_torques_             = false;
    bool show_original_curves_     = true;
    bool show_time_varying_weight_ = false;
    bool show_process_animation_   = true;
    
    /// If this member is true, any optimization will be forced to stop immediately.
    /// This member is expected to set by another thread than the optimization thread.
    bool stop_optimization_    = false;
    
    /// Main character object.
    std::shared_ptr<Object> object_;
    /// Model for computing Inverse Dynamics.
    std::shared_ptr<RigidBodyDynamics::Model> model_;
    
    /// The number and order is compatible with the descriptor.
    /// True: the dof will not be used for optimization.
    /// False: the dof will be modified via optimization.
    std::vector<bool> freezed_dofs_;
    
    /// Update freezed_dofs_ from the inforamtion stored in Joint::is_selected_
    void UpdateFreezedDofsFromSelection();
    
    /// Weight for per-item regularization
    std::vector<double> per_item_weights_;
    
    void UpdatePerItemWeight(double weight, const std::string& item_name);
    
    /// Weight for trajectory-based regularization
    double weight_trajectory_;
   
    void UpdateGlobalWeight(double weight);
    
    /// \param explicit_termination When this is true, the optimization continues to run until it is explicitly halted.
    void PerformOptimization(bool explicit_termination = false);

    /// Used only for visualization
    Eigen::VectorXd CalculateTorques(double t) const;
    double max_torque_; // Set heuristically
    double max_cost_;   // Set heuristically
    
    /// Calculate the full objective function (i.e., the main cost function + regularization).
    /// \warning This method modifies the main object and does not restore it.
    /// \warning This method is not thread-safe.
    double CalculateFullObjective(double min_t,
                                  double max_t,
                                  const Eigen::VectorXd& x,
                                  const Eigen::VectorXd& x_original);
    
    /// Register the current state to the history.
    void PushHistory();
    /// Discard the current state and restore the last state.
    void PopHistory();
    void ClearHistory();
    
    // IO
    void ReadJson(const std::string& file_path);
    void WriteJson(const std::string& file_path) const;
    void WriteHistory(const std::string& file_path) const;
    void ExportMotion(const std::string& file_path) const;
    
    /// UI widgets will read this cache information for visualizing optimization during optimization
    struct OptimizationCache
    {
        std::vector<double> time_sequence;
        std::vector<Eigen::VectorXd> q;
        std::vector<Eigen::VectorXd> q_dot;
        std::vector<Eigen::VectorXd> q_ddot;
        std::vector<Eigen::VectorXd> tau;
        std::vector<double> local_cost_sequence;
        
        int GetSize() const { return time_sequence.size(); }
    };

    /// If cache is not available, this method newly generates cache.
    /// For thread-safety, this method returns a copy of the cache data, instead of const reference.
    OptimizationCache GetOptimizationCache()
    {
        if (optimization_cache_.time_sequence.size() == 0)
        {
            CalculateCost(min_frame_ - 1.0, max_frame_ + 1.0, 0.20);
        }
        optimization_cache_mutex_.lock();
        OptimizationCache cache_copy = optimization_cache_;
        optimization_cache_mutex_.unlock();
        return cache_copy;
    }
    
    /// This method is thread-safe.
    void ClearOptimizationCache()
    {
        optimization_cache_mutex_.lock();
        optimization_cache_ = OptimizationCache();
        optimization_cache_mutex_.unlock();
    }
    
    using VecN = Eigen::VectorXd;
    
    /// Update the original parameter set, which will be used in the regularization of optimization.
    /// This method is usually called together with the history pop/push methods.
    void UpdateOriginalParameters(const VecN& x_original);
    /// Retrieve the original parameter set, which will be used in the regularization of optimization.
    const VecN& RetrieveOriginalParameters() const { return x_original_; }
    
    /// Add a Gaussian kernel for the time-varying weight function
    void AddKernel(double t) { kernels_.push_back(GaussianKernel{ 1.0, t, 1.0 }); }

    /// \param t Time.
    /// \returns The time-varying weight value.
    double GetCostWeight(double t) const;
    GaussianKernel* GetKernelPointer(int index) { return &kernels_[index]; }
    GaussianKernel* GetLastKernelPointer() { return GetKernelPointer(kernels_.size() - 1); }
    
private:
    
    void UpdateMaximumTorqueAndCost();
    
    std::vector<GaussianKernel> kernels_;
    
    /// This method should be called every time after the object is newly constructed
    void InitializeDynamicsModel();
    
    /// Linear list of previous states in the JSON format
    std::list<std::string> history_;
    
    std::string ConvertCurrentSceneToJson() const;
    void ApplyJsonToScene(const std::string& json_text);
    std::string ConvertSpecificFrameToJson(int t) const;
    
    /// Calculate the configurations over time.
    /// This function is multi-threaded for fast computation.
    /// \param[out] p A pointer for the joint position container.
    /// If p is specified as NULL, the joint positions will not be calculated, which results in fast computation.
    static void SetStates(const std::list<std::shared_ptr<Joint>>& joints,
                          double min_t,
                          double max_t,
                          double dt,
                          int n_dof,
                          std::vector<double>* time_sequence,
                          std::vector<VecN>* q,
                          std::vector<VecN>* q_dot,
                          std::vector<VecN>* q_ddot,
                          std::vector<VecN>* p = nullptr);

    /// Calculate the configuration at the specified time.
    /// This static method is not efficient if iteratively used.
    /// It is not guaranteed that the time differentiations are exactly the same as the ones calculated by SetStates method.
    static void SetCurrentState(const std::list<std::shared_ptr<Joint>>& joints,
                                double t,
                                double dt,
                                VecN* q,
                                VecN* q_dot,
                                VecN* q_ddot);

    /// Wrapper function for the motion cost function.
    /// Using this API should be kept minimum because it does not effectively utilize cache.
    double CalculateCost(double min_t, double max_t, double dt);
    
    /// Calculate the motion cost function using Inverse Dynamics.
    /// This function is not thread-safe.
    /// Intermediate results will be cached for UI.
    double CalculateCost(double min_t,
                         double max_t,
                         double dt,
                         const std::vector<double>& time_sequence,
                         const std::vector<VecN>& q,
                         const std::vector<VecN>& q_dot,
                         const std::vector<VecN>& q_ddot);

    OptimizationCache optimization_cache_;
    std::mutex optimization_cache_mutex_;

    VecN x_original_;
    
    /// Calculate the weights for parameter-based regularization from per_item_weights_
    Eigen::VectorXd CalculateParameterWeights() const;
    
    /// Cache structure for efficiently evaluating the trajectory-based regularization.
    /// This cache will be updated when x_original is changed
    struct OriginalMotionCache
    {
        std::vector<VecN> p_original;
        Eigen::VectorXd x_original;
    };
    
    std::shared_ptr<OriginalMotionCache> original_motion_cache_;
};

#endif /* core_hpp */
