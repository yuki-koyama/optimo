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

#ifndef OBJECT_H
#define OBJECT_H

#include <string>
#include <memory>
#include <Eigen/Geometry>
#include "joint.h"
#include "ikhandler.h"

/// \brief Main class for managing a skeletal character that consists of linked rigid bodies
class Object
{
public:
    Object();
    
    /// The order does not have a specific meaning but it is ensured to be consistent.
    std::list<std::shared_ptr<Joint>> GetJoints() const;
    /// The order does not have a specific meaning but it is ensured to be consistent.
    std::list<std::shared_ptr<IkHandler>> GetIkHandlers() const;
    /// The order does not have a specific meaning but it is ensured to be consistent.
    std::list<std::shared_ptr<Item>> GetItems() const;
    
    void AddIkHandler(std::shared_ptr<IkHandler> ik_handler);

    /// \returns Null pointer if there is no such item
    std::shared_ptr<Item> GetItemByName(const std::string& name) const;
    /// \returns Null pointer if there is no such joint
    std::shared_ptr<Joint> GetJointByName(const std::string& name) const;
    /// \returns a negative number (- 1) if there is no such item
    int GetItemIndexByName(const std::string& name) const;
    
    Eigen::VectorXd GetDescriptor() const;
    void SetDescriptor(const Eigen::VectorXd& x);
    
    void ClearIkHandlers() { ik_handlers_.clear(); }
    
    /// \param t Time
    void Draw(int t) const;

    std::shared_ptr<Joint> GetRootJoint() const { return root_joint_; }
    void SetRootJoint(std::shared_ptr<Joint> root_joint) { root_joint_ = root_joint; }
    
    /// If this is true, the resulting motion will be (pseudo-)cyclic.
    /// (i.e., the first and last control handles share the tangent)
    bool is_cyclic_ = false;
    
private:
    /// Root joint of the kinematic tree
    std::shared_ptr<Joint> root_joint_;
    /// List of all the IK handles defined in this object
    std::list<std::shared_ptr<IkHandler>> ik_handlers_;
    
    /// Once this method is called, the control handles are manipulated so that the motion becomes (pseudo)cyclic.
    void MakeMotionCyclic();
};

#endif // OBJECT_H
