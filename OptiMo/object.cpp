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

#include "object.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

Object::Object()
{
    root_joint_ = std::make_shared<Joint>();
}

void Object::AddIkHandler(std::shared_ptr<IkHandler> ik_handler)
{
    ik_handlers_.push_back(ik_handler);
}

std::list<std::shared_ptr<Joint>> Object::GetJoints() const
{
    std::function<std::list<std::shared_ptr<Joint>>(std::shared_ptr<Joint>)> lambda = [&](std::shared_ptr<Joint> joint) -> std::list<std::shared_ptr<Joint>>
    {
        std::list<std::shared_ptr<Joint>> joint_list;
        joint_list.push_back(joint);
        for (auto child : joint->children_)
        {
            joint_list.splice(joint_list.end(), lambda(child));
        }
        return joint_list;
    };
    return lambda(root_joint_);
}

std::list<std::shared_ptr<IkHandler>> Object::GetIkHandlers() const
{
    return ik_handlers_;
}

std::list<std::shared_ptr<Item>> Object::GetItems() const
{
    std::list<std::shared_ptr<Item>> item_list;
    
    for (auto joint : GetJoints())
    {
        item_list.push_back(joint);
    }
    for (auto ik_handler : ik_handlers_)
    {
        item_list.push_back(ik_handler);
    }

    return item_list;
}

std::shared_ptr<Item> Object::GetItemByName(const std::string& name) const
{
    auto ptr = GetJointByName(name);
    if (ptr != nullptr) return ptr;

    for (auto ik_handler : ik_handlers_)
    {
        if (ik_handler->GetName() == name) return ik_handler;
    }
    
    return nullptr;
}

std::shared_ptr<Joint> Object::GetJointByName(const std::string& name) const
{
    std::function<std::shared_ptr<Joint>(std::shared_ptr<Joint>)> lambda = [&](std::shared_ptr<Joint> joint) -> std::shared_ptr<Joint>
    {
        if (joint->GetName() == name) return joint;
        
        for (auto child : joint->children_)
        {
            auto joint_ptr = lambda(child);
            if (joint_ptr != nullptr) return joint_ptr;
        }
        
        return nullptr;
    };
    return lambda(root_joint_);
}

int Object::GetItemIndexByName(const std::string &name) const
{
    auto items = GetItems();
    
    int index = 0;
    for (auto item : items)
    {
        if (item->GetName() == name)
        {
            return index;
        }
        index ++;
    }
    assert(false);
    return - 1;
}

VectorXd Object::GetDescriptor() const
{
    const auto items = GetItems();
    const int dim = 5;
    int n = 0;
    for (auto item : items)
    {
        for (Variable* var : item->GetVariablePointers())
        {
            if (var->IsKeyframed()) n += var->curve_.GetControlPoints().size();
        }
    }
    
    VectorXd x(n * dim);
    
    int index = 0;
    for (auto item : items)
    {
        for (Variable* var : item->GetVariablePointers())
        {
            if (!var->IsKeyframed()) continue;
            
            Curve& curve = var->curve_;
            const int m = static_cast<int>(curve.GetControlPoints().size());
            
            for (int p = 0; p < m; ++ p)
            {
                const VectorXd x_p = curve.GetControlPoint(p).GetDescriptor();
                x.segment(index, x_p.rows()) = x_p;
                index += x_p.rows();
            }        
        }
    }
    
    return x;
}

void Object::SetDescriptor(const VectorXd& x)
{
    const auto items = GetItems();
    constexpr int dim = 5;
    int index = 0;

    for (auto item : items)
    {
        for (Variable* var : item->GetVariablePointers())
        {
            if (!var->IsKeyframed()) continue;

            Curve& curve = var->curve_;
            const int m = static_cast<int>(curve.GetControlPoints().size());
            
            for (int p = 0; p < m; ++ p)
            {
                const VectorXd x_p = x.segment(index, dim);
                curve.GetControlPoint(p).SetDescriptor(x_p);
                index += dim;
            }
        }
    }
    
    if (is_cyclic_) MakeMotionCyclic();
}

void Object::MakeMotionCyclic()
{
    const auto items = GetItems();
    for (auto item : items)
    {
        for (Variable* var : item->GetVariablePointers())
        {
            if (!var->IsKeyframed()) continue;
            
            var->curve_.MakeMotionCyclic();
        }
    }
}

void Object::Draw(int t) const
{
    assert(root_joint_ != nullptr);
    
    // Draw the body
    root_joint_->Draw(t);
    
    // Draw the IK handles
    for (const auto& ik_handle : ik_handlers_)
    {
        if (ik_handle->is_selected_) ik_handle->Draw(t);
    }
}
