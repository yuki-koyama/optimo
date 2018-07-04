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

#ifndef ITEM_H
#define ITEM_H

#include <string>
#include <Eigen/Core>

class Variable;

/// \brief A virtual class for representing user-editable items such as joints and IK handles.
class Item
{
public:
    bool is_selected_ = false;
    const std::string& GetName() const { return name_; }
    void SetName(const std::string& name) { name_ = name; }
    virtual std::vector<Variable*> GetVariablePointers() = 0;
    
    /// \param t Time.
    virtual Eigen::Vector3d GetPosition(double t) const = 0;

protected:
    std::string name_;
};

#endif // ITEM_H
