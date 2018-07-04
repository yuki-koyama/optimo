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
#include <fstream>
#include <iostream>
#include <json11.hpp>

using json11::Json;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace
{
    
    std::vector<Json> InterpretVectorXd2Json(const VectorXd& x)
    {
        std::vector<Json> tmp;
        for (int i = 0; i < static_cast<int>(x.rows()); ++ i)
        {
            tmp.push_back(Json(x(i)));
        }
        return tmp;
    }
    
    std::vector<Json> InterpretVector3d2Json(const Vector3d& v)
    {
        return std::vector<Json>{ v(0), v(1), v(2) };
    }
    
    std::vector<Json> InterpretCurve2Json(const Curve& curve)
    {
        std::vector<Json> points;
        for (const auto& point : curve.GetControlPoints())
        {
            points.push_back(InterpretVectorXd2Json(point.GetDescriptor()));
        }
        return points;
    }
    
    Json InterpretVariable2Json(const Variable& var)
    {
        return Json::object
        {
            { "var", Json(var.var_) },
            { "curve", InterpretCurve2Json(var.curve_) }
        };
    }
    
    Json InterpretJoint2Json(std::shared_ptr<Joint> joint)
    {
        std::vector<Json> children;
        for (auto child : joint->children_)
        {
            children.push_back(InterpretJoint2Json(child));
        }
        
        return Json::object
        {
            { "name",  Json(joint->GetName()) },
            { "offset", InterpretVector3d2Json(joint->offset_) },
            { "rot_x", InterpretVariable2Json(joint->rot_x_) },
            { "rot_y", InterpretVariable2Json(joint->rot_y_) },
            { "rot_z", InterpretVariable2Json(joint->rot_z_) },
            { "children", children }
        };
    }
    
    Json InterpretIkHandler2Json(std::shared_ptr<IkHandler> ik_handler)
    {
        return Json::object
        {
            { "name", Json(ik_handler->GetName()) },
            { "pos_x", InterpretVariable2Json(*ik_handler->GetVariablePointers()[0]) },
            { "pos_y", InterpretVariable2Json(*ik_handler->GetVariablePointers()[1]) },
            { "pos_z", InterpretVariable2Json(*ik_handler->GetVariablePointers()[2]) },
            { "root_name", Json(ik_handler->GetRootName()) },
            { "end_effector_name", Json(ik_handler->GetEndEffectorName()) }
        };
    }
    
    std::vector<Json> InterpretIkHandlers2Json(std::list<std::shared_ptr<IkHandler>> ik_handlers)
    {
        std::vector<Json> jsons;
        for (auto ik_handler : ik_handlers)
        {
            jsons.push_back(InterpretIkHandler2Json(ik_handler));
        }
        return jsons;
    }
    
    Variable InterpretJson2Variable(const Json& json)
    {
        Variable var(json["var"].number_value());
        
        for (const auto& j : json["curve"].array_items())
        {
            var.curve_.AddControlPoint(ControlPoint(j[0].number_value(), j[1].number_value(), j[2].number_value(), j[3].number_value(), j[4].number_value()));
        }
        
        return var;
    }
    
    std::shared_ptr<Joint> InterpretJson2Joint(const Json& json, std::shared_ptr<Joint> parent = nullptr)
    {
        const std::string name = json["name"].string_value();
        
        const Vector3d offset = Vector3d(json["offset"][0].number_value(), json["offset"][1].number_value(), json["offset"][2].number_value());
        
        const Variable rot_x = InterpretJson2Variable(json["rot_x"]);
        const Variable rot_y = InterpretJson2Variable(json["rot_y"]);
        const Variable rot_z = InterpretJson2Variable(json["rot_z"]);
        
        std::shared_ptr<Joint> joint = std::make_shared<Joint>(parent, offset, rot_x, rot_y, rot_z, name);
        
        for (const auto& j : json["children"].array_items())
        {
            joint->AddChild(InterpretJson2Joint(j, joint));
        }
        
        return joint;
    }
    
    void InterpretJson2IkHandlerAndAddToObject(const Json& json, std::shared_ptr<Object> object)
    {
        const Variable pos_x = InterpretJson2Variable(json["pos_x"]);
        const Variable pos_y = InterpretJson2Variable(json["pos_y"]);
        const Variable pos_z = InterpretJson2Variable(json["pos_z"]);
        
        const std::string name = json["name"].string_value();
        
        const std::string root_name         = json["root_name"].string_value();
        const std::string end_effector_name = json["end_effector_name"].string_value();
        
        std::shared_ptr<IkHandler> ik_handler = std::make_shared<IkHandler>(name);
        ik_handler->Initialize(object->GetJointByName(root_name), object->GetJointByName(end_effector_name));
        
        *(ik_handler->GetVariablePointers()[0]) = pos_x;
        *(ik_handler->GetVariablePointers()[1]) = pos_y;
        *(ik_handler->GetVariablePointers()[2]) = pos_z;
        
        object->AddIkHandler(ik_handler);
    }
}

void Core::ReadJson(const std::string &file_path)
{
    std::ifstream reading_file(file_path);
    const std::string json_text = std::string(std::istreambuf_iterator<char>(reading_file), std::istreambuf_iterator<char>());;
    
    ApplyJsonToScene(json_text);
    
    InitializeDynamicsModel();
    UpdateMaximumTorqueAndCost();
}

void Core::WriteJson(const std::string &file_path) const
{
    std::ofstream writing_file(file_path);
    writing_file << ConvertCurrentSceneToJson();
}

void Core::WriteHistory(const std::string &file_path) const
{
    std::ofstream writing_file(file_path);
    writing_file << "[";
    
    bool first = true;
    for (const std::string& json_text : history_)
    {
        if (first) first = false;
        else writing_file << ",";
        
        writing_file << json_text;
    }
    writing_file << "]";
}

void Core::PushHistory()
{
    std::cout << "PushHistory()" << std::endl;
    history_.push_back(ConvertCurrentSceneToJson());
}

void Core::PopHistory()
{
    std::cout << "PopHistory()" << std::endl;
    
    if (history_.size() <= 1)
    {
        std::cerr << "Cannot undo any more." << std::endl;
        return;
    }
    
    history_.pop_back();
    const std::string& last_stamp = history_.back();
    
    ApplyJsonToScene(last_stamp);
}

void Core::ClearHistory()
{
    history_.clear();
}

std::string Core::ConvertCurrentSceneToJson() const
{
    const Json obj = Json::object
    {
        { "min_frame", Json(min_frame_) },
        { "max_frame", Json(max_frame_) },
        { "frames_per_second", Json(frames_per_second_) },
        { "joints", InterpretJoint2Json(object_->GetRootJoint()) },
        { "ik_handlers", InterpretIkHandlers2Json(object_->GetIkHandlers()) }
    };
    
    return obj.dump();
}

void Core::ApplyJsonToScene(const std::string& json_text)
{
    // Reset the object
    this->object_->SetRootJoint(nullptr);
    this->object_->ClearIkHandlers();
    
    // Parse json text
    std::string err;
    const auto json = Json::parse(json_text, err);
    
    this->min_frame_ = json["min_frame"].int_value();
    this->max_frame_ = json["max_frame"].int_value();
    this->frames_per_second_ = json["frames_per_second"].int_value();
    this->object_->SetRootJoint(InterpretJson2Joint(json["joints"]));
    
    for (auto ik_handler_json : json["ik_handlers"].array_items())
    {
        InterpretJson2IkHandlerAndAddToObject(ik_handler_json, this->object_);
    }
    
    // Clear the optimization cache
    this->ClearOptimizationCache();
}

void Core::ExportMotion(const std::string &file_path) const
{
    std::ofstream writing_file(file_path);
    writing_file << "[";
    
    for (int t = min_frame_; t <= max_frame_; ++ t)
    {
        if (t != min_frame_) writing_file << ", ";
        
        writing_file << ConvertSpecificFrameToJson(t);
    }
    writing_file << "]";
}

std::string Core::ConvertSpecificFrameToJson(int t) const
{
    std::vector<Json> joints_json;
    
    auto joints = object_->GetJoints();
    for (auto joint : joints)
    {
        const std::string name = joint->GetName();
        const Vector3d angle = joint->GetEulerAngle(t);
        
        const Json joint_json = Json::object
        {
            { "name", Json(name) },
            { "r_x", Json(angle(0)) },
            { "r_y", Json(angle(1)) },
            { "r_z", Json(angle(2)) }
        };
        
        joints_json.push_back(joint_json);
    }
    
    const Json frame_json = Json::object
    {
        { "frame", Json(t) },
        { "joints", joints_json }
    };
    
    return frame_json.dump();
}

