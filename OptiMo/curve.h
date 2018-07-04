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

#ifndef CURVE_H
#define CURVE_H

#include <vector>
#include <list>
#include <memory>
#include <cassert>
#include <Eigen/Core>

struct ControlPoint
{
    double time;
    double value;
    double theta;
    double time_backward;
    double time_forward;
    
    double GetValueBackward() const
    {
        return time_backward * std::tan(theta);
    }
    
    double GetValueForward() const
    {
        return time_forward * std::tan(theta);
    }
    
    Eigen::VectorXd GetDescriptor() const
    {
        Eigen::VectorXd x(5);
        x << time, value, theta, time_backward, time_forward;
        return x;
    }
    
    void SetDescriptor(const Eigen::VectorXd& x)
    {
        assert (x.rows() == 5);
        time  = x(0);
        value = x(1);
        theta = x(2);
        time_backward = x(3);
        time_forward  = x(4);
    }

    ControlPoint(double t, double v)
    {
        time  = t;
        value = v;
        theta = 0.0;
        time_backward = - 1.0;
        time_forward  = + 1.0;
    }
    
    ControlPoint(double t, double v, double theta, double t_b, double t_f)
    {
        this->time  = t;
        this->value = v;
        this->theta = theta;
        this->time_backward = t_b;
        this->time_forward  = t_f;
    }
    
    double GetT() const { return time; }
    double GetV() const { return value; }
};

class Curve
{
public:
    Curve();

    /// \brief Structure for managing a single cubic Bezier curve segment
    struct Segment
    {
    public:
        int start_point_index;
        int end_point_index;
        
        bool HasStart() const { return start_point_index >= 0; }
        bool HasEnd() const { return end_point_index >= 0; }
    };

private:
    /// No guarantee that control points are sorted by time or referred by at least one segment
    std::vector<ControlPoint> control_points_;
    
    /// Always sorted by the time (former is earlier)
    std::list<Segment> segments_;
    
    std::list<Segment>::const_iterator FindSegmentIterator(double t) const;
    const Segment& FindSegment(double t) const { return *FindSegmentIterator(t); }

    double GetValue(double t, const Segment& segment) const;

public:
    
    void MakeMotionCyclic();
    
    double GetMaximumKeyValue() const;
    double GetMinimumKeyValue() const;

    double GetValue(double t) const;
    double GetValue(int t) const { return GetValue(static_cast<double>(t)); }

    void AddControlPoint(const ControlPoint& control_point);
    void AddControlPoint(double t, double v) { AddControlPoint(ControlPoint(t, v)); }

    const std::vector<ControlPoint>& GetControlPoints() const { return control_points_; }
    std::vector<ControlPoint>& GetControlPoints() { return control_points_; }
    const std::list<Segment>& GetCurveSegments() const { return segments_; }
    ControlPoint& GetControlPoint(int i) { assert(i >= 0 && i < static_cast<int>(control_points_.size())); return control_points_[i]; }
};

#endif // CURVE_H
