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

#include "curve.h"
#include <cassert>
#include <cmath>
#include <cfloat>

namespace
{
    // Bernstein basis
    double CubicBasisFunction(double t, int i)
    {
        switch (i) {
            case 0:
                return (1.0 - t) * (1.0 - t) * (1.0 - t);
            case 1:
                return 3.0 * (1.0 - t) * (1.0 - t) * t;
            case 2:
                return 3.0 * (1.0 - t) * t * t;
            case 3:
                return t * t * t;
            default:
                assert(false);
                return 0.0;
        }
    }
    
    double CubicBezier(double t, double p0, double p1, double p2, double p3)
    {
        return p0 * CubicBasisFunction(t, 0) + p1 * CubicBasisFunction(t, 1) + p2 * CubicBasisFunction(t, 2) + p3 * CubicBasisFunction(t, 3);
    }

    // Solve a x^3 + b x^2 + c x + d = 0 for x using Newton's method with the initial guess of x = 0.5
    double SolveCubic(double a, double b, double c, double d, double threshold = 1e-09)
    {
        double x = 0.5;
        for (int i = 0; i < 50; ++ i)
        {
            const double f_x  = x * (x * (a * x + b) + c) + d;
            const double fp_x = x * (3.0 * a * x + 2.0 * b) + c;
            if (std::abs(f_x) < threshold) break;
            x = x - f_x / fp_x;
        }
        return x;
    }
}

double Curve::GetValue(double t, const Segment& segment) const
{
    // Handle corner cases
    if (!segment.HasStart() && !segment.HasEnd())
    {
        return 0.0;
    }
    else if (!segment.HasStart())
    {
        return control_points_[segment.end_point_index].GetV();
    }
    else if (!segment.HasEnd())
    {
        return control_points_[segment.start_point_index].GetV();
    }

    const auto& s_point = control_points_[segment.start_point_index];
    const auto& e_point = control_points_[segment.end_point_index];
    const double s_t = s_point.GetT();
    const double e_t = e_point.GetT();
    const double s_v = s_point.GetV();
    const double e_v = e_point.GetV();

    const double p_0 = s_t;
    const double p_1 = s_t + s_point.time_forward;
    const double p_2 = e_t + e_point.time_backward;
    const double p_3 = e_t;
    
    const double a = - p_0 + 3.0 * p_1 - 3.0 * p_2 + p_3;
    const double b = 3.0 * (p_0 - 2.0 * p_1 + p_2);
    const double c = - 3.0 * p_0 + 3.0 * p_1;
    const double d = p_0 - t;
    
    const double t_opt = SolveCubic(a, b, c, d);
    
    return CubicBezier(t_opt, s_v, s_v + s_point.GetValueForward(), e_v + e_point.GetValueBackward(), e_v);
}

Curve::Curve()
{
    segments_.push_back(Segment{ -1, -1 });
}

std::list<Curve::Segment>::const_iterator Curve::FindSegmentIterator(double t) const
{
    assert(segments_.size() > 0);
    for (auto it = segments_.cbegin(); it != segments_.cend(); ++ it)
    {
        const auto& seg = *it;
        if (!seg.HasEnd())
        {
            return it;
        }
        if (!seg.HasStart())
        {
            if (control_points_[seg.end_point_index].GetT() > t)
            {
                return it;
            }
            else
            {
                continue;
            }
        }
        if (control_points_[seg.start_point_index].GetT() <= t && control_points_[seg.end_point_index].GetT() > t)
        {
            return it;
        }
    }
    assert(false);
    return segments_.end();
}

void Curve::AddControlPoint(const ControlPoint &control_point)
{
    // Register the new control point
    control_points_.push_back(control_point);
    
    // Find the existing segment that will be sprit
    auto segment_iterator = FindSegmentIterator(control_point.GetT());
    
    // Sprit the existing segment into two
    const Segment& seg = *segment_iterator;
    const int new_point_index = static_cast<int>(control_points_.size()) - 1;
    const Segment new_seg_behind{ seg.start_point_index, new_point_index };
    const Segment new_seg_ahead{ new_point_index, seg.end_point_index };

    // Remove the original segment and insert new ones
    auto iter = segments_.erase(segment_iterator);
    auto iter2 = segments_.insert(iter, new_seg_ahead);
    segments_.insert(iter2, new_seg_behind);
}

double Curve::GetValue(double t) const
{
    // Find the relevant segment
    const Segment& seg = FindSegment(t);
    return GetValue(t, seg);
}

double Curve::GetMinimumKeyValue() const
{
    assert(!GetControlPoints().empty());
    double val = + DBL_MAX;
    for (const auto& point : GetControlPoints())
    {
        val = std::min(val, point.GetV());
    }
    return val;
}

double Curve::GetMaximumKeyValue() const
{
    assert(!GetControlPoints().empty());
    double val = - DBL_MAX;
    for (const auto& point : GetControlPoints())
    {
        val = std::max(val, point.GetV());
    }
    return val;
}

void Curve::MakeMotionCyclic()
{
    const int n = control_points_.size();
    const double theta_first = GetControlPoint(0    ).theta;
    const double theta_last  = GetControlPoint(n - 1).theta;
    const double theta_new   = 0.5 * (theta_first + theta_last);
    
    GetControlPoint(0    ).theta = theta_new;
    GetControlPoint(n - 1).theta = theta_new;
}

