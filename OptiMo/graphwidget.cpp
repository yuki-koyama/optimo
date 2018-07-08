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

#include "graphwidget.h"
#include <random>
#include <iostream>
#include <cfloat>
#include <QImage>
#include <QMouseEvent>
#include <GLKit/GLKMatrix4.h>
#include <Eigen/Core>
#include <tinycolormap.hpp>
#include <three-dim-util/gl-wrapper.hpp>
#include <three-dim-util/glut-wrapper.hpp>
#include "core.h"
#include "mainwindow.h"
#include "mainwidget.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using threedimutil::glColor;
using threedimutil::glVertex;
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

#define USE_CACHE_FOR_COST_VISUALIZATION

namespace
{
    Core& core = Core::GetInstance();
    
    inline void SetOrthMatrix(double min_x, double max_x, double min_y, double max_y)
    {
        glLoadMatrixf(GLKMatrix4MakeOrtho(min_x, max_x, min_y, max_y, - 1.0f, + 1.0f).m);
    }
    
    inline QColor ConvertColorFormat(const tinycolormap::Color& color)
    {
        return QColor(255.0 * color.r(), 255.0 * color.g(), 255.0 * color.b());
    }
    
    std::list<Curve*> GetSelectedCurveList(std::shared_ptr<Object> object)
    {
        std::list<Curve*> curves;
        for (auto item : object->GetItems())
        {
            if (!item->is_selected_) continue;
            
            auto vars = item->GetVariablePointers();
            if (vars[0]->IsKeyframed()) curves.push_back(&vars[0]->curve_);
            if (vars[1]->IsKeyframed()) curves.push_back(&vars[1]->curve_);
            if (vars[2]->IsKeyframed()) curves.push_back(&vars[2]->curve_);
        }
        return curves;
    }
    
    void DrawHandle(const ControlPoint& point,
                    const Vec3& point_color,
                    const Vec3& handle_color,
                    double handle_width,
                    double handle_size,
                    double control_point_size,
                    bool is_first,
                    bool is_last)
    {
        const double t = point.GetT();
        const double v = point.GetV();
        
        // Note: the backward handle of the first point and the forward handle of the last points are not drawn
        glColor(handle_color);
        glLineWidth(handle_width);
        glPointSize(handle_size);
        glBegin(GL_LINE_STRIP);
        if (!is_first) glVertex2d(t + point.time_backward, v + point.GetValueBackward());
        glVertex2d(t, v);
        if (!is_last)  glVertex2d(t + point.time_forward, v + point.GetValueForward());
        glEnd();
        glBegin(GL_POINTS);
        if (!is_first) glVertex2d(t + point.time_backward, v + point.GetValueBackward());
        if (!is_last)  glVertex2d(t + point.time_forward, v + point.GetValueForward());
        glEnd();
        
        glColor(point_color);
        glPointSize(control_point_size);
        glBegin(GL_POINTS);
        glVertex2d(t, v);
        glEnd();
    }
    
    /// \param image QImage that is formatted using QImage::Format_RGBA8888
    void DrawQImages(const QImage& image, double min_x, double max_x, double min_y, double max_y)
    {
        glEnable(GL_TEXTURE_2D);
        GLuint texture_id;
        glGenTextures(1, &texture_id);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width(), image.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, image.bits());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glBegin(GL_QUADS);
        glTexCoord2d(0.0, 0.0); glVertex2d(min_x, min_y);
        glTexCoord2d(1.0, 0.0); glVertex2d(max_x, min_y);
        glTexCoord2d(1.0, 1.0); glVertex2d(max_x, max_y);
        glTexCoord2d(0.0, 1.0); glVertex2d(min_x, max_y);
        glEnd();
        glDisable(GL_TEXTURE_2D);
        glDeleteTextures(1, &texture_id);
    }
}

GraphWidget::GraphWidget(QWidget *parent) : QOpenGLWidget(parent)
{
    UpdateValueRange();
    
    // Set the number of samplings for anti-aliasing
    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
    format.setSamples(4);
    this->setFormat(format);
}

void GraphWidget::initializeGL()
{
    glClearColor(color_.background(0), color_.background(1), color_.background(2), 1.0f);
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
}

void GraphWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

///////////////////////////////////////////////////////////////////////////

void GraphWidget::mousePressEvent(QMouseEvent *event)
{
    current_mouse_pos_x_ = event->x();
    current_mouse_pos_y_ = event->y();
    
    const Vec2 p_cursor(current_mouse_pos_x_, current_mouse_pos_y_);
    
    std::shared_ptr<DragTarget> drag_target = nullptr;
    double min_dist = 0.0;
    
    auto curves = GetSelectedCurveList(core.object_);
    for (Curve* curve : curves)
    {
        auto lambda = [&](ControlPoint* point, const Vec2& p, TargetType type)
        {
            const double dist = (p_cursor - p).norm();
            if (drag_target == nullptr || dist < min_dist)
            {
                drag_target = std::make_shared<DragTarget>(DragTarget{ curve, point, type });
                min_dist = dist;
            }
        };
        
        for (ControlPoint& point : curve->GetControlPoints())
        {
            if (use_per_curve_value_range_)
            {
                const Vec2 p_center(ConvertTimeToX(point.GetT()), ConvertValueToY(point.GetV(), curve));
                lambda(&point, p_center, TargetType::CENTER);
                
                const Vec2 p_backward(ConvertTimeToX(point.GetT() + point.time_backward), ConvertValueToY(point.GetV() + point.GetValueBackward(), curve));
                lambda(&point, p_backward, TargetType::BACKWARD);
                
                const Vec2 p_forward(ConvertTimeToX(point.GetT() + point.time_forward), ConvertValueToY(point.GetV() + point.GetValueForward(), curve));
                lambda(&point, p_forward, TargetType::FORWARD);
            }
            else
            {
                const Vec2 p_center(ConvertTimeToX(point.GetT()), ConvertValueToY(point.GetV()));
                lambda(&point, p_center, TargetType::CENTER);
                
                const Vec2 p_backward(ConvertTimeToX(point.GetT() + point.time_backward), ConvertValueToY(point.GetV() + point.GetValueBackward()));
                lambda(&point, p_backward, TargetType::BACKWARD);
                
                const Vec2 p_forward(ConvertTimeToX(point.GetT() + point.time_forward), ConvertValueToY(point.GetV() + point.GetValueForward()));
                lambda(&point, p_forward, TargetType::FORWARD);
            }
        }
    }
    
    // Threshold distance in screen space
    const double threshold = 12.0;
    
    if (min_dist < threshold)
    {
        this->drag_target_ = drag_target;
    }
    
    update();
}

void GraphWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (drag_target_ == nullptr) return;
    
    const int diff_x = event->x() - current_mouse_pos_x_;
    const int diff_y = event->y() - current_mouse_pos_y_;
    
    const Vec2 diff(ScaleXToTime(diff_x), use_per_curve_value_range_ ? ScaleYToValue(diff_y, drag_target_->curve_ptr) : ScaleYToValue(diff_y));
    const Vec2 backward = Vec2(drag_target_->control_point_ptr->time_backward, drag_target_->control_point_ptr->GetValueBackward());
    const Vec2 forward  = Vec2(drag_target_->control_point_ptr->time_forward, drag_target_->control_point_ptr->GetValueForward());
    
    switch (drag_target_->target_type)
    {
        case TargetType::CENTER:
        {
            drag_target_->control_point_ptr->time  += diff(0);
            drag_target_->control_point_ptr->value += diff(1);
            break;
        }
        case TargetType::BACKWARD:
        {
            const Vec2   backward_new = backward + diff;
            const double theta_new    = std::atan(backward_new(1) / backward_new(0));
            const double d            = forward.norm();
            const Vec2   forward_new  = Vec2(d * std::cos(theta_new), d * std::sin(theta_new));
            
            drag_target_->control_point_ptr->time_backward = backward_new(0);
            drag_target_->control_point_ptr->time_forward  = forward_new(0);
            drag_target_->control_point_ptr->theta         = theta_new;
            break;
        }
        case TargetType::FORWARD:
        {
            const Vec2   forward_new  = forward + diff;
            const double theta_new    = std::atan(forward_new(1) / forward_new(0));
            const double d            = backward.norm();
            const Vec2   backward_new = - Vec2(d * std::cos(theta_new), d * std::sin(theta_new));
            
            drag_target_->control_point_ptr->time_backward = backward_new(0);
            drag_target_->control_point_ptr->time_forward  = forward_new(0);
            drag_target_->control_point_ptr->theta         = theta_new;
            break;
        }
    }
    
    current_mouse_pos_x_ = event->x();
    current_mouse_pos_y_ = event->y();
    
    core.ClearOptimizationCache();
    
    main_window_->main_widget_->update();
    update();
}

void GraphWidget::mouseReleaseEvent(QMouseEvent* /* event */)
{
    if (drag_target_ == nullptr) return;
    
    drag_target_ = nullptr;
    
    core.PushHistory();
    core.UpdateOriginalParameters(core.object_->GetDescriptor());
    
    update();
}

///////////////////////////////////////////////////////////////////////////

void GraphWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);
    
    const double device_pixel_ratio = this->devicePixelRatioF();
    
    // Warning: there is a limit of maximum line width
    const double grid_line_width     =  2.0 * device_pixel_ratio;
    const double curve_width         =  3.0 * device_pixel_ratio;
    const double current_frame_width =  3.0 * device_pixel_ratio;
    const double handle_width        =  5.0 * device_pixel_ratio;
    const double handle_size         = 10.0 * device_pixel_ratio;
    const double control_point_size  = 12.0 * device_pixel_ratio;
    
    // Set projection matrix
    glMatrixMode(GL_PROJECTION);
    SetOrthMatrix(min_t_, max_t_, min_v_, max_v_);
    
    if (core.show_time_varying_weight_)
    {
        // Visualize time-varying weight
        const int width = 1000;
        auto weight_texture = std::make_shared<QImage>(width, 1, QImage::Format_RGBA8888);
        for (int i = 0; i < width; ++ i)
        {
            double x = ((static_cast<double>(i) + 0.5) / static_cast<double>(width)) * (max_t_ - min_t_) + min_t_;
            
            const double weight = core.GetCostWeight(x);
            const auto   color  = tinycolormap::GetGrayColor(1.0 - weight);
            weight_texture->setPixelColor(i, 0, ConvertColorFormat(color));
        }
        
        // Draw the image as GL texture
        DrawQImages(*weight_texture, min_t_, max_t_, min_v_, max_v_);
    }
    else if (core.show_cost_visualization_)
    {
        // Visualize cost values
#ifdef USE_CACHE_FOR_COST_VISUALIZATION
        const Core::OptimizationCache cache = core.GetOptimizationCache();
        const int width = cache.GetSize();
        auto cost_texture = std::make_shared<QImage>(width, 1, QImage::Format_RGBA8888);
        for (int i = 0; i < width; ++ i)
        {
            const double cost  = cache.local_cost_sequence[i];
            const auto   color = tinycolormap::GetHotColor(1.0 - std::sqrt(cost / core.max_cost_));
            cost_texture->setPixelColor(i, 0, ConvertColorFormat(color));
        }
#else
        const int width = 1000;
        auto cost_texture = std::make_shared<QImage>(width, 1, QImage::Format_RGBA8888);
        for (int i = 0; i < width; ++ i)
        {
            double x = ((static_cast<double>(i) + 0.5) / static_cast<double>(width)) * (max_t_ - min_t_) + min_t_;
            
            const double cost  = core.CalculateTorques(x).squaredNorm();
            const auto   color = tinycolormap::GetHotColor(1.0 - std::sqrt(cost / core.max_cost_));
            cost_texture->setPixelColor(i, 0, ConvertColorFormat(color));
        }
#endif
        
        // Calculate adequate stretches
        const double n = cost_texture->width();
        const double min_t = cache.time_sequence[0];
        const double max_t = cache.time_sequence[n - 1];
        const double min_u = 1.0 / (2.0 * n);
        const double max_u = 1.0 - 1.0 / (2.0 * n);
        auto u2t = [&](double u) { return (u - min_u) * (max_t - min_t) / (max_u - min_u) + min_t; };
        
        // Draw the image as GL texture
        DrawQImages(*cost_texture, u2t(0.0), u2t(1.0), min_v_, max_v_);
    }
    
    // Draw vertical lines
    glColor(color_.grid_line);
    for (int i = core.min_frame_; i <= core.max_frame_; ++ i)
    {
        const double x      = static_cast<double>(i);
        const double y      = 0.5 * (max_v_ + min_v_);
        const double size_x = (grid_line_width / width()) * (max_t_ - min_t_);
        const double size_y = max_v_ - min_v_;
        threedimutil::drawRectangle(x, y, size_x, size_y);
    }
    
    // Define the drawing process for a curve
    auto draw_curve = [&](const Curve& curve, const Vector3d& curve_color, double alpha = 1.0, bool draw_handle = true)
    {
        if (use_per_curve_value_range_)
        {
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            const double max_v = curve.GetMaximumKeyValue();
            const double min_v = curve.GetMinimumKeyValue();
            SetOrthMatrix(min_t_, max_t_, min_v - per_curve_value_padding_, max_v + per_curve_value_padding_);
        }
        
        // Sampling the curve
        const double dx = 0.10;
        std::vector<Vector2d> sampling_points;
        for (double x = min_t_; x <= max_t_; x += dx)
        {
            const double v = curve.GetValue(x);
            sampling_points.push_back(Vector2d(x, v));
        }
        
        // Draw the primary curve
        glColor(curve_color, alpha);
        glLineWidth(curve_width);
        glBegin(GL_LINE_STRIP);
        for (const auto& p : sampling_points)
        {
            glVertex(p);
        }
        glEnd();
        
        if (!draw_handle) return;
        
        // Draw the control points and handles
        const auto& points = curve.GetControlPoints();
        const int n_points = static_cast<int>(points.size());
        for (int i = 0; i < n_points; ++ i)
        {
            DrawHandle(points[i],
                       color_.control_point,
                       color_.handle,
                       handle_width,
                       handle_size,
                       control_point_size,
                       i == 0,
                       i == n_points - 1);
        }
        
        if (use_per_curve_value_range_) glPopMatrix();
    };
    
    // Draw original curves
    if (core.show_original_curves_)
    {
        const auto x_original = core.RetrieveOriginalParameters();
        int point_index = 0;
        const int dim_per_point = 5;
        for (auto item : core.object_->GetItems())
        {
            auto vars = item->GetVariablePointers();
            
            for (int axis : { 0, 1, 2 })
            {
                // Copy the curve for thread safety
                Curve curve(vars[axis]->curve_);
                
                for (int i = 0; i < vars[axis]->GetNumOfControlPoints(); ++ i)
                {
                    curve.GetControlPoint(i).SetDescriptor(x_original.segment<5>(point_index * dim_per_point));
                    point_index ++;
                }
                
                if (item->is_selected_ && vars[axis]->IsKeyframed())
                {
                    Vec3 color = (axis == 0) ? color_.curve_x : ((axis == 1) ? color_.curve_y : color_.curve_z);
                    draw_curve(curve, color, 0.5, false);
                }
            }
        }
    }
    
    // Draw curves
    for (auto item : core.object_->GetItems())
    {
        if (!item->is_selected_) continue;
        
        // Draw each degree of freedom (if keyframed)
        auto vars = item->GetVariablePointers();
        if (vars[0]->IsKeyframed()) draw_curve(vars[0]->curve_, color_.curve_x);
        if (vars[1]->IsKeyframed()) draw_curve(vars[1]->curve_, color_.curve_y);
        if (vars[2]->IsKeyframed()) draw_curve(vars[2]->curve_, color_.curve_z);
    }
    
    // Highlight selected handle
    if (drag_target_ != nullptr)
    {
        // TODO: Handle first and last points correctly
        if (use_per_curve_value_range_)
        {
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            const double max_v = drag_target_->curve_ptr->GetMaximumKeyValue() + per_curve_value_padding_;
            const double min_v = drag_target_->curve_ptr->GetMinimumKeyValue() - per_curve_value_padding_;
            SetOrthMatrix(min_t_, max_t_, min_v, max_v);
        }
        if (drag_target_->target_type == TargetType::CENTER)
        {
            DrawHandle(*drag_target_->control_point_ptr,
                       color_.control_point,
                       color_.handle,
                       handle_width,
                       handle_size,
                       control_point_size * 1.5,
                       false,
                       false);
        }
        else
        {
            DrawHandle(*drag_target_->control_point_ptr,
                       color_.control_point,
                       color_.handle,
                       handle_width,
                       handle_size * 1.5,
                       control_point_size,
                       false,
                       false);
        }
        if (use_per_curve_value_range_) glPopMatrix();
    }
    
    // Highlight current frame
    {
        const double x      = core.current_frame_;
        const double y      = 0.5 * (max_v_ + min_v_);
        const double size_x = (current_frame_width / width()) * (max_t_ - min_t_);
        const double size_y = max_v_ - min_v_;
        glColor(color_.current_frame);
        threedimutil::drawRectangle(x, y, size_x, size_y);
    }
}

///////////////////////////////////////////////////////////////////////////

void GraphWidget::UpdateValueRange()
{
    const double x_padding = 0.5;
    min_t_ = static_cast<double>(core.min_frame_) - x_padding;
    max_t_ = static_cast<double>(core.max_frame_) + x_padding;
    
    min_v_ = + DBL_MAX;
    max_v_ = - DBL_MAX;
    for (auto item : core.object_->GetItems())
    {
        if (!item->is_selected_) continue;
        
        for (const Variable* var : item->GetVariablePointers())
        {
            if (!var->IsKeyframed()) continue;
            
            min_v_ = std::min(min_v_, var->GetMinimumKeyValue());
            max_v_ = std::max(max_v_, var->GetMaximumKeyValue());
        }
    }
    const double v_padding = 1.0;
    min_v_ -= v_padding;
    max_v_ += v_padding;
    
    if (min_v_ > max_v_)
    {
        min_v_ = - 2.0;
        max_v_ = + 2.0;
    }
}

///////////////////////////////////////////////////////////////////////////

double GraphWidget::ConvertXToTime(int x) const
{
    return static_cast<double>(x) * (max_t_ - min_t_) / width() + min_t_;
}

double GraphWidget::ConvertYToValue(int y) const
{
    return static_cast<double>(height() - y) * (max_v_ - min_v_) / height() + min_v_;
}

double GraphWidget::ConvertTimeToX (double t) const
{
    return (t - min_t_) * width() / (max_t_ - min_t_);
}

double GraphWidget::ConvertValueToY(double v) const
{
    return height() - (v - min_v_) * height() / (max_v_ - min_v_);
}

double GraphWidget::ScaleXToTime(int x) const
{
    return static_cast<double>(x) * (max_t_ - min_t_) / width();
}

double GraphWidget::ScaleYToValue(int y) const
{
    return - static_cast<double>(y) * (max_v_ - min_v_) / height();
}

double GraphWidget::ScaleTimeToX(double t) const
{
    return t * width() / (max_t_ - min_t_);
}

double GraphWidget::ScaleValueToY(double v) const
{
    return v * height() / (max_v_ - min_v_);
}

///////////////////////////////////////////////////////////////////////////

double GraphWidget::ScaleYToValue(int y, const Curve* curve) const
{
    const double max_v = curve->GetMaximumKeyValue() + per_curve_value_padding_;
    const double min_v = curve->GetMinimumKeyValue() - per_curve_value_padding_;
    return - static_cast<double>(y) * (max_v - min_v) / height();
}

double GraphWidget::ConvertValueToY(double v, const Curve* curve) const
{
    const double max_v = curve->GetMaximumKeyValue() + per_curve_value_padding_;
    const double min_v = curve->GetMinimumKeyValue() - per_curve_value_padding_;
    return height() - (v - min_v) * height() / (max_v - min_v);
}
