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

#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <memory>
#include <QOpenGLWidget>
#include <Eigen/Core>

class QImage;
class MainWindow;
class Curve;
struct ControlPoint;

class GraphWidget : public QOpenGLWidget
{
    Q_OBJECT
public:
    explicit GraphWidget(QWidget *parent = nullptr);
    
    struct Color {
        const Eigen::Vector3d background   {1.00, 1.00, 1.00};
        const Eigen::Vector3d grid_line    {0.85, 0.85, 0.85};
        const Eigen::Vector3d curve_x      {0.60, 0.20, 0.20};
        const Eigen::Vector3d curve_y      {0.20, 0.60, 0.20};
        const Eigen::Vector3d curve_z      {0.20, 0.20, 0.60};
        const Eigen::Vector3d handle       {0.60, 0.40, 0.80};
        const Eigen::Vector3d control_point{0.30, 0.20, 0.40};
        const Eigen::Vector3d current_frame{0.80, 0.20, 0.20};
    } color_;
    
    void SetMainWindow(MainWindow* main_window) { main_window_ = main_window; }
    void UpdateValueRange();

signals:

public slots:

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    
private:
    
    MainWindow* main_window_;
    
    int current_mouse_pos_x_;
    int current_mouse_pos_y_;

    double min_t_;
    double max_t_;
    double min_v_;
    double max_v_;
    
    const bool   use_per_curve_value_range_ = true;
    const double per_curve_value_padding_   = 0.5;
    
    enum class TargetType { CENTER, BACKWARD, FORWARD };
    
    struct DragTarget
    {
        Curve*        curve_ptr;
        ControlPoint* control_point_ptr;
        TargetType    target_type;
    };
    
    /// This is nullptr when no handle is dragging.
    std::shared_ptr<DragTarget> drag_target_;
    
    double ConvertXToTime (int x) const;
    double ConvertYToValue(int y) const;
    double ConvertTimeToX (double t) const;
    double ConvertValueToY(double v) const;
    double ScaleXToTime (int x) const;
    double ScaleYToValue(int y) const;
    double ScaleTimeToX (double t) const;
    double ScaleValueToY(double v) const;
    
    double ConvertValueToY(double v, const Curve* curve) const;
    double ScaleYToValue(int y, const Curve* curve) const;
};

#endif // GRAPHWIDGET_H
