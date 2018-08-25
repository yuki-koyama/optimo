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

#include "mainwidget.h"
#include <iostream>
#include <QMouseEvent>
#include <Eigen/Geometry>
#include <three-dim-util/gl.hpp>
#include <three-dim-util/gl-wrapper.hpp>
#include <three-dim-util/matrix.hpp>
#include <three-dim-util/draw-functions.hpp>
#include "core.h"
#include "util.h"
#include "jointweightdialog.h"

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Matrix4d;
using Eigen::Affine3d;

namespace
{
    Core& core = Core::GetInstance();
}

MainWidget::MainWidget(QWidget *parent) : QOpenGLWidget(parent)
{
    // Set the number of samplings for anti-aliasing
    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
    format.setSamples(4);
    this->setFormat(format);
}

void MainWidget::mousePressEvent(QMouseEvent *event)
{
    const int x_dev = devicePixelRatio() * event->x();
    const int y_dev = devicePixelRatio() * event->y();
    
    const Vector2d cursor(event->x(), event->y());
    
    if (event->modifiers() == Qt::AltModifier)
    {
        core.camera_.BeginTrackball(x_dev, y_dev, threedimutil::Camera::Mode::Rotate);
        if (!core.is_playing_) { update(); }
    }
    else
    {
        // Make this gl context current
        this->makeCurrent();
        
        glMatrixMode(GL_MODELVIEW);
        threedimutil::load_matrix(threedimutil::make_look_at(core.camera_));
        
        // Retrieve the model view matrix
        Matrix4d modelview_matrix;
        glGetDoublev(GL_MODELVIEW_MATRIX, modelview_matrix.data());
        const Affine3d modelview_affine(modelview_matrix);
        
        // Retrieve the projection matrix
        Matrix4d projection_matrix;
        glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix.data());
        
        // Release this gl context
        this->doneCurrent();
        
        // Find the closest item
        double min_dist = 0.0;
        std::shared_ptr<Item> min_item_ptr = nullptr;
        
        const auto items = core.object_->GetItems();
        for (auto item : items)
        {
            const Vector3d x_world  = item->GetPosition(core.current_frame_);
            const Vector3d x_camera = modelview_affine * x_world;
            const Vector3d x_pers   = (projection_matrix * x_camera.homogeneous()).hnormalized();
            const Vector2d x_screen = Vector2d((x_pers(0) + 1.0) * width() / 2.0, (1.0 - x_pers(1)) * height() / 2.0);
            
            const double dist = (x_screen - cursor).norm();
            
            if (dist < min_dist || min_item_ptr == nullptr)
            {
                min_dist = dist;
                min_item_ptr = item;
            }
        }
        
        const double threshold = 20.0;
        if (min_dist < threshold)
        {
            std::cout << min_item_ptr->GetName() << std::endl;
            
            // Ask the user to specify the weight
            JointWeightDialog dialog(this, min_item_ptr->GetName());
            dialog.exec();
        }
        else
        {
            core.camera_.BeginTrackball(x_dev, y_dev, threedimutil::Camera::Mode::Pan);
            if (!core.is_playing_) { update(); }
        }
    }
}

void MainWidget::mouseMoveEvent(QMouseEvent *event)
{
    const int x_dev = devicePixelRatio() * event->x();
    const int y_dev = devicePixelRatio() * event->y();
    
    core.camera_.MoveTrackball(x_dev, y_dev);
    
    if (!core.is_playing_) { update(); }
}

void MainWidget::mouseReleaseEvent(QMouseEvent *)
{
    core.camera_.EndTrackball();
    if (!core.is_playing_) { update(); }
}

void MainWidget::wheelEvent(QWheelEvent *event)
{
    core.camera_.BeginTrackball(0, 0, threedimutil::Camera::Mode::Zoom);
    core.camera_.MoveTrackball (0, event->delta());
    core.camera_.EndTrackball  ();
    if (!core.is_playing_) { update(); }
}

void MainWidget::initializeGL()
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    
    glEnable(GL_DEPTH_TEST);
    
    // Lighting
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
}

void MainWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void MainWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    const double aspect = static_cast<double>(this->width()) / static_cast<double>(this->height());
    
    // Set projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(threedimutil::make_perspective(core.camera_.vertical_angle_of_view(), aspect, 0.05, 20.0).data());
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glLightfv(GL_LIGHT0, GL_POSITION, std::vector<GLfloat>{ + 1.0, 2.0, 3.0, 0.0 }.data());
    glLightfv(GL_LIGHT1, GL_POSITION, std::vector<GLfloat>{ - 1.0, 0.0, 3.0, 0.0 }.data());
    
    glLoadMatrixd(threedimutil::make_look_at(core.camera_).data());
    
    const double t = core.current_frame_;
    
    // Draw the main character
    core.object_->Draw(t);
    
    // Visualize torques
    if (core.show_torques_)
    {
        const double max_length = 2.0;
        const Eigen::Vector3d color1(0.0, 0.6, 0.0);
        const Eigen::Vector3d color2(0.0, 0.0, 0.6);
        const double radius = core.drawing_scale_ * 0.025;
        
        const Eigen::VectorXd tau = core.CalculateTorques(t);
        int i = 0;
        for (auto joint : core.object_->GetJoints())
        {
            if (joint->parent_.expired())
            {
                ++ i;
                continue;
            }
            
            const Eigen::Affine3d transformation = joint->parent_.lock()->GetAffineRelativeToWorld(t);
            glPushMatrix();
            glMultMatrixd(transformation.data());
            
            const Eigen::Vector3d local_torque = tau.segment(i * 3, 3);
            const Eigen::Vector3d scaled_local_torque = core.drawing_scale_ * (max_length / core.max_torque_) * local_torque;
            
            threedimutil::color_3d(color1);
            threedimutil::draw_cylinder(radius, scaled_local_torque, Vector3d::Zero());
            threedimutil::draw_sphere(radius, scaled_local_torque);
            threedimutil::color_3d(color2);
            threedimutil::draw_cylinder(radius, Vector3d::Zero(), - scaled_local_torque);
            threedimutil::draw_sphere(radius, - scaled_local_torque);
            
            glPopMatrix();
            
            ++ i;
        }
    }
}
