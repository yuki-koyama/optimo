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

#include "timevaryingweightdialog.h"
#include <QSlider>
#include <QLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <QCheckBox>
#include "mainwindow.h"
#include "core.h"

namespace
{
    Core& core = Core::GetInstance();

    inline double GetSliderValue(const QSlider* slider)
    {
        return static_cast<double>(slider->value() - slider->minimum()) / static_cast<double>(slider->maximum() - slider->minimum());
    }
    
    inline void SetSliderValue(double value, QSlider* slider)
    {
        slider->setValue(static_cast<int>(std::round(value * (slider->maximum() - slider->minimum()) + slider->minimum())));
    }
    
    inline double ConvertSliderValueToSigma(double slider_value)
    {
        return 0.1 + slider_value * 5.0;
    }
    
    inline double ConvertSigmaToSliderValue(double sigma)
    {
        return (sigma - 0.1) / 5.0;
    }
    
    inline double ConvertSliderValueToTime(double slider_value)
    {
        return core.min_frame_ + (core.max_frame_ - core.min_frame_) * slider_value;
    }
    
    inline double ConvertTimeToSliderValue(double time)
    {
        return (time - core.min_frame_) / (core.max_frame_ - core.min_frame_);
    }
}

TimeVaryingWeightDialog::TimeVaryingWeightDialog(MainWindow* main_window, GaussianKernel* kernel) : QDialog(main_window), kernel_(kernel)
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    
    sigma_slider_ = new QSlider(Qt::Horizontal, this);
    time_slider_  = new QSlider(Qt::Horizontal, this);

    time_slider_->setMinimum(0);
    time_slider_->setMaximum(core.max_frame_ - core.min_frame_);

    check_box_ = new QCheckBox("Interactive Preview", this);

    QLabel* label = new QLabel(QString("<b>Weight function:</b>"), this);
    
    layout->addWidget(label);
    layout->addWidget(time_slider_);
    layout->addWidget(sigma_slider_);
    layout->addWidget(check_box_);
    
    QDialogButtonBox* button_box = new QDialogButtonBox(QDialogButtonBox::Ok);
    layout->addWidget(button_box);
    
    QObject::connect(sigma_slider_, SIGNAL(valueChanged(int)), this, SLOT(on_slider_value_changed(int)));
    QObject::connect(time_slider_, SIGNAL(valueChanged(int)), this, SLOT(on_slider_value_changed(int)));
    QObject::connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
    QObject::connect(check_box_, SIGNAL(stateChanged(int)), this, SLOT(on_checked(int)));
    
    const double sigma_slider_value = ConvertSigmaToSliderValue(kernel_->sigma);
    SetSliderValue(sigma_slider_value, sigma_slider_);
    
    const double time_slider_value = ConvertTimeToSliderValue(kernel_->mu);
    SetSliderValue(time_slider_value, time_slider_);
    
    setLayout(layout);
}

void TimeVaryingWeightDialog::accept()
{
    const double sigma_slider_value = GetSliderValue(sigma_slider_);
    kernel_->sigma = ConvertSliderValueToSigma(sigma_slider_value);
    
    const double time_slider_value = GetSliderValue(time_slider_);
    kernel_->mu = ConvertSliderValueToTime(time_slider_value);

    done_ = true;
    core.stop_optimization_ = true;
    
    QDialog::accept();
}

void TimeVaryingWeightDialog::on_slider_value_changed(int /* value */)
{
    const double slider_value = GetSliderValue(sigma_slider_);
    kernel_->sigma = ConvertSliderValueToSigma(slider_value);

    const double time_slider_value = GetSliderValue(time_slider_);
    kernel_->mu = ConvertSliderValueToTime(time_slider_value);
    
    core.stop_optimization_ = true;
}

void TimeVaryingWeightDialog::on_checked(int /* state */)
{
    is_interactive_ = check_box_->isChecked();
    core.stop_optimization_ = true;
}

