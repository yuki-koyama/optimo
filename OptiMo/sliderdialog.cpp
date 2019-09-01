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

#include "sliderdialog.h"
#include <iostream>
#include <QLayout>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QDialogButtonBox>
#include "mainwindow.h"
#include "core.h"

namespace
{
    Core& core = Core::GetInstance();

    inline double GetSliderValue(const QSlider* slider)
    {
        return static_cast<double>(slider->value() - slider->minimum()) / static_cast<double>(slider->maximum() - slider->minimum());
    }
}

SliderDialog::SliderDialog(MainWindow* parent) : QDialog(parent)
{
    QVBoxLayout* layout = new QVBoxLayout();

    this->setLayout(layout);

    slider_ = new QSlider(Qt::Horizontal);
    slider_->setMaximum(20);

    QLabel* label = new QLabel(QString("<b>Regularization weight:</b>"));
    QDialogButtonBox* button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

    layout->addWidget(label);
    layout->addWidget(slider_);
    layout->addWidget(button_box);

    QObject::connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(on_slider_value_changed(int)));
    QObject::connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
    QObject::connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

    // Set the initial tick position at the center of the slider
    slider_->setValue((slider_->maximum() - slider_->minimum()) / 2);
}

void SliderDialog::accept()
{
    done = true;
    core.stop_optimization_ = true;
    QDialog::accept();
}

void SliderDialog::reject()
{
    done = true;
    canceled = true;
    core.stop_optimization_ = true;
    QDialog::reject();
}

void SliderDialog::on_slider_value_changed(int /* value */)
{
    const double weight = MapSliderValueToWeight(GetSliderValue(slider_));
    core.UpdateGlobalWeight(weight);
    core.stop_optimization_ = true;
}
