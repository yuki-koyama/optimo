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

#include "jointweightdialog.h"
#include <QSlider>
#include <QDialogButtonBox>
#include <QLayout>
#include <QLabel>
#include "mainwidget.h"
#include "core.h"

namespace
{
    Core& core = Core::GetInstance();
    
    inline double GetSliderValue(const QSlider* slider)
    {
        return static_cast<double>(slider->value() - slider->minimum()) / static_cast<double>(slider->maximum() - slider->minimum());
    }
}

JointWeightDialog::JointWeightDialog(MainWidget* parent, const std::string& item_name) : QDialog(parent), main_widget_(parent), item_name_(item_name)
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    
    slider_ = new QSlider(Qt::Horizontal, this);
    slider_->setMaximum(10);
    
    QLabel* label = new QLabel(QString("<b>Regularization weight for this joint:</b>"), this);
    
    layout->addWidget(label);
    layout->addWidget(slider_);
    
    QDialogButtonBox* button_box = new QDialogButtonBox(QDialogButtonBox::Ok);
    layout->addWidget(button_box);
    
    QObject::connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
    
    const int    item_index   = core.object_->GetItemIndexByName(item_name_);
    const double weight       = core.per_item_weights_[item_index];
    const double slider_value = MapWeightToSliderValue(weight);
    slider_->setValue(static_cast<int>(std::round(slider_value * (slider_->maximum() - slider_->minimum()) + slider_->minimum())));
    
    setLayout(layout);
}

void JointWeightDialog::accept()
{
    const double slider_value = GetSliderValue(slider_);
    const double weight = MapSliderValueToWeight(slider_value);
    core.UpdatePerItemWeight(weight, item_name_);
    QDialog::accept();
}
