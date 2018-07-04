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

#ifndef JOINTWEIGHTDIALOG_H
#define JOINTWEIGHTDIALOG_H

#include <cmath>
#include <QDialog>

class MainWidget;
class QSlider;

class JointWeightDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit JointWeightDialog(MainWidget* parent, const std::string& item_name);

    static double MapSliderValueToWeight(double slider_value)
    {
        return 100.0 * std::pow(slider_value, 3.0);
    }
    
    static double MapWeightToSliderValue(double weight)
    {
        return std::pow(weight / 100.0, 1.0 / 3.0);
    }

private:
    MainWidget* main_widget_;
    QSlider* slider_;
    std::string item_name_;

public slots:
    
    void accept();
};

#endif // JOINTWEIGHTDIALOG_H
