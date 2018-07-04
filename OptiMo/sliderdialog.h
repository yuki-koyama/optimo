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

#ifndef SLIDERDIALOG_H
#define SLIDERDIALOG_H

#include <cmath>
#include <QDialog>

class MainWindow;
class QSlider;

class SliderDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SliderDialog(MainWindow* parent = nullptr);
    
    QSlider* slider_;
    
    /// Once the OK button is pushed, this parameter is set to true
    bool done = false;
    
    /// If the "cancel" button is pushed, this parameter is set to true
    bool canceled = false;
    
    static double MapSliderValueToWeight(double slider_value)
    {
        return 100.0 * std::pow(slider_value, 3.0);
    }
    
public slots:
    
    /// This slot informs the optimizer to stop the optimization process
    void accept();

    /// This slot informs the optimizer to stop the optimization process
    void reject();
    
private slots:
    
    void on_slider_value_changed(int value);
};
#endif // SLIDERDIALOG_H
