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

#ifndef TIMEVARYINGWEIGHTDIALOG_H
#define TIMEVARYINGWEIGHTDIALOG_H

#include <QDialog>

class MainWindow;
class QSlider;
class QCheckBox;
struct GaussianKernel;

class TimeVaryingWeightDialog : public QDialog
{
    Q_OBJECT
public:
    TimeVaryingWeightDialog(MainWindow* main_window, GaussianKernel* kernel);
    
    bool Done() const { return done_; }
    bool IsInteractive() const { return is_interactive_; }
    
private:
    QSlider* sigma_slider_;
    QSlider* time_slider_;
    QCheckBox* check_box_;
    GaussianKernel* kernel_;
    
    bool done_           = false;
    bool is_interactive_ = false;
    
public slots:
    
    void accept();
    
private slots:
    
    void on_slider_value_changed(int value);
    void on_checked(int state);
};

#endif // TIMEVARYINGWEIGHTDIALOG_H
