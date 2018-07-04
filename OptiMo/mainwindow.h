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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <memory>
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class GraphWidget;
class MainWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
    GraphWidget* graph_widget_;
    MainWidget* main_widget_;

private slots:
    void on_timelineSlider_sliderMoved(int position);

    void on_timelineSlider_sliderPressed();

    void on_playButton_clicked();
    
    void StepFrame();

    void on_stopButton_clicked();

    void on_stepButton_clicked();

    void on_actionShow_Hide_visualization_triggered();

    void on_actionExport_as_Video_triggered();

    void on_actionPerform_Optimization_triggered();

    void on_actionImport_JSON_triggered();

    void on_optimizeButton_clicked();

    void on_treeWidget_itemSelectionChanged();

    void on_actionSet_Predefined_Camera_Position_1_triggered();

    void on_actionSet_Predefined_Camera_Position_2_triggered();

    void on_actionSet_Predefined_Camera_Position_3_triggered();

    void on_actionUndo_triggered();

    void on_undoButton_clicked();

    void on_global_weight_slider_sliderMoved(int position);

    void on_global_weight_slider_sliderPressed();

    void on_actionPlay_triggered();

    void on_actionCapture_Window_triggered();

    void on_actionShow_Hide_Original_Curves_triggered();

    void on_actionShow_Hide_Torques_triggered();

    void on_actionOptimize_Interactively_triggered();

    void on_optimizeRegularizeButton_clicked();

    void on_actionEdit_Time_Varying_Weight_triggered();

    void on_actionShow_Hide_Time_Varying_Weight_triggered();

    void on_actionSet_Cyclic_Non_Cyclic_triggered();

    void on_actionIncrease_Scale_triggered();

    void on_actionDecrease_Scale_triggered();

    void on_actionShow_Hide_Optimization_Process_triggered();

    void on_pushButton_clicked();

    void on_actionChange_Curve_Editor_Size_triggered();

    void on_actionChange_UI_Size_triggered();

    void on_checkBox_cost_vis_stateChanged(int);

    void on_checkBox_process_vis_stateChanged(int);

    void on_actionChange_Buttons_Widget_Size_triggered();

private:

    void UpdateTimelineUi();
    void UpdateTreeViewContents();
    void UpdateSliderMinMax();
    
    Ui::MainWindow *ui;
    
    std::shared_ptr<QTimer> timer_;
};

#endif // MAINWINDOW_H
