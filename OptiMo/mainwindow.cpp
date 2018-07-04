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

#include "mainwindow.h"
#include <iostream>
#include <cstdlib>
#include <QTimer>
#include <QFileDialog>
#include <QProgressDialog>
#include <QFileDialog>
#include <QFutureWatcher>
#include <QtConcurrent>
#include "ui_mainwindow.h"
#include "core.h"
#include "util.h"
#include "sliderdialog.h"
#include "jointweightdialog.h"
#include "timevaryingweightdialog.h"

#define FFMPEG

namespace {
    Core& core = Core::GetInstance();
    
    QTreeWidgetItem* MakeJointItemWithItsChildren(std::shared_ptr<Joint> joint)
    {
        auto item = new QTreeWidgetItem(QStringList(joint->GetName().c_str()));
        
        for (auto child : joint->children_)
        {
            item->addChild(MakeJointItemWithItsChildren(child));
        }
        
        return item;
    }
    
    double GetSliderValue(const QSlider* slider)
    {
        return static_cast<double>(slider->value() - slider->minimum()) / static_cast<double>(slider->maximum() - slider->minimum());
    }

    //////////////////////////////////////////////////
    // UI size control for live demo
    //////////////////////////////////////////////////

    int ui_size             = 1; // 0, 1, 2, 3
    int graph_widget_size   = 1; // 0, 1, 2, 3
    int buttons_widget_size = 1; // 0, 1, 2, 3

    constexpr int font_sizes[]            = {  10,  13,  18,  24 };
    constexpr int button_heights[]        = {  26,  32,  42,  46 };
    constexpr int graph_widget_heights[]  = { 180, 240, 320, 420 };
    constexpr int buttons_widget_widths[] = { 240, 320, 420, 520 };

    void ChangeUiSize()            { ui_size             = (ui_size + 1) % 4;             }
    void ChangeGraphWidgetSize()   { graph_widget_size   = (graph_widget_size + 1) % 4;   }
    void ChangeButtonsWidgetSize() { buttons_widget_size = (buttons_widget_size + 1) % 4; }
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    graph_widget_ = ui->graphWidget;
    main_widget_ = ui->mainWidget;
    graph_widget_->SetMainWindow(this);
    
    UpdateSliderMinMax();
    UpdateTimelineUi();
    
    timer_ = std::make_shared<QTimer>(this);
    connect(timer_.get(), SIGNAL(timeout()), this, SLOT(StepFrame()));
    if (core.is_playing_)
    {
        timer_->start(1000 / core.frames_per_second_);
    }
    ui->playButton->setEnabled(!core.is_playing_);
    ui->stopButton->setEnabled(core.is_playing_);
    ui->stepButton->setEnabled(!core.is_playing_);

    ui->checkBox_cost_vis->setChecked(core.show_cost_visualization_);
    ui->checkBox_process_vis->setChecked(core.show_process_animation_);

    UpdateTreeViewContents();
    
    // Disable some widgets
    ui->widget->setVisible(false);
    ui->stepButton->setVisible(false);
    
    core.UpdateFreezedDofsFromSelection();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::UpdateTreeViewContents()
{
    ui->treeWidget->clear();
    ui->treeWidget->setHeaderLabel(QString("List"));
    ui->treeWidget->setSelectionMode(QAbstractItemView::SelectionMode::MultiSelection);
    ui->treeWidget->addTopLevelItem(MakeJointItemWithItsChildren(core.object_->GetRootJoint()));
    for (auto ik_handler : core.object_->GetIkHandlers())
    {
        auto item = new QTreeWidgetItem(QStringList(ik_handler->GetName().c_str()));
        ui->treeWidget->addTopLevelItem(item);
    }
    ui->treeWidget->expandAll();
}

void MainWindow::UpdateTimelineUi()
{
    ui->timelineSlider->setValue(core.current_frame_);
    ui->frameLabel->setText(QString::fromStdString("<b>" + std::to_string(core.current_frame_) + "</b>/" + std::to_string(core.max_frame_)));
    
    ui->graphWidget->update();
    ui->mainWidget->update();
}

void MainWindow::UpdateSliderMinMax()
{
    ui->timelineSlider->setMinimum(core.min_frame_);
    ui->timelineSlider->setMaximum(core.max_frame_);
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::on_timelineSlider_sliderMoved(int position)
{
    const int new_frame = position;
    core.current_frame_ = new_frame;
    UpdateTimelineUi();
}

void MainWindow::on_timelineSlider_sliderPressed()
{
    const int new_frame = ui->timelineSlider->value();
    core.current_frame_ = new_frame;
    UpdateTimelineUi();
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::on_global_weight_slider_sliderMoved(int /* position */)
{
    const double x = GetSliderValue(ui->global_weight_slider);
    core.UpdateGlobalWeight(SliderDialog::MapSliderValueToWeight(x));
}

void MainWindow::on_global_weight_slider_sliderPressed()
{
    const double x = GetSliderValue(ui->global_weight_slider);
    core.UpdateGlobalWeight(SliderDialog::MapSliderValueToWeight(x));
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::StepFrame()
{
    const int new_frame = (core.current_frame_ >= core.max_frame_) ? core.min_frame_ : core.current_frame_ + 1;
    core.current_frame_ = new_frame;
    UpdateTimelineUi();
}

void MainWindow::on_playButton_clicked()
{
    core.is_playing_ = true;
    timer_->start(1000 / core.frames_per_second_);
    
    ui->playButton->setEnabled(!core.is_playing_);
    ui->stopButton->setEnabled(core.is_playing_);
    ui->stepButton->setEnabled(!core.is_playing_);
}

void MainWindow::on_stopButton_clicked()
{
    core.is_playing_ = false;
    timer_->stop();

    ui->playButton->setEnabled(!core.is_playing_);
    ui->stopButton->setEnabled(core.is_playing_);
    ui->stepButton->setEnabled(!core.is_playing_);
}

void MainWindow::on_stepButton_clicked()
{
    StepFrame();
}

void MainWindow::on_actionPlay_triggered()
{
    core.is_playing_ = !core.is_playing_;
    if (core.is_playing_)
    {
        timer_->start(1000 / core.frames_per_second_);
    }
    else
    {
        timer_->stop();
    }

    ui->playButton->setEnabled(!core.is_playing_);
    ui->stopButton->setEnabled( core.is_playing_);
    ui->stepButton->setEnabled(!core.is_playing_);
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::on_actionPerform_Optimization_triggered()
{
    // Define the background process
    auto background_process = [&]()
    {
        core.PerformOptimization();
    };

    // Set timer for updating the graph widget during optimization
    auto timer = std::make_shared<QTimer>(this);
    connect(timer.get(), SIGNAL(timeout()), ui->graphWidget, SLOT(update()));
    
    if (core.show_process_animation_)
    {
        timer->start(1000 / 30);
    }
    else
    {
        on_stopButton_clicked();
    }

    // Show progress diaglog and run optimization
    QProgressDialog dialog(QString("Optimizing..."), QString(), 0, 0, this);
    QFutureWatcher<void> watcher;
    QObject::connect(&watcher, SIGNAL(finished()), &dialog, SLOT(reset()));
    watcher.setFuture(QtConcurrent::run(background_process));
    dialog.exec();

    // Wait for optimization finished
    watcher.waitForFinished();
    
    // Stop timer (if it is running)
    if (core.show_process_animation_)
    {
        timer->stop();
    }
    
    // Post processing
    core.PushHistory();
    core.UpdateOriginalParameters(core.object_->GetDescriptor());
    UpdateTimelineUi();
}

void MainWindow::on_actionOptimize_Interactively_triggered()
{
    // Save the original parameters for the case of cancel
    const auto x_original = core.object_->GetDescriptor();
    
    // Set timer for updating the graph widget during optimization
    auto timer = std::make_shared<QTimer>(this);
    connect(timer.get(), SIGNAL(timeout()), ui->graphWidget, SLOT(update()));
    timer->start(1000 / 30);
    
    // Show diaglog
    SliderDialog dialog(this);
    
    // Define the background process
    std::function<void()> background_process = [&]()
    {
        core.PerformOptimization(true);
        if (!dialog.done) background_process();
    };

    // Run optimization
    QFutureWatcher<void> watcher;
    watcher.setFuture(QtConcurrent::run(background_process));
    dialog.exec();
    
    // Wait for optimization finished
    watcher.waitForFinished();
    timer->stop();
    
    // Post processing
    if (!dialog.canceled)
    {
        core.PushHistory();
        core.UpdateOriginalParameters(core.object_->GetDescriptor());
        UpdateTimelineUi();
    }
    else
    {
        core.object_->SetDescriptor(x_original);
        core.ClearOptimizationCache();
        UpdateTimelineUi();
    }
}

void MainWindow::on_optimizeButton_clicked()
{
    on_actionPerform_Optimization_triggered();
}

void MainWindow::on_optimizeRegularizeButton_clicked()
{
    on_actionOptimize_Interactively_triggered();
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::on_actionShow_Hide_visualization_triggered()
{
    core.show_cost_visualization_ = !core.show_cost_visualization_;
    UpdateTimelineUi();

    ui->checkBox_cost_vis->setChecked(core.show_cost_visualization_);
}

void MainWindow::on_actionShow_Hide_Original_Curves_triggered()
{
    core.show_original_curves_ = !core.show_original_curves_;
    UpdateTimelineUi();
}

void MainWindow::on_actionShow_Hide_Torques_triggered()
{
    core.show_torques_ = !core.show_torques_;
    UpdateTimelineUi();
}

void MainWindow::on_actionShow_Hide_Optimization_Process_triggered()
{
    core.show_process_animation_ = !core.show_process_animation_;

    ui->checkBox_process_vis->setChecked(core.show_process_animation_);
}


void MainWindow::on_checkBox_cost_vis_stateChanged(int)
{
    core.show_cost_visualization_ = (ui->checkBox_cost_vis->checkState() == Qt::Checked);
    UpdateTimelineUi();
}

void MainWindow::on_checkBox_process_vis_stateChanged(int)
{
    core.show_process_animation_ = (ui->checkBox_process_vis->checkState() == Qt::Checked);
    UpdateTimelineUi();
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::on_actionExport_as_Video_triggered()
{
    const std::string base_directory = QFileDialog::getExistingDirectory().toStdString();
    
    // Prepare output directory
    if (std::system(("cd " + base_directory).c_str()) != 0)
    {
        std::cerr << "The directory does not exist." << std::endl;
        return;
    }
    const std::string directory = base_directory + "/" + Util::GetCurrentTimeInString();
    if (std::system(("mkdir " + directory).c_str()) != 0)
    {
        std::cerr << "Warning: the directory already exists; the old files will be overrided." << std::endl;
    }
    std::system(("mkdir " + directory + "/graph").c_str());
    std::system(("mkdir " + directory + "/result").c_str());
    std::system(("mkdir " + directory + "/move").c_str());
    std::system(("mkdir " + directory + "/window").c_str());
    
    // Render images
    for (int i = core.min_frame_; i <= core.max_frame_; ++ i)
    {
        core.current_frame_ = i;
        UpdateTimelineUi();
        this->grab().save(QString::fromStdString(directory) + "/window/window" + QString("%1").arg(i, 6, 10, QChar('0')) + QString(".png"));
        ui->mainWidget->grabFramebuffer().save(QString::fromStdString(directory) + "/result/result" + QString("%1").arg(i, 6, 10, QChar('0')) + QString(".png"));
        ui->graphWidget->grabFramebuffer().save(QString::fromStdString(directory) + "/graph/graph" + QString("%1").arg(i, 6, 10, QChar('0')) + QString(".png"));
    }
    
#if 0
    // Render images with camera moving
    constexpr int loop = 4;
    const int number_of_export_frames = loop * (core.max_frame_ - core.min_frame_ + 1);
    const Eigen::Vector3d pan_velocity(2.0, 0.0, 1.0);
    const Eigen::Vector3d original_camera_position = core.camera_.position_;
    int index = 0;
    for (int i = 0; i < loop; ++ i)
    {
        for (int i = core.min_frame_; i <= core.max_frame_; ++ i)
        {
            core.camera_.position_ = original_camera_position + (static_cast<double>(index) / static_cast<double>(number_of_export_frames) - 0.5) * pan_velocity;
            core.current_frame_ = i;
            UpdateTimelineUi();
            ui->mainWidget->grabFramebuffer().save(QString::fromStdString(directory) + "/move/move" + QString("%1").arg(index, 6, 10, QChar('0')) + QString(".png"));
            ++ index;
        }
    }
    core.camera_.position_ = original_camera_position;
#endif
    
    core.current_frame_ = core.min_frame_ - 1;
    UpdateTimelineUi();
    ui->graphWidget->grabFramebuffer().save(QString::fromStdString(directory) + "/graph.png");
    core.show_time_varying_weight_ = true;
    UpdateTimelineUi();
    ui->graphWidget->grabFramebuffer().save(QString::fromStdString(directory) + "/weight.png");
    core.show_time_varying_weight_ = false;
    core.current_frame_ = core.min_frame_;
    
#ifdef FFMPEG
    // Convert images into animations (this requires ffmpeg)
    std::system(std::string("/usr/local/bin/ffmpeg -framerate " + std::to_string(core.frames_per_second_) + " -i " + directory + "/window/window%06d.png -pix_fmt yuv420p -r " + std::to_string(core.frames_per_second_) + " -y " + directory + "/window.mp4").c_str());
    std::system(std::string("/usr/local/bin/ffmpeg -framerate " + std::to_string(core.frames_per_second_) + " -i " + directory + "/result/result%06d.png -pix_fmt yuv420p -r " + std::to_string(core.frames_per_second_) + " -y " + directory + "/result.mp4").c_str());
    std::system(std::string("/usr/local/bin/ffmpeg -framerate " + std::to_string(core.frames_per_second_) + " -i " + directory + "/move/move%06d.png -pix_fmt yuv420p -r " + std::to_string(core.frames_per_second_) + " -y " + directory + "/move.mp4").c_str());
    std::system(std::string("/usr/local/bin/ffmpeg -framerate " + std::to_string(core.frames_per_second_) + " -i " + directory + "/graph/graph%06d.png -pix_fmt yuv420p -r " + std::to_string(core.frames_per_second_) + " -y " + directory + "/graph.mp4").c_str());
#endif
    
    // Output JSON
    core.WriteJson(directory + "/scene.json");
    core.WriteHistory(directory + "/history.json");
    core.ExportMotion(directory + "/motion.json");
}

void MainWindow::on_actionImport_JSON_triggered()
{
    const std::string file_path = QFileDialog::getOpenFileName().toStdString();
    core.ReadJson(file_path);
    core.ClearHistory();
    core.ClearOptimizationCache();
    core.PushHistory();
    core.UpdateOriginalParameters(core.object_->GetDescriptor());
    UpdateSliderMinMax();
    UpdateTreeViewContents();
    UpdateTimelineUi();
    ui->graphWidget->UpdateValueRange();
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::on_treeWidget_itemSelectionChanged()
{
    // Reset all the selections
    for (auto item : core.object_->GetItems()) item->is_selected_ = false;

    // Set selections
    for (auto item : ui->treeWidget->selectedItems())
    {
        core.object_->GetItemByName(item->text(0).toStdString())->is_selected_ = true;
    }
    
    // Finialize the process
    core.UpdateFreezedDofsFromSelection();
    UpdateTimelineUi();
    ui->graphWidget->UpdateValueRange();
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::on_actionSet_Predefined_Camera_Position_1_triggered()
{
    core.camera_.position_ = Eigen::Vector3d(- 2.0, 0.0, 3.0);
    ui->mainWidget->update();
}

void MainWindow::on_actionSet_Predefined_Camera_Position_2_triggered()
{
    // For the conductor scene v3.3
    core.camera_.position_ = Eigen::Vector3d(- 3.0, 3.0, 9.0);
    core.camera_.target_   = Eigen::Vector3d(  0.0, 3.0, 0.5);
    core.camera_.vertical_angle_of_view_ = 30.0;
    ui->mainWidget->update();
}

void MainWindow::on_actionSet_Predefined_Camera_Position_3_triggered()
{
    // For the fox tail scene v3.1
    core.camera_.position_ = Eigen::Vector3d(- 5.499, 4.318, 13.955);
    core.camera_.target_   = Eigen::Vector3d(  0.038, 1.858,  1.928);
    core.camera_.vertical_angle_of_view_ = 30.0;
    ui->mainWidget->update();
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::on_actionUndo_triggered()
{
    core.PopHistory();
    core.UpdateOriginalParameters(core.object_->GetDescriptor());
    on_treeWidget_itemSelectionChanged(); // For maintaining the joint selection
    UpdateTimelineUi();
}

void MainWindow::on_undoButton_clicked()
{
    on_actionUndo_triggered();
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::on_actionCapture_Window_triggered()
{
    const std::string base_directory = QFileDialog::getExistingDirectory().toStdString();
    const std::string file_name      = Util::GetCurrentTimeInString() + ".png";
    
    this->grab().save(QString::fromStdString(base_directory + "/" + file_name));
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::on_actionEdit_Time_Varying_Weight_triggered()
{
    core.show_time_varying_weight_ = true;
    
    const auto x_original = core.object_->GetDescriptor();

    auto timer = std::make_shared<QTimer>(this);
    connect(timer.get(), SIGNAL(timeout()), ui->graphWidget, SLOT(update()));
    timer->start(1000 / 30);
    
    core.AddKernel(core.current_frame_);
    
    TimeVaryingWeightDialog dialog(this, core.GetLastKernelPointer());
    
    // Define the background process
    std::function<void()> background_process = [&]()
    {
        if (dialog.IsInteractive()) core.PerformOptimization(true);
        if (!dialog.Done()) background_process();
    };
    
    // Run optimization
    QFutureWatcher<void> watcher;
    watcher.setFuture(QtConcurrent::run(background_process));
    dialog.exec();
    
    // Wait for optimization finished
    watcher.waitForFinished();
    timer->stop();
    
    core.show_time_varying_weight_ = false;

    if (!core.object_->GetDescriptor().isApprox(x_original))
    {
        core.PushHistory();
        core.UpdateOriginalParameters(core.object_->GetDescriptor());
    }
    
    UpdateTimelineUi();
}

void MainWindow::on_actionShow_Hide_Time_Varying_Weight_triggered()
{
    core.show_time_varying_weight_ = !core.show_time_varying_weight_;
    UpdateTimelineUi();
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::on_actionSet_Cyclic_Non_Cyclic_triggered()
{
    core.object_->is_cyclic_ = !core.object_->is_cyclic_;
}

/////////////////////////////////////////////////////////////////////////

namespace
{
    constexpr double scaling_factor = 1.2;
}

void MainWindow::on_actionIncrease_Scale_triggered()
{
    core.drawing_scale_ *= scaling_factor;
    UpdateTimelineUi();
}

void MainWindow::on_actionDecrease_Scale_triggered()
{
    core.drawing_scale_ /= scaling_factor;
    UpdateTimelineUi();
}

void MainWindow::on_pushButton_clicked()
{
    on_actionEdit_Time_Varying_Weight_triggered();
}

void MainWindow::on_actionChange_Curve_Editor_Size_triggered()
{
    ChangeGraphWidgetSize();

    // Change the graph widget height
    ui->graphWidget->setMinimumHeight(graph_widget_heights[graph_widget_size]);

    // Refrash the window
    update();
}

void MainWindow::on_actionChange_UI_Size_triggered()
{
    ChangeUiSize();

    // Change the font size
    QApplication* app = qApp;
    QFont font = app->font();
    font.setPointSize(font_sizes[ui_size]);
    app->setFont(font);

    // Change the push button height
    app->setStyleSheet(QString(("QPushButton { min-height: " + std::to_string(button_heights[ui_size]) + "px; }").c_str()));

    // Refrash the window
    update();
}

void MainWindow::on_actionChange_Buttons_Widget_Size_triggered()
{
    ChangeButtonsWidgetSize();

    // Change the buttons widget width
    ui->scrollArea->setMinimumWidth(buttons_widget_widths[buttons_widget_size]);

    // Refrash the window
    update();
}
