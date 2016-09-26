/*
 * File         : controlPanel.hpp
 * Author       : Benjamin Faul
 * 
 * Created      : 05/09/2016
 * Description  : Header file for controlPanel.cpp
 * 
 * Version      : 1.0 - bfaul 5/9/16
 *                initial version
 */

#ifndef AR_DRONE_RVIZ_CONTROLS_H_
#define AR_DRONE_RVIZ_CONTROLS_H_

#include <ros/ros.h>
#include <rviz/config.h>
#include <rviz/display.h>
#include <rviz/display_group.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/properties/property.h>

#include <std_msgs/String.h>
//#include <crosbot/handle.hpp>
//#include <crosbot/controls/command.hpp>
//#include <crosbot/controls/status.hpp>

#include <QtGui/QGroupBox>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace drone {

class ControlsWidget : public QWidget {
Q_OBJECT
public:
    ControlsWidget(rviz::Property* propertyParent);
    ~ControlsWidget() {};

    void start();
    void stop();

    // Save in RViz Config
    void loadRVizConfig(const rviz::Config& config);
    void saveRVizConfig(rviz::Config config) const;

    // Subscriber callbacks
    virtual void callback_receivedStatus(const std_msgs::StringConstPtr status);
    virtual void callback_receivedCommand(const std_msgs::StringConstPtr command);

public Q_SLOTS:
    void slot_btn_reset();
    void slot_btn_start();
    void slot_btn_stop();
    void slot_btn_mode();
    void slot_btn_cam();

Q_SIGNALS:
    //void signal_update_data();

private:
    // Widget active
    bool operating;

    // ROS elements
    ros::NodeHandle nh;
    ros::Subscriber statusSub;
    ros::Publisher commandPub;


    // RViz Properties
    rviz::Property* propertyParent;

    // Status
    QLabel *commandLabel;
    QLabel *statusLabel;

    // Control elements
    QPushButton *resetBtn;
    QPushButton *startBtn;
    QPushButton *stopBtn;
    QPushButton *modeBtn;
    QPushButton *cameraBtn;
};

class ControlsRVizDisplay : public rviz::Display {
Q_OBJECT
public:
    ControlsRVizDisplay();
    ~ControlsRVizDisplay();

    // Over-ridden methods
    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

protected:
   virtual void onInitialize();
   virtual void reset();
   virtual void onDisable();
   virtual void onEnable();

private:
    // Widget rendering the controls
    ControlsWidget* controlsWidget;
    rviz::PanelDockWidget* controlsWidgetContainer;
};

}

#endif
