/*
 * File         : controlPanel.cpp
 * Author       : Benjamin Faul
 *  
 * Created      : 05/09/2016
 * Description  : AR Drone Project control panel. Based heavily on the
 * 		  control panel provided in assignment 1.
 *
 *  Version      : 1.0 - bfaul 5/9/16
 *		   initial version.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <common.hpp>
#include <rviz/controlPanel.hpp>

#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>

#include <typeinfo>
#include <QtCore/QVariant>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QVBoxLayout>

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <utility>

#define COMMAND_LABEL           "Command: "
#define LOG_START               "ControlsWidget ::"
#define STATUS_LABEL            "Status: "

#define ADD_BUTTON(VAR, TEXT, LYT, ROW, COL, BSLOT)             \
        VAR = new QPushButton(TEXT);                            \
        VAR->setText(TEXT);                                     \
        VAR->setEnabled(true);                                  \
        LYT->addWidget(VAR, ROW, COL);                          \
        connect(VAR, SIGNAL(clicked()), this, SLOT(BSLOT()));

namespace drone {

ControlsWidget::ControlsWidget(rviz::Property* propertyParent) :
    QWidget(),
    operating(false),
    propertyParent(propertyParent)
{
    // Button Layout
    QVBoxLayout *vlayout = new QVBoxLayout(this);

    // Create  buttons
    QGroupBox *controlsGB = new QGroupBox("AR Drone Controls");
    QGridLayout *buttonLayout = new QGridLayout();
    ADD_BUTTON(startBtn, "Take Off", buttonLayout, 0, 0, slot_btn_start);
    ADD_BUTTON(stopBtn, "Land", buttonLayout, 0, 1, slot_btn_stop);
    ADD_BUTTON(resetBtn, "Emergency Stop", buttonLayout, 1, 0, slot_btn_reset);
    ADD_BUTTON(modeBtn, "Beacon Track", buttonLayout, 1, 1, slot_btn_mode);
    ADD_BUTTON(cameraBtn, "Toggle Camera", buttonLayout, 2, 0, slot_btn_cam);
    controlsGB->setLayout(buttonLayout);

    // Configure Labels
    QVBoxLayout *labelsLayout = new QVBoxLayout();
    commandLabel = new QLabel(COMMAND_LABEL, this);
    statusLabel = new QLabel(STATUS_LABEL, this);
    commandLabel->setWordWrap(true);
    statusLabel->setWordWrap(true);
    statusLabel->setText(STATUS_LABEL);
    labelsLayout->addWidget(commandLabel);
    labelsLayout->addWidget(statusLabel);

    // HRules
    QFrame *line = new QFrame(this);
    line->setGeometry(QRect(0, 0, 10, 3));
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);

    // Set layout
    vlayout->addWidget(controlsGB);
    vlayout->addWidget(line);
    vlayout->addItem(labelsLayout);
    setLayout(vlayout);

    commandPub = nh.advertise<std_msgs::String>("cmd", 1, false);
}

void ControlsWidget::start() {
    ROS_INFO("ControlsWidget::start");
    //crosbotCommand = new crosbot::CrosbotCommand(TURTLEBOT_CONTROL_NAMESPACE,
    //        true,
    //        new _RVIZCommandCallback(this));
    //crosbotStatus = new crosbot::CrosbotStatus("", new _RVIZStatusCallback(this));

    operating = true;
}

void ControlsWidget::stop() {
    ROS_INFO("ControlsWidget::stop");
    operating = false;

    //crosbotCommand = NULL;
    //crosbotStatus = NULL;
}

void ControlsWidget::loadRVizConfig(const rviz::Config& config) {
}

void ControlsWidget::saveRVizConfig(rviz::Config config) const {
}

void ControlsWidget::callback_receivedStatus(const std_msgs::StringConstPtr status) {
    ROS_INFO("ControlWidget::callback_receivedStatus");
     if (operating) {
	ROS_INFO("Status: %s", status->data.c_str());
	/*
        // Set status
        std::string label = STATUS_LABEL;
        if (!status->stats_namespace.empty()) {
            label += "(" + status->stats_namespace + ")";
        }
        label += status->status;
        statusLabel->setText(label.c_str());

        // Set colour
        std::string level = status->level;
        QColor colour;
        if (level == crosbot_msgs::ControlStatus::LEVEL_INFO) {
            colour = Qt::black;
        } else if (level == crosbot_msgs::ControlStatus::LEVEL_WARNING) {
            colour = Qt::yellow;
        } else if (level == crosbot_msgs::ControlStatus::LEVEL_ERROR) {
            colour = Qt::red;
        }
        QPalette palette = statusLabel->palette();
        palette.setColor(statusLabel->foregroundRole(), colour);
        statusLabel->setPalette(palette);

        statusLabel->update();
	*/
    }
}

void ControlsWidget::callback_receivedCommand(const std_msgs::StringConstPtr command) {
    ROS_INFO("ControlsWidget::callback_receivedCommand");
    if (operating) {
	/*
        std::string label = COMMAND_LABEL + command->command;
        commandLabel->setText(label.c_str());
        commandLabel->update();
	*/
    }
}

void ControlsWidget::slot_btn_reset() {
    if (operating) {
        ROS_INFO("%s send emergency_stop", LOG_START);
	std_msgs::String msg;
	msg.data = "emergency_stop";
	commandPub.publish(msg);
        //crosbotCommand->sendCommandAll(crosbot_msgs::ControlCommand::CMD_RESET);
    }
}

void ControlsWidget::slot_btn_start() {
    if (operating) {
        ROS_INFO("%s send take_off", LOG_START);
        std_msgs::String msg;
	msg.data = "take_off";
	commandPub.publish(msg);
        //crosbotCommand->sendCommandAll(crosbot_msgs::ControlCommand::CMD_START);
    }
}

void ControlsWidget::slot_btn_stop() {
    if (operating) {
        ROS_INFO("%s send land", LOG_START);
        std_msgs::String msg;
	msg.data = "land";
	commandPub.publish(msg);
	//crosbotCommand->sendCommandAll(crosbot_msgs::ControlCommand::CMD_STOP);
    }
}

void ControlsWidget::slot_btn_mode() {
    if (operating) {
        ROS_INFO("%s toggle mode", LOG_START);
        std_msgs::String msg;
        msg.data = "mode_toggle";
        commandPub.publish(msg);
    }
}

void ControlsWidget::slot_btn_cam() {
    if (operating) {
        ROS_INFO("%s toggle camera", LOG_START);
        std_msgs::String msg;
        msg.data = "camera_toggle";
        commandPub.publish(msg);
    }
}

ControlsRVizDisplay::ControlsRVizDisplay() :
    controlsWidget(NULL),
    controlsWidgetContainer(NULL)
{
    controlsWidget = new ControlsWidget(NULL);
    //commandPub = nh.advertise<std_msgs::String>("cmd", 1, false);
}

ControlsRVizDisplay::~ControlsRVizDisplay() {
    if (controlsWidget != NULL) {
        ROS_INFO("ControlsRVizDisplay::~ControlsRVizDisplay");
        controlsWidget->stop();
        if (controlsWidgetContainer) {
            delete controlsWidgetContainer;
        } else {
            delete controlsWidget;
        }
    }
}

void ControlsRVizDisplay::load(const rviz::Config& config) {
    // Do Normal rviz::Display save
    rviz::Display::load(config);

    // Widgets load
    if (controlsWidget != NULL) {
        controlsWidget->loadRVizConfig(config);
    }
}

void ControlsRVizDisplay::save(rviz::Config config) const {
    // Do Normal rviz::Display save
    rviz::Display::save(config);

    // Widgets save
    if (controlsWidget != NULL) {
        controlsWidget->saveRVizConfig(config);
    }
}

void ControlsRVizDisplay::onInitialize() {
    // Connect widget to window manager and start
    ROS_INFO("ControlsRVizDisplay::onInitialize");
    rviz::WindowManagerInterface* wm = context_->getWindowManager();
    if (wm && controlsWidget != NULL) {
        // Controls widget
        controlsWidgetContainer = wm->addPane("Controls", controlsWidget);
    }
}

void ControlsRVizDisplay::reset() {
    ROS_INFO("ControlsRVizDisplay::reset");
    if (controlsWidget != NULL) {
        controlsWidget->stop();
        controlsWidget->start();
    }
}

void ControlsRVizDisplay::onDisable() {
    ROS_INFO("ControlsRVizDisplay::onDisable");
    if (controlsWidget != NULL) {
        controlsWidget->stop();
    }
}

void ControlsRVizDisplay::onEnable() {
    ROS_INFO("ControlsRVizDisplay::onEnable");
    if (controlsWidget != NULL) {
        controlsWidget->start();
    }
}

}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(drone::ControlsRVizDisplay, rviz::Display)

