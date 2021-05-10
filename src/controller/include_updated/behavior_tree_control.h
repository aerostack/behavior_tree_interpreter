/*!********************************************************************************
 * \brief     This is the header of the behavior_tree_control class 
 * \authors   Abraham Carrera, Daniel Del Olmo,Oscar Cabrera
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/
#ifndef BEHAVIOR_TREE_CONTROL_H
#define BEHAVIOR_TREE_CONTROL_H
#define CTE_POSE (1.00)
 
#include <ros/ros.h>
#include <aerostack_msgs/RequestBehaviorActivation.h>
#include <aerostack_msgs/BehaviorCommandPriority.h>
#include <droneMsgsROS/ListOfBehaviors.h>
#include <behavior_coordinator_msgs/ListOfRunningTasks.h>
#include <behavior_coordinator_msgs/TaskCommand.h>
#include <behavior_coordinator_msgs/StartTask.h>
#include <behavior_coordinator_msgs/StopTask.h>
#include "droneMsgsROS/battery.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/PoseStamped.h>

#include <QWidget>
#include <QTime>
#include <QTimer>
#include <QObject>
#include <QTextEdit> 
#include <QLabel>
#include <QCheckBox>
#include <QGridLayout>
#include <QString>
#include <QMessageBox>
#include <QSplitter>
#include <QDir>
#include <string>

#include <thread>
#include <future>
#include <chrono>

#include <iostream>
#include <dirent.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include "yaml-cpp/yaml.h"



#include <mutex>
#include "behavior_tree.h"
#include "tree_node.h"
#include "behavior_tree_control_view.h"

#include "ui_behavior_tree_control.h"

#include <aerostack_msgs/CheckBehaviorFormat.h>
#include "file_manager.h"


namespace Ui {
  class BehaviorTreeControl;
}

class BehaviorTreeControlView;

class BehaviorTreeControl : public QWidget
{
  Q_OBJECT

  public:
    std::thread* executing_tree;
    
    bool teleoperationActivated;
    void closeDialog();
    //Constructor & Destructor
    explicit BehaviorTreeControl(QWidget *parent);
    ~BehaviorTreeControl();
    QCheckBox* expand_text_button;
    bool correct_format;
    bool hider;
    bool hider3;
    bool pause_continue_changer;
    void changeVisual();
    static std::mutex mutexVisual;
    //this is for the teleoperation
    geometry_msgs::PoseStamped motion_reference_pose_msg; 
    ros::Publisher pose_reference_publ;
    std::string pose_ref_topic_name;

    /*!********************************************************************************************************************
    *  \brief      This method returns the current variables' textbox content
    **********************************************************************************************************************/
    std::string getText();

    /*!********************************************************************************************************************
    *  \brief      This method copies the given text to the private variable 'text' which conforms the variables' textbox content
    **********************************************************************************************************************/
    void setText(std::string texto);

    /*!********************************************************************************************************************
    *  \brief   This method launch a window which contains detailed error
    **********************************************************************************************************************/
    void windowManager(char type, std::string title, std::string message);
        
    void executeTreeFromItem(BT::TreeNode * node);
    
    void deletePauseNode();
    void deleteRunningNodes();
    void resetTreeColor(BT::TreeNode* node);
    void resetTreeStatus(BT::TreeNode* node);
    bool setAbortedItems(BT::TreeNode* node);
    void cleanVariableText();
    
  private:
    Ui::BehaviorTreeControl* ui;
    ros::ServiceClient stop_task_srv;
    ros::ServiceClient activate_task_srv;
    ros::ServiceClient check_behavior_format_srv;
    ros::NodeHandle n;
    QKeyEvent *lastEvent;
    ros::Subscriber list_of_behaviors_sub;
    ros::Subscriber behavior_event_sub;
    ros::Subscriber battery_subs;
    //ros::Subscriber wificonnection_subs;
    ros::Publisher mission_state_publ;
    ros::Publisher behavior_command_publ;

    std_msgs::Bool missionStateMsgs;
    aerostack_msgs::RequestBehaviorActivation::Request req;

    aerostack_msgs::RequestBehaviorActivation::Response res;
    aerostack_msgs::BehaviorCommandPriority behavior_msg;
    //aerostack_msgs::CheckBehaviorFormat::Request check_format_msg_req;
    //aerostack_msgs::CheckBehaviorFormat::Response check_format_msg_res;
    //droneMsgsROS::battery battery_msgs;
    std::string task_stopped;
    ros::Subscriber tasks_ended;

    sensor_msgs::BatteryState battery_msgs;
    BT::BehaviorTree* tree;
    BehaviorTreeControlView* behavior_viewer;
    BT::TreeNode* root_node;
    

    QTimer* flight_timer; //Timer that sends the timeout signal every second.
    QTime* current_time;
    QString text;
    QLabel* tree_label;
    QLabel* beliefs_label;
    QTextEdit* beliefs_text;

    QMap<int, bool> acceptedKeys;
    QMessageBox error_message;
    QMessageBox* msg_error;
    QAbstractButton* m_save_button;
    QAbstractButton* m_cancel_button;
    QAbstractButton* m_dont_save_button;

    std::string check_behavior_format;
    std::string activate_behavior;
    std::string drone_id_namespace;
    std::string list_of_running_tasks;
    std::string behavior_event;
    std::string drone_driver_sensor_battery;
    //std::string wifi_connection_topic;
    std::string behavior_tree_execute_str;
    std::string behavior_tree_execute;
    std::string mission_configuration_folder;
    std::string folder_name;
    std::string homePath;
    std::string default_folder;
    std::string file_route;
    std::string behavior_command;
    std::string mission_state_topic;
    //std::ifstream aux_file;
    std::string error_behavior;
    std::string my_stack_directory;

    std::string start_task;

    int d_interval;
    int d_timerId;
    bool is_takenOff;
    //bool is_wifi_connected;
    bool isAKeyPressed;
    
    /* This method is called when a task is finished and it sets the 
        * status of the corresponding node to SUCCESSFULL_COMPLETION
        * and delete it from the vector of running_nodes
        */
    void CallbackBehaviorActivationFinished(const behavior_coordinator_msgs::TaskStopped &msg);

    /*!********************************************************************************************************************
    *  \brief      This method resets the behavior tree in order to repeat the mission
    **********************************************************************************************************************/
    void cleanTree();

    /*!********************************************************************************************************************
    *  \brief      This method returns the BehaviorTree
    **********************************************************************************************************************/
    //BT::BehaviorTree* getBehaviorTree();

    /*!********************************************************************************************************************
    *  \brief     This method initializes the timer that informs about the time the drone has been flying.
    *  \param ms  The interval at which the timer works.
    *********************************************************************************************************************/
    void setTimerInterval(double ms);

    //bool checkParameters(std::string task,std::string parameters){


    /*!********************************************************************************************************************
    *  \brief      This method is the responsible for seting up connections.
    *********************************************************************************************************************/
    void setUp();

    void setKeyboard();

    /*!********************************************************************************************************************
    *  \brief      This method is executed when the user wants to load a tree from a file.
    **********************************************************************************************************************/
    void loadTree();

    //void behaviorEventCallBack(const aerostack_msgs::BehaviorEvent & msg);

    /*!**********************************************************************************************************************
    *  \brief     This callback is executed when the list of behaviors is modified.
    *  \details   It's purpose is to control the state of the drone and the actions the GUI should allow the user to execute.
    *************************************************************************************************************************/
    void newBehaviorCallback(const behavior_coordinator_msgs::ListOfRunningTasks & msg);

    /*!************************************************************************
    *  \brief     Receives the battery status.
    ***************************************************************************/
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

    /*!************************************************************************
    *  \brief     Receives the state of the WiFi connection
    **************************************************************************/
    //void wifiConnectionCheckCallback(const std_msgs::Bool::ConstPtr& msg);

    /*!********************************************************************************************************************
    *  \brief      This method takes action when the user wants to make the drone to take off.
    *********************************************************************************************************************/
    void onTakeOffButton();

    /*!********************************************************************************************************************
    *  \brief      This method takes action when the user wants to make the drone to land.
    *********************************************************************************************************************/
    void onLandButton();

    /*!********************************************************************************************************************
    *  \brief      This method takes action when the user wants to reset the drone.
    *  \details    Resets angles (yaw).
    *********************************************************************************************************************/
    void onResetCommandButton();

    void keyPressEvent(QKeyEvent* e);
    void keyReleaseEvent(QKeyEvent* e);

    void addTextVariables();



  public Q_SLOTS:

    void setVariableText();

    /*!********************************************************************************************************************
    *  \brief     This method takes action when the user wants to make the drone to land.
    *********************************************************************************************************************/
    void landTreeMission();
    void expandVariable(int);
    void pauseMission();
    void waitTimeForStart();
    void completedMission();
    void missionFailed();

    /*!********************************************************************************************************************
    *  \brief      This method starts a behavior tree execution
    *********************************************************************************************************************/
    void executeTreeMission();
    void enableManualControl();

    /*!********************************************************************************************************************
    *  \brief      This method cancels the current behavior tree execution
    *********************************************************************************************************************/
    void abortTreeMission();


    /*!********************************************************************************************************************
    *  \brief      This method informs about the time the drone has been flying.
    *********************************************************************************************************************/
    void setFlightTime();

    /*!********************************************************************************************************************
    *  \brief      This slot is executed when a tree is executed. It disables the variables textbox editing.
    **********************************************************************************************************************/
    void setStartBlockingTextInput();

    /*!********************************************************************************************************************
    *  \brief      This slot is executed when a tree's execution is finished or canceled. It enables the variables textbox editing.
    **********************************************************************************************************************/
    void setStopBlockingTextInput();
    
    /*!********************************************************************************************************************
    *  \brief   Emitted when the tree's visual representation needs to be refreshed
    **********************************************************************************************************************/
    void update();

    //std::string outsideProcessData(std::string raw_arguments);

    /*!********************************************************************************************************************
    *  \brief      This method processes the arguments of a node and replaces the variables with the correct argument if needed.
    **********************************************************************************************************************/
    //std::string processData(std::string raw_arguments);

    /*!********************************************************************************************************************
    *  \brief      This method helps the processQueryData method by processing the argument data differently depending on which type it is.
    **********************************************************************************************************************/
    //std::string processType(YAML::const_iterator it);

  Q_SIGNALS:
    void finished();
    void executionStarted();
  

};

#endif // BEHAVIOR_TREE_CONTROL_H