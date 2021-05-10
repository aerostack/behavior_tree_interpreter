/*!********************************************************************************
 * \brief     This is the header of the behavior_tree_control_view class 
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
#ifndef BEHAVIOR_TREE_CONTROL_VIEW_H
#define BEHAVIOR_TREE_CONTROL_VIEW_H

#include <ros/ros.h>
#include "std_msgs/Bool.h"

//#include <droneMsgsROS/openMissionFile.h>
#include <aerostack_msgs/ListOfBeliefs.h>
#include <aerostack_msgs/RequestBehaviorActivation.h>
#include <aerostack_msgs/BehaviorCommandPriority.h>
 
#include <QWidget>
#include <QRect>
#include <QGuiApplication>
#include <QScreen>
#include <QProcess>
#include <QKeyEvent>
#include <QMap>
#include <QCloseEvent>

#include "behavior_tree_control.h"
#include "ui_behavior_tree_control_view.h"


#include <thread>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "fstream"

namespace Ui {
class BehaviorTreeControlView;
}

class BehaviorTreeControl;

class BehaviorTreeControlView : public QWidget
{
  Q_OBJECT

  public:
    //Constructor & Destructor
    explicit BehaviorTreeControlView(int argc, char** argv, QWidget *parent = 0);
    ~BehaviorTreeControlView();

    /*!********************************************************************************************************************
     *  \brief      This method returns the behavior tree control
     *********************************************************************************************************************/
    BehaviorTreeControl* getBehaviorTreeControl();

  private:
    Ui::BehaviorTreeControlView *ui;

    ros::NodeHandle n;
    ros::ServiceClient activate_task_srv;

    ros::Publisher window_event_pub;
    ros::Subscriber window_event_sub;

    std::string window_event_topic; 
    std::string start_task;

    BehaviorTreeControl *behavior_tree_control;
  
    std::string drone_id_namespace;
    std::string activate_behavior;

    boost::property_tree::ptree root;

    /*!************************************************************************
     *  \brief  Kills the process
     ***************************************************************************/
    void killMe();
  
    /*!********************************************************************************************************************
    *  \brief      This method is the responsible for seting up connections.
    *********************************************************************************************************************/
    void setUp();

    void setWidgetDimensions();
    int heightV= 790;
    int widthV=500;
    int position_x=-845;
    int position_y=-395;
};

#endif // BEHAVIOR_TREE_CONTROL_VIEW_H
