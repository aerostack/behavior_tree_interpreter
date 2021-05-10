/*!********************************************************************************
 * \brief     This is the header of the behavior_task class 
 * \authors   Oscar Cabrera
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
#ifndef BEHAVIOR_TASK_H
#define BEHAVIOR_TASK_H

#include <leaf_node.h>
#include <string>
#include<ros/ros.h>
#include <behavior_coordinator_msgs/StartTask.h>
#include <behavior_coordinator_msgs/TaskCommand.h>
#include <QMessageBox>

namespace BT
{
  class BehaviorTree;
  class BehaviorTask : public LeafNode
  {
    private:
      std::string task_name;
      std::string task_parameters;
      int task_priority;
      //We need to declare a BehaviorTree in order to add the running nodes
      BT::BehaviorTree * bt;
      
      ros::ServiceClient activate_task_srv;
      behavior_coordinator_msgs::StartTask::Response res_activate;
      behavior_coordinator_msgs::StartTask::Request req_activate;
      std::string task;
      

    public:
      explicit BehaviorTask(std::string name,std::string task_name,std::string task_parameters,int task_priority,QPixmap icono_pixmap);
      ~BehaviorTask();
      
      BT::ReturnStatus executeStep();

      //this function starts a task and returns if the task was succesfully started and an error msg
      behavior_coordinator_msgs::StartTask::Response activateBehaviorTask();
      
      std::string getTaskName();
      bool checkTaskVariables(std::string parameters);
      std::string substitutionVariables(std::string parameters);

  };
}
#endif