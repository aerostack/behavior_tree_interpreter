/*!********************************************************************************
 * \brief     This is the behavior_task class, this is a leaf node that
 *            specifies a particular task that Aerostack can execute with a robot
 *            behavior. 
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

#include "behavior_task.h"
#include "behavior_tree.h"

#include <string>

BT::BehaviorTask::BehaviorTask(std::string name, std::string task_name,std::string parameters,int priority,QPixmap icono_pixmap) : LeafNode::LeafNode(name,icono_pixmap)
{
  action=BEHAVIOR_TASK;

  this->task_name=task_name;
  this->task_parameters=parameters;
  this->task_priority=priority;

  setBehaviorType(task_name);
  setNodeAttributes("Parameters:[" + task_parameters + "] Priority:[" + std::to_string(task_priority) +"]" );
  
  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1"); 
  n.param<std::string>("start_task", task, "start_task");   

  
}
BT::BehaviorTask::~BehaviorTask(){}

BT::ReturnStatus BT::BehaviorTask::executeStep()
{
  
  if(status==NON_INITIATED || status==BT::FAILURE_COMPLETION) 
  {
    setColorBackground(COLOR_BLUE);
    setColor("#ffffff");
    itemPaused=this;
    /*If activation_result.ack=true the service was succcesfully launched but it is not completed
    * until we recieved the confirmation from the callback function
    */
    behavior_coordinator_msgs::StartTask::Response activation_result=activateBehaviorTask();
    if(activation_result.ack)
    {
      status=RUNNING;
    }     
    else
    {
      std::cout<< activation_result.error_message;
      setColor(COLOR_RED);
      setColorBackground("#ffffff");
      status=FAILURE_COMPLETION;
    }
  }
  return status;
}

behavior_coordinator_msgs::StartTask::Response BT::BehaviorTask::activateBehaviorTask()
{
  QMessageBox error_message;
  behavior_coordinator_msgs::TaskCommand behavior;
  /*We build de TaskCommand msg*/
  behavior.name=task_name;
  if(checkTaskVariables(task_parameters))
  {
    try
    { 
      behavior.parameters=substitutionVariables(task_parameters);
      for(int i=0; i<behavior.parameters.length();i++)
      {
        if(behavior.parameters[i]=='&')
        {
          behavior.parameters[i]='\n';
          std::string aux1=behavior.parameters.substr(0,i);
          std::string aux2=behavior.parameters.substr(i+2);
          behavior.parameters=aux1+aux2;
        }
      } 
    }
    catch(const std::exception& e)
    {
      std::cout << "There is no value for variable \n";
      error_message.setWindowTitle(QString::fromStdString("Loading variable value"));
      error_message.setText(QString::fromStdString("There is no value for a variable.\n"));
      error_message.exec();
      
      return res_activate;
    }
  }
  else
  {
    behavior.parameters=task_parameters;
    for(int i=0; i<behavior.parameters.length();i++)
    {
      if(behavior.parameters[i]=='&')
      {
        behavior.parameters[i]='\n';
        std::string aux1=behavior.parameters.substr(0,i);
        std::string aux2=behavior.parameters.substr(i+2);
        behavior.parameters=aux1+"\n"+aux2;
      }
    }
  }
  behavior.priority=task_priority;
  activate_task_srv=n.serviceClient<behavior_coordinator_msgs::StartTask>("/"+drone_id_namespace+"/"+task);
  /*We call the service start task and add this node to the vector of running nodes in order to
  * the callback function can track this node and change its status when the task ends  
  */
  req_activate.task = behavior;
  activate_task_srv.call(req_activate,res_activate);
  
  running_nodes.push_back(this);
  return res_activate;  
 }

std::string BT::BehaviorTask::getTaskName()
{
  return task_name;
}

bool BT::BehaviorTask::checkTaskVariables(std::string parameters)
{
  bool variables=false;
  for(int i=0 ; i<parameters.size() && !variables ; i++)
  {
    if(parameters[i]=='+')
    {
      variables=true;
    }
  }
  return variables;
}
std::string BT::BehaviorTask::substitutionVariables(std::string parameters)
{
  std::string substitution=parameters;
  std::string aux1,aux2;
  for(int i=0 ; i<substitution.size()  ; i++)
  {
    if(substitution[i]=='+')
    {
      substitution[i]=' ';
      std::string value=variables->getValue(substitution[i+1]);
      substitution[i+1]=' ';
      
      aux1=substitution.substr(0,i);
      aux2=substitution.substr(i+2);
      aux1=aux1+value+aux2;
      substitution=aux1;
    }
  }
  return substitution;
}