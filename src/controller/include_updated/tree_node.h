/*!********************************************************************************
 * \brief     This is the header of the tree_node class 
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
#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <iostream>
#include <unistd.h>
#include <string>
#include <memory>
#include <variables.h>

#include <QWidget>
#include <QTreeWidget>
#include <QPoint>
#include <iostream>
#include <QString>
#include <QStringList>
#include <unistd.h>
#include <QIcon>
#include <QBrush>
#include <QColor>
#include <QPixmap>
#include <QColor>
#include <QBrush>
#include <QPropertyAnimation>

static const std::string COLOR_BLUE = "#74b6f7";
static const std::string COLOR_GREEN = "#006400";
static const std::string COLOR_RED = "#c41306";
static const std::string COLOR_BLACK = "#000000";
static const std::string COLOR_PURPLE = "#ce42f5";
static const std::string COLOR_CIAN = "#02E5E7";
static const std::string COLOR_GRAY = "#808080";
static const std::string SUBSTITUTION_S = "+";

namespace BT
{
  /*!
  * \enum NodeType
  * We have action or control nodes. Action nodes perform tasks and control nodes control the execution 
  * of the tree based on the return status of the control nodes.
  */
  enum NodeType {ACTION_NODE, CONTROL_NODE};
  /*!
  * \enum ReturnStatus
  * We have five different return status.  
  * "RUNNING" indicates that the task is not finished yet.
  * "SUCCESSFUL_COMPLETION" indicates that the task has been completed.
  * "FAILURE_COMPLETION" indicates that the task couldn't be completed.
  * "NON_INITIATED" indicates that the node hasn run yet.
  * "PAUSED" indicates that the node has been paused. 
  */
  enum ReturnStatus {RUNNING, SUCCESSFUL_COMPLETION, FAILURE_COMPLETION, NON_INITIATED, PAUSED, ABORTED};

  /*!
  * \enum NodeAction
  * We have six differente type of nodes.  
  * "ADD_BELIEF" it's an action_node that adds a belief to the memory of beliefs.
  * "REMOVE_BELIEF" it's an action_node that removes a belief to the memory of beliefs.
  * "QUERY_BELIEF" it's an action_node that asks a query to the memory of beliefs.
  * "BEHAVIOR_TASK" it's an action node that starts a task.
  * "SEQUENCE_NODE" it's a control node.
  * "FALLBACK_NODE" it's a control node. 
  */
  enum NodeAction {ADD_BELIEF,REMOVE_BELIEF,QUERY_BELIEF,BEHAVIOR_TASK,SEQUENCE_NODE,FALLBACK_NODE};

  /*! \class tree_node */
  class TreeNode : public QTreeWidgetItem
  {
    private:
      int id;
      std::string name;
      //partial_node_name is an explanation of the type of the node
      std::string partial_node_name;
      bool has_parent;
      std::string behavior_type;
      std::string parameters;
      bool isAborted;
      
    protected:
      ReturnStatus status;
      NodeType type;
      NodeAction action;

      std::vector<TreeNode*> children;
      unsigned int number_of_children;

    public:
      explicit TreeNode(std::string name, QPixmap icono_pixmap);
      ~TreeNode();

      virtual BT::ReturnStatus executeStep() = 0;    

      ReturnStatus getStatus();
      void setStatus(ReturnStatus new_status);

      std::string getName();
      void setName(std::string new_name);
      
      NodeType getType();
      void setType(NodeType type);
      
      NodeAction getAction();
      void setAction(NodeAction action);

      bool getHasParent();
      void setHasParent(bool value);

      bool getisAborted();
      void setisAborted(bool value);
      
      std::string getBehaviorType();
      void setBehaviorType(std::string btype);
      
      std::string getNodeAttributes();
      void setNodeAttributes(std::string attr);

      std::string getPartialNodeName();
      void setPartialNodeName(std::string text);

      std::string nodeTypeToString(NodeType node);
      NodeType stringToNodeType(std::string str);
      
      std::string nodeActionToString(NodeAction node);
      NodeAction stringToNodeAction(std::string str);

      std::string statusToString(ReturnStatus status);

      BT::TreeNode* modifyNode(std::string node_name, BT::NodeType node_type, BT::NodeAction ACTION_NODE, 
        std::string behavior_type, std::string arguments, bool multivalued);  
    
      void addChild(TreeNode* child);
      unsigned int GetChildrenNumber();

      std::vector<TreeNode*> getChildren();
      //this function receives a string with the task parameters+priority and returns them in a map 
      std::pair<std::string,int> getTaskParameters(std::string arguments);

      void setColor(std::string color);

      void setColorBackground(std::string color);

      bool itemAborted=false;

  };
  
}
#endif