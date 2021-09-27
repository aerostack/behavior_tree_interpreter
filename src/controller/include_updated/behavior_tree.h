/*!********************************************************************************
 * \brief     This is the header of the behavior_tree class 
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
#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H

#include <fallback_node.h>
#include <sequence_node.h>
#include <behavior_task.h>
#include <add_belief.h>
#include <query_belief.h>
#include <remove_belief.h>
#include "tree_node.h"
#include <string>
#include <map>
#include <behavior_coordinator_msgs/TaskStopped.h>
#include <behavior_coordinator_msgs/StartTask.h>
#include <thread>
#include <chrono>
#include <future>

#include <QHeaderView>
#include <QTreeWidget>
#include <QWidget>
#include <QTreeWidgetItem>
#include <QMouseEvent>
#include <QString>
#include <QStringList>
#include <QPoint>
#include <QMenu>
#include <QIcon>
#include <QPixmap>
#include <QBrush>
#include <QColor>
#include <QTextEdit>
#include <QCloseEvent>
#include <QMessageBox>
#include <QErrorMessage>
#include <QApplication>
#include "behavior_tree_control.h"
#include "global.h"


class BehaviorTreeControl;
namespace BT
{
  class BehaviorTree : public QTreeWidget
  {
    Q_OBJECT

    private:
      
      ros::NodeHandle nh;
      
      behavior_coordinator_msgs::StartTask::Response res_activate;
      behavior_coordinator_msgs::StartTask::Request req_activate;

      std::string drone_id_namespace;
      
      BehaviorTreeControl * control_parent;
      bool running = false;
      bool isTextExpanded;
      bool is_menu_created = false;
      BT::TreeNode* root_before;
      BT::TreeNode* root_item;

      BT::TreeNode* last_executed;

      std::string start_task;
      ros::ServiceClient activate_task_srv;

      QMenu* contextMenu;
      QPoint point;
    
    protected:
      void expandText(BT::TreeNode* item);
      void minimizeText(BT::TreeNode* item);

    public:
      bool has_root;
      std::string all_beliefs;

      BehaviorTree(QWidget* parent = 0);
      ~BehaviorTree();
      /*This method executes the root and the rest of the nodes*/
      void execute(BT::TreeNode* root, int TickPeriod_milliseconds);

      void createMissionByTreeItem(BT::TreeNode * root);
      void addTopLevelItem(BT::TreeNode *top_level_item);

      BehaviorTreeControl* getVisualizer();

      bool isRunning();
      void setRunning(bool value);
    
      BT::TreeNode* getLastExecuted();
      void setLastExecuted(BT::TreeNode* node);
      
    public Q_SLOTS:
      void setStyleTreeSheet();
      void expandTreeText(int);
      void setUp();
      //right click functionalities
      void onCustomContextMenu(const QPoint &);
      void terminationMessage();
      void cancelTree();
      void executeTreeFromItem();
  /*!********************************************************************************************************************
  *  \brief      This method active the custom context menu
  **********************************************************************************************************************/
  void connectCustomContextMenu();

  /*!********************************************************************************************************************
  *  \brief      This method disable the custom context menu
  **********************************************************************************************************************/
  void disconnectCustomContextMenu();

    Q_SIGNALS:
      void executionStarted();
      void cancelExecutionSignal();
      void missionFinishedCorrectly();
      void missionFinishedFailure();
      void updateText();

  };
}
#endif