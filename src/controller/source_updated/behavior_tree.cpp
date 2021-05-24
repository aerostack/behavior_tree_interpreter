/*!********************************************************************************
 * \brief     This is the behavior_tree class, this class is responsable of executing
 *            the nodes. 
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

#include<behavior_tree.h>
#include <ros/ros.h>

BT::BehaviorTree::BehaviorTree(QWidget *parent) : QTreeWidget(parent)
{
  //window always on top
  QWidget::setLocale(QLocale());
  this->headerItem()->setHidden(true);
  this->setContextMenuPolicy(Qt::CustomContextMenu);
  this->resize(600, 300);
  this->setFocusPolicy(Qt::FocusPolicy::NoFocus);
  this->header()->setStretchLastSection(false);
  this->header()->setSectionResizeMode(QHeaderView::ResizeToContents);

  qRegisterMetaType<QVector<int>>("QVector<int>");
  control_parent = (BehaviorTreeControl*) parent;

  has_root = false;
  running = false;
  isTextExpanded = true;
    
  QObject::connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onCustomContextMenu(const QPoint &)));

  setStyleTreeSheet();

  nh.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
  //nh.param<std::string>("task_stopped", task_stopped, "task_stopped");
  //tasks_ended=nh.subscribe("/" + drone_id_namespace +  "/" + task_stopped, 1000, &BehaviorTree::CallbackBehaviorActivationFinished, this);

  setUp();
  
}

BT::BehaviorTree::~BehaviorTree(){}


void BT::BehaviorTree::setStyleTreeSheet()
{
	setIconSize(QSize(25, 25));
	this->setStyleSheet(" \
		QTreeView::branch { \
			background: white; \
		} \
		QTreeView::branch:has-siblings:!adjoins-item { \
			border-image: url(:/images/images/vline.png) 0; \
		} \
		QTreeView::branch:has-siblings:adjoins-item { \
			border-image: url(:/images/images/branch-more.png) 0; \
		} \
		QTreeView::branch:!has-children:!has-siblings:adjoins-item { \
			border-image: url(:/images/images/branch-end.png) 0; \
		} \
		QTreeView::branch:has-children:!has-siblings:closed, \
		QTreeView::branch:closed:has-children:has-siblings { \
			border-image: url(:/images/images/branch-more_proto_mas.png) 0; \
		} \
		QTreeView::branch:has-children:!has-siblings:closed, \
		QTreeView::branch:closed:has-children:!has-siblings { \
			border-image: url(:/images/images/branch-end_proto_mas.png) 0; \
		} \
		QTreeView::branch:open:has-children:has-siblings  { \
			border-image: url(:/images/images/branch-more_proto_menos.png) 0; \
		} \
		QTreeView::branch:open:has-children:!has-siblings  { \
			border-image: url(:/images/images/branch-end_proto_menos.png) 0; \
		} \
		QTreeView::item:selected { \
			background-color:transparent; \
			color:black; \
		} \
		");
}

bool BT::BehaviorTree::isRunning()
{
  return running;
}
      
void BT::BehaviorTree::setRunning(bool value)
{
  this->running=value;
}

void BT::BehaviorTree::executeTreeFromItem()
{
  BT::TreeNode * item_to_execute=(BT::TreeNode*)itemAt(point);
  control_parent->executeTreeFromItem(item_to_execute);
}


void BT::BehaviorTree::execute(BT::TreeNode* root, int TickPeriod_milliseconds)
{   
  ros::Rate r(30); // 100 hz
  while (ros::ok() && !stopMission)
  {
   BT::ReturnStatus status;
   
   if(paused)
	{
	  std::cout<<"ejecutando item"<<itemPaused->getName()<<"\n";
	  status=itemPaused->executeStep();
	  paused=false;
	}
	else
	{
	  status=root->executeStep();
	}
	if(status==BT::SUCCESSFUL_COMPLETION)
	{
	  std::cout << "esta true y acaba \n";
	  stopMission=true;
	  Q_EMIT(missionFinishedCorrectly());
	}
	if(status==BT::FAILURE_COMPLETION)
	{
	  std::cout << "ha fallado \n";
	  stopMission=true;
	  Q_EMIT(missionFinishedFailure());
	}
	if(!variables->isEmpty() && new_variable)
	{
	  new_variable=false;
	  Q_EMIT(updateText());
	}
    ros::spinOnce();

   r.sleep();
  }
}


void BT::BehaviorTree::createMissionByTreeItem(BT::TreeNode * root)
{
  root_item = root;
  this->addTopLevelItem(root_item);
  if (root != 0) 
  {
    if (isTextExpanded) 
	{
	  expandTreeText(2);
	}
	else 
	{
	  expandTreeText(0);
	}
	this->expandAll();
  }
}

void BT::BehaviorTree::addTopLevelItem(BT::TreeNode *top_level_item)
{
  if (has_root) 
  {
	if (root_before != 0) 
	{
	  delete root_before;
	}
  }
  if (top_level_item != 0) 
  {
    root_before = top_level_item;
	has_root = true;
	top_level_item->setHasParent(false);
  }
  else 
  {
	has_root = false;
  }
  QTreeWidget::addTopLevelItem(top_level_item);
}

void BT::BehaviorTree::expandTreeText(int checkState)
{
  if (has_root) 
  {
	if(checkState == 2)
	{			
	  expandText(root_item);
	  isTextExpanded=true;
	}
	else if(checkState == 0)
	{
	  minimizeText(root_item);
	  isTextExpanded=false;
    }
  }
}

void BT::BehaviorTree::expandText(BT::TreeNode *item)
{   
  expandedText = true;
  std::string text = item->getName();
  text = text +  item->getPartialNodeName();
  item->setText(0, QString::fromStdString(text));
  std::vector<BT::TreeNode*> children=item->getChildren();
  if(item->GetChildrenNumber()>0)
	{
	  for(int i = 0; i < item->GetChildrenNumber(); i++)
	  {
        expandText(children.at(i));
	  }
	}
}

void BT::BehaviorTree::minimizeText(BT::TreeNode *item)
{
  expandedText = false;
  item->setText(0, QString::fromStdString(item->getName()));
  std::vector<BT::TreeNode*> children=item->getChildren();
  if(item->childCount()>0)
  {
    for(int i = 0; i < item->GetChildrenNumber(); i++)
	{
	  minimizeText(children.at(i));
	}
  }
}


void BT::BehaviorTree::setUp()
{
  ros::NodeHandle n("~");
  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
  n.param<std::string>("start_task", start_task, "start_task");
  activate_task_srv=n.serviceClient<behavior_coordinator_msgs::StartTask>("/"+drone_id_namespace+"/"+start_task);
  n.param<std::string>("all_beliefs", all_beliefs, "all_beliefs");
  
}

BehaviorTreeControl* BT::BehaviorTree::getVisualizer()
{
  return control_parent;
}


/*
This function is called whenever we do a right click
It creates the context menu and disables any functionality that cannot be executed. This depends on where we right clicked.
*/
void BT::BehaviorTree::onCustomContextMenu(const QPoint &p)
{
  if(itemAt(p)!=0)
  {
    BT::TreeNode *item_clicked;
	if(is_menu_created)
	{
	  contextMenu->close();
	  delete contextMenu;
	  is_menu_created = false;
	}
	contextMenu = new QMenu("Menu", this);
	is_menu_created = true;
	item_clicked = (BT::TreeNode*)itemAt(p);
	BT::TreeNode *item_clicked_parent;

	int children;
	//Check if the clicked node can have children
	if(item_clicked->getType()==BT::NodeType::CONTROL_NODE)
	{
	  children = item_clicked->GetChildrenNumber();
	}

	point = p;

	std::string termination_cause;
		
    termination_cause = item_clicked->getStatus();
	QAction action("Execute tree from this node", this);
	contextMenu->addAction(&action);
	connect(&action, SIGNAL(triggered()), this, SLOT(executeTreeFromItem()));
	QAction action2("View termination state", this);
                
	contextMenu->addAction(&action2);
	connect(&action2, SIGNAL(triggered()), this, SLOT(terminationMessage()));

	contextMenu->exec(mapToGlobal(p));
	}
	else if (itemAt(p)==0)
	{
	  point = p;
	  if(is_menu_created)
	  {
	    contextMenu->close();
		delete contextMenu;
		is_menu_created = false;
	  }
	is_menu_created = true;
	contextMenu = new QMenu("Menu", this);
	}
}

void BT::BehaviorTree::terminationMessage()
{
  BT::TreeNode * item_to_view = (BT::TreeNode*)itemAt(point);
  this->getVisualizer()->windowManager('e', "Termination state", item_to_view->statusToString(item_to_view->getStatus()) );
}

void BT::BehaviorTree::cancelTree()
{		
  behavior_coordinator_msgs::TaskCommand behavior;
  behavior.name = "HOVER";
  behavior.priority= 1;
  req_activate.task = behavior;
  running=false;
  activate_task_srv.call(req_activate,res_activate);

  if(!res_activate.ack)
	std::cout << res_activate.error_message << std::endl;

  //connectCustomContextMenu();
  if (lastVisualState==3)
  {
    running=false;
  }
  else
  {
    Q_EMIT(cancelExecutionSignal());
  }
}

BT::TreeNode* BT::BehaviorTree::getLastExecuted()
{
  return last_executed;
}

void BT::BehaviorTree::setLastExecuted(BT::TreeNode* node)
{
  this->last_executed=node;
}

void BT::BehaviorTree::connectCustomContextMenu()
{
  QObject::connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onCustomContextMenu(const QPoint &)));
}


void BT::BehaviorTree::disconnectCustomContextMenu()
{
  QObject::disconnect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(onCustomContextMenu(const QPoint &)));
}
