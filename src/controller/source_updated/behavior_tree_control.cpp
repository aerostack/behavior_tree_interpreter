/*!********************************************************************************
 * \brief     BehaviorTreeControl
 * \authors   Daniel Del Olmo, Jorge Luis Pascual, Carlos Valencia, Adrián Cabrera, Oscar Cabrera.
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

#include "behavior_tree_control.h"
#include <iostream>
#include <fstream>
#include <QtConcurrent>
#include <future>
#include "global.h"
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <thread>

std::mutex BehaviorTreeControl::mutexVisual;

BehaviorTreeControl::BehaviorTreeControl(QWidget *parent) :
QWidget(parent),
ui(new Ui::BehaviorTreeControl)
{
  //initialization of visual components of the GUI.
  visualState=0;//this is initialization to landed state.
  ui->setupUi(this); //connects all ui's triggers
  changeVisual();

  teleoperationActivated=false;
  hider=false;
  hider3=false;
  paused=false;
  stopMission=false;
  completed_mission=false;
  mission_failed=false;
  pause_continue_changer=false;
  variables=new Variables();
  new_variable=false;

  tree = new BT::BehaviorTree(this);
  expand_text_button = new QCheckBox("Show description", this);
  tree_label = new QLabel("Behavior Tree");
  QCheckBox * beliefCheck = new QCheckBox("Show variables",this);
  beliefCheck->setCheckState(Qt::Unchecked);
  beliefs_label = new QLabel("Variables");
  beliefs_text = new QTextEdit(this);
  behavior_viewer = (BehaviorTreeControlView*) parent;
  //Widgets
  QGridLayout * p = new QGridLayout();
  p->addWidget(tree_label, 0, 0, 1, 1);
  p->addWidget(expand_text_button, 0, 1, 1, 1,Qt::AlignLeft);
  p->addWidget(beliefCheck,0,2,1,1,Qt::AlignRight);
  this->QWidget::setWindowTitle(QString::fromStdString("Behavior Tree"));

  ui->groupBox_3->hide();
  ui->groupBox_4->hide(),
  ui->groupBox_5->hide();

  beliefs_text->hide();
  beliefs_label->hide();

  ui->label_wifi->hide();
  ui->value_wifi->hide();

  ui->gridLayout_6->addLayout(p, 0, 0, 1, 1,Qt::AlignTop);
  ui->gridLayout_6->addWidget(tree, 1, 0, 1, 1);
  ui->gridLayout_6->addWidget(beliefs_label, 3, 0, 1, 1);
  ui->gridLayout_6->addWidget(beliefs_text, 4, 0, 1, 1);
  is_takenOff = false;
  correct_format = false;
  beliefs_text->setMaximumHeight(80);
  beliefs_text->setText(QString::fromStdString(""));
  ui->value_battery->setText(QString::number(100) +  "%");
  //Set Flight time
  this->current_time = new QTime(0, 0, 0);
  setTimerInterval(1000);// 1 second = 1000
  flight_timer = new QTimer(this);
  flight_timer->start(1000);

  //Q_EMIT(expand_text_button->stateChanged(2));
  expand_text_button->setCheckState(Qt::Checked);
  ui->groupBox_3->setStyleSheet("QGroupBox{ border: 10px solid transparent;}");
  ui->groupBox_4->setStyleSheet("QGroupBox{ border: 10px solid transparent;}");
  ui->groupBox_5->setStyleSheet("QGroupBox{ border: 10px solid transparent;}");

  //Default configuration folder
  homePath = QDir::homePath().toStdString();

  //Establishment of connections
  setUp(); 

  //Settings keyboard control
  setKeyboard();

  //Load tree
  loadTree();

  //Connects
  
  QObject::connect(ui->execute_tree_button, SIGNAL(clicked()), this, SLOT(executeTreeMission()));
  QObject::connect(ui->abort_tree_button, SIGNAL(clicked()), this, SLOT(abortTreeMission()));
  QObject::connect(this, SIGNAL(executionStarted()), this, SLOT(setStartBlockingTextInput()));
  QObject::connect(ui->emergency_land_tree_button, SIGNAL(clicked()), this, SLOT(landTreeMission()));
  QObject::connect(ui->finish_teleoperation_button, SIGNAL(clicked()), this, SLOT(landTreeMission()));
  QObject::connect(ui->teleoperation_button,SIGNAL(clicked()), this,SLOT(enableManualControl())); 
  QObject::connect(ui->pause_mission, SIGNAL(clicked()), this, SLOT(pauseMission()));
  QObject::connect(tree,SIGNAL(missionFinishedCorrectly()),this,SLOT(completedMission()));
  QObject::connect(tree,SIGNAL(missionFinishedFailure()),this,SLOT(missionFailed()));
  QObject::connect(expand_text_button, SIGNAL(stateChanged(int)), tree, SLOT(expandTreeText(int)));
  QObject::connect(beliefCheck,SIGNAL(stateChanged(int)), this,SLOT(expandVariable(int)));
  QObject::connect(flight_timer, SIGNAL(timeout()), this, SLOT(setFlightTime()));
  QObject::connect(tree, SIGNAL(updateText()), this, SLOT(setVariableText()));
}

/*------------------------------------------------------------
--------------------------------------------------------------
                      Destructor
--------------------------------------------------------------  
------------------------------------------------------------*/
BehaviorTreeControl::~BehaviorTreeControl()
{
  delete ui;
  delete flight_timer;
  delete current_time;
  delete tree;
  delete expand_text_button;
  delete tree_label;
  delete beliefs_label;
  delete beliefs_text;
}

/*------------------------------------------------------------
--------------------------------------------------------------
                Getters and setters
--------------------------------------------------------------
------------------------------------------------------------*/

void BehaviorTreeControl::expandVariable(int state){//this method makes possible to expand the variables section
  if (state==0){
    beliefs_text->hide();
    beliefs_label->hide();
  }
  else{
    beliefs_text->show();
    beliefs_label->show();
  }

}

void BehaviorTreeControl::enableManualControl(){//this method enable to activate the manual control
  if(tree->isRunning())
    abortTreeMission();

  if (hider == true)
  {
    visualState=4;
    changeVisual();
    teleoperationActivated=false;
    hider=false;
  }
  else{
    teleoperationActivated=true;
    hider=true;
    if(visualState==2)
    {
      lastVisualState=visualState;
      visualState=5;
      changeVisual();
    }
    if(visualState==3)
    {
      lastVisualState=visualState;
      visualState=7;
      changeVisual();
    }
  }
}


std::string BehaviorTreeControl::getText()
{
  std::string result = this->beliefs_text->toPlainText().toUtf8().constData();
  return result;
}

void BehaviorTreeControl::setText(std::string texto)
{
  text = QString(texto.c_str());
}

void BehaviorTreeControl::setTimerInterval(double ms)
{
  d_interval = qRound(ms);
  if (d_interval >= 0 )
    d_timerId = startTimer(d_interval);
}

void BehaviorTreeControl::setFlightTime()
{
  if (tree->isRunning())
  {
    this->current_time->setHMS(this->current_time->addSecs(+1).hour(), this->current_time->addSecs(+1).minute(), this->current_time->addSecs(+1).second());
    ui->value_flight_time->setText(this->current_time->toString());
  }
}

void BehaviorTreeControl::setStartBlockingTextInput()//this is used at the beginning of the mission.
{
  visualState=1;
  changeVisual();
  pause_continue_changer= false;
  beliefs_text->setReadOnly(true);
}

void BehaviorTreeControl::setStopBlockingTextInput() //this is used at the end of the mission.
{ 
  if(visualState!=1)
  {
    if (visualState!=2 && visualState!=3 && visualState!=6){
    lastVisualState=9;
    visualState=9;
    changeVisual();
    lastVisualState=0;
    visualState=0;
    changeVisual();
    }
  }
  if (visualState==6 )
  {
    visualState=-1;
  }
  if(visualState!=3)
  {
    tree->connectCustomContextMenu();
  }
    
  beliefs_text->setReadOnly(false);

  if (visualState!=-1 )
  {
    missionStateMsgs.data=false;
    mission_state_publ.publish(missionStateMsgs);
  }
   
}
void BehaviorTreeControl::addTextVariables()
{
  std::string texto=getText();
  if(texto!="")
  {   
    std::string parsed;
    std::stringstream input_stringstream(texto);

    while(std::getline(input_stringstream,parsed,'\n'))
    {
      char nombre_var=parsed[0];
      std::string valor_var = parsed.substr(2, parsed.find("\n"));
      for(int i=0;i<valor_var.size();i++)
      {
        if(valor_var[i]=='.')
        {
          valor_var[i]=',';
        }
      }
      std::pair <char,double> variable (nombre_var,std::stod(valor_var));
      variables->insert(variable);
    }
  }
}
void BehaviorTreeControl::update()//this is an important method because it makes possible the process of using variables.
{
  beliefs_text->setText(text);
  processing_belief_query = false;
}
void BehaviorTreeControl::setVariableText()
{
  std::string new_text=variables->getValues();
  text = QString(new_text.c_str());
  beliefs_text->setText(text);
}
void BehaviorTreeControl::cleanVariableText()
{
  std::string new_text="";
  text = QString(new_text.c_str());
  beliefs_text->setText(text);
}
void BehaviorTreeControl::setUp()
{   
  ros::NodeHandle nh("~");
  //Nodes
  nh.param<std::string>("start_task", start_task, "start_task");
  nh.param<std::string>("list_of_running_task", list_of_running_tasks, "list_of_running_tasks");
  nh.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
  nh.param<std::string>("behavior_tree_execute", behavior_tree_execute, "behavior_tree_execute");
  nh.param<std::string>("mission_state", mission_state_topic, "mission_state");
  nh.param<std::string>("mission_configuration_folder", mission_configuration_folder, "$(env AEROSTACK_STACK)/configs/drone1");
  nh.param<std::string>("check_behavior_format", check_behavior_format, "check_behavior_format");
  nh.param<std::string>("task_stopped", task_stopped, "task_stopped");
  
  if (!nh.getParam("drone_driver_sensor_battery", drone_driver_sensor_battery))
    drone_driver_sensor_battery = "/sensor_measurement/battery_state";

  /*if (!nh.getParam("wifiIsOk", wifi_connection_topic))
  wifi_connection_topic = "wifiIsOk";*/
  ros::param::get("~pose_ref_topic_name", pose_ref_topic_name);
  if ( pose_ref_topic_name.length() == 0)
  {
    pose_ref_topic_name="motion_reference/pose";
  }

  //Service communications
  //check_behavior_format_srv=nh.serviceClient<aerostack_msgs::CheckBehaviorFormat>("/"+drone_id_namespace+"/"+check_behavior_format);
  activate_task_srv = nh.serviceClient<behavior_coordinator_msgs::StartTask>("/" + drone_id_namespace +  "/" + start_task);
  
  std::cout << "Mission configuration folder: " << mission_configuration_folder << std::endl;

  //Subscribers
  list_of_behaviors_sub = nh.subscribe("/" + drone_id_namespace +  "/" + list_of_running_tasks, 1000, &BehaviorTreeControl::newBehaviorCallback, this);
  battery_subs = nh.subscribe("/" + drone_id_namespace + "/" + drone_driver_sensor_battery, 1000, &BehaviorTreeControl::batteryCallback, this);
  tasks_ended=nh.subscribe("/" + drone_id_namespace +  "/" + task_stopped, 1000, &BehaviorTreeControl::CallbackBehaviorActivationFinished, this);

  //wificonnection_subs = nh.subscribe("/" + drone_id_namespace + "/"  + wifi_connection_topic, 1, &BehaviorTreeControl::wifiConnectionCheckCallback, this);
  ui->value_vehicle->setText( QString::fromUtf8(drone_id_namespace.c_str()));

  //Publishers
  mission_state_publ=nh.advertise<std_msgs::Bool>("/"+ drone_id_namespace +  "/" +mission_state_topic ,1, true);
  pose_reference_publ = n.advertise<geometry_msgs::PoseStamped>("/"+drone_id_namespace+"/"+pose_ref_topic_name, 1, true);

}

/*------------------------------------------------------------
--------------------------------------------------------------
                    File handlers
--------------------------------------------------------------
------------------------------------------------------------*/

void BehaviorTreeControl::loadTree()
{
  file_route = mission_configuration_folder + "/behavior_tree_mission_file.yaml";

  std::ifstream aux_file(file_route);

  if (aux_file.fail())
  {
    windowManager('c', "Loading mission error", "behavior_tree_mission_file.yaml is not defined in configuration folder. Please create it using the Behavior Tree Editor.");
  }

  else
  {
    FileManager* tfm = new FileManager();
    root_node = tfm->loadTree(file_route);
    delete tfm;
    
    correct_format = true;
    tree->createMissionByTreeItem(root_node);
    tree->show();
    tree_label->show();
  }
}

/*------------------------------------------------------------
--------------------------------------------------------------
                    Button actions
--------------------------------------------------------------
------------------------------------------------------------*/

void BehaviorTreeControl::landTreeMission()
{
  
  stopMission=true;
  
  behavior_coordinator_msgs::StartTask::Request msg;
  behavior_coordinator_msgs::StartTask::Response res;
  behavior_coordinator_msgs::TaskCommand behavior;
  behavior.name = "LAND";
  behavior.priority = 1;
  msg.task = behavior;
  activate_task_srv.call(msg,res);

  if(!res.ack)
  {
    std::cout << res.error_message << std::endl;
    if (visualState==3 || visualState==2 )//if (paused)
    {
      itemPaused->setColor(COLOR_RED);
      itemPaused->setColorBackground("#ffffff");
    }
    visualState=0;
    lastVisualState=0;

    if (visualState==5 ||visualState==7)
    {
      itemPaused->setColor(COLOR_RED);
      itemPaused->setColorBackground("#ffffff");
    }
  }

  visualState=0;
  lastVisualState=0;
  lastVisualState=visualState;
  changeVisual();
  visualState=lastVisualState;
  tree->setRunning(false);
  tree->connectCustomContextMenu();
  deleteRunningNodes();
  itemPaused->setColor(COLOR_RED);
  itemPaused->setColorBackground("#ffffff");
  setStopBlockingTextInput();
  variables->deleteVariables();
  cleanVariableText();
}

void BehaviorTreeControl::waitTimeForStart(){
  lastVisualState=visualState;
  visualState=9;
  changeVisual();
  visualState=lastVisualState;
}

void BehaviorTreeControl::executeTreeMission()//this method executes the mission and calls executetree to do it
{
  addTextVariables();
  if(completed_mission || mission_failed)
  {
    cleanTree();
  }
  paused=false;
  stopMission=false;
  completed_mission=false;
  mission_failed=false;

  if(aborted)
  {    
    aborted=false;
    resetTreeStatus(root_node);
  }
  //we need to check if it is the first time the mission is executed
  if(root_node->getStatus()!=BT::NON_INITIATED)
  { 
    cleanTree();
    deleteRunningNodes();
    tree->disconnectCustomContextMenu();
    doneFirst=false;
    Q_EMIT(executionStarted());
    root_node->setColor(COLOR_BLACK);
    root_node->setColorBackground("#ffffff");
    executing_tree= new std::thread(&BT::BehaviorTree::execute,tree,root_node,2000);
    tree->setRunning(true);
    missionStateMsgs.data=true;
    mission_state_publ.publish(missionStateMsgs);
  }
  else
  {
    tree->disconnectCustomContextMenu();
    doneFirst=false;
    Q_EMIT(executionStarted());
    root_node->setColor(COLOR_BLACK);
    root_node->setColorBackground("#ffffff");
    executing_tree= new std::thread(&BT::BehaviorTree::execute,tree,root_node,2000);
    tree->setRunning(true);
    missionStateMsgs.data=true;
    mission_state_publ.publish(missionStateMsgs);
  }
  
}
void BehaviorTreeControl::executeTreeFromItem(BT::TreeNode * item_to_execute)
{
  addTextVariables();
  
  cleanTree();
  
  completed_mission=false;
  itemPaused=item_to_execute;
  stopMission=false;
  paused=true;
  mission_failed=false;
  if(aborted)
  {
    itemPaused->setisAborted(true);
    aborted=false;
    resetTreeStatus(root_node);
    //poner subarbol a nuevo estado de no ejecucion
    bool treeaborted=setAbortedItems(root_node);
    itemPaused->setisAborted(false);
  }
  else if(paused){
    itemPaused->setisAborted(true);
    aborted=false;
    resetTreeStatus(root_node);
    //poner subarbol a nuevo estado de no ejecucion
    bool treeaborted=setAbortedItems(root_node);
    itemPaused->setisAborted(false);
  }
  tree->disconnectCustomContextMenu();
  doneFirst=false;
  Q_EMIT(executionStarted());
  root_node->setColor(COLOR_BLACK);
  root_node->setColorBackground("#ffffff");
  executing_tree= new std::thread(&BT::BehaviorTree::execute,tree,root_node,2000);
  tree->setRunning(true);
  missionStateMsgs.data=true;
  mission_state_publ.publish(missionStateMsgs);
  
}

void BehaviorTreeControl::pauseMission(){
  
  if (!pause_continue_changer)
  { 
    //pause mission
    stopMission=true;
    lastVisualState=visualState;
    tree->connectCustomContextMenu();
    //change visual to paused
    visualState=3;
    changeVisual();
    pause_continue_changer= true;
    beliefs_text->setReadOnly(false);


    if(tree->isRunning())
    {
      BT::BehaviorTree * myTree = this->tree;
      tree->setRunning(false);  
      myTree->cancelTree();
      itemPaused->setStatus(BT::NON_INITIATED);
      itemPaused->setColorBackground(COLOR_GRAY);
      itemPaused->setColor("#ffffff");
      paused=true;
    }
    deletePauseNode();

  }
  else
  {
    //add variables and stop the input
    beliefs_text->setReadOnly(true);
    addTextVariables();
    //change visual state
    lastVisualState=visualState;
    visualState=6;
    changeVisual();
    pause_continue_changer= false;
    //execute the tree
    doneFirst=false;
    Q_EMIT(executionStarted());
    stopMission=false;
    executing_tree= new std::thread(&BT::BehaviorTree::execute,tree,root_node,2000);
    tree->setRunning(true);
    tree->disconnectCustomContextMenu();
    missionStateMsgs.data=true;
    mission_state_publ.publish(missionStateMsgs);
    
    
  }  
}


void BehaviorTreeControl::abortTreeMission()
{ 
  //set abort true and stop mission
  aborted=true;
  stopMission=true;
  if(tree->isRunning())
  {
    BT::BehaviorTree * myTree = this->tree;
    tree->setRunning(false);  
    myTree->cancelTree();
    tree->connectCustomContextMenu();
  }
  //paint respective node and allow variables input
  itemPaused->setColor(COLOR_PURPLE);
  itemPaused->setColorBackground("#ffffff");
  resetTreeStatus(root_node);
  lastVisualState= visualState;
  visualState=2;
  variables->deleteVariables();
  cleanVariableText();
  beliefs_text->setReadOnly(false);

  changeVisual();
  
}

void BehaviorTreeControl::completedMission()
{
  completed_mission=true;
  visualState=0;
  changeVisual();
  resetTreeStatus(root_node);
  tree->connectCustomContextMenu();
  setStopBlockingTextInput();
  variables->deleteVariables();
  cleanVariableText();
}
void BehaviorTreeControl::missionFailed()
{
  mission_failed=true;
  resetTreeStatus(root_node);
  tree->connectCustomContextMenu();

  setStopBlockingTextInput();
  variables->deleteVariables();
  cleanVariableText();

}

void BehaviorTreeControl::cleanTree(){//this method changes the state of the nodes to the non initiated state in order to repeat the mission
  resetTreeStatus(root_node);
  resetTreeColor(root_node);
}
void BehaviorTreeControl::resetTreeColor(BT::TreeNode* node)
{
  itemPaused->setColor(COLOR_BLACK);
  itemPaused->setColorBackground("#ffffff");
  root_node->setColor(COLOR_BLACK);
  root_node->setColorBackground("#ffffff");
  std::vector<BT::TreeNode*> nodes =node->getChildren();
  for(int i=0 ; i<nodes.size() ; i++)
  {
    if(!nodes[i]->getChildren().empty())
    {
      resetTreeColor(nodes[i]);
    }
    nodes[i]->setColor(COLOR_BLACK);
    root_node->setColorBackground("#ffffff");
  }
}

void BehaviorTreeControl::resetTreeStatus(BT::TreeNode* node)
{
  root_node->setStatus(BT::NON_INITIATED);
  itemPaused->setStatus(BT::NON_INITIATED);
  std::vector<BT::TreeNode*> nodes =node->getChildren();
  for(int i=0 ; i<nodes.size() ; i++)
  {
    if(!nodes[i]->getChildren().empty())
    {
      resetTreeStatus(nodes[i]);
    }
    nodes[i]->setStatus(BT::NON_INITIATED);
  }
}

void BehaviorTreeControl::deletePauseNode()
{
  for(int i=0; i<running_nodes.size(); i++)
  {
    if(!running_nodes[i]->getName().compare(itemPaused->getName()))
    { 
      BT::TreeNode* node=running_nodes[i];
      running_nodes.erase(running_nodes.begin()+i);
    }
  }
}
bool BehaviorTreeControl::setAbortedItems(BT::TreeNode* node)
{
  std::vector<BT::TreeNode*> children=node->getChildren();
  bool exit=false;
  root_node->setStatus(BT::ABORTED);
  for(int i =0; i < children.size() && !exit; i++)
  {
    if(!children[i]->getName().compare(itemPaused->getName()) && children[i]->getisAborted())
    {
      exit=true;
    }
    else
    {
      children[i]->setStatus(BT::ABORTED);
    }
    if(!children[i]->getChildren().empty())
    {
      if(!exit){
        exit=setAbortedItems(children[i]);
      }
    }
    
  }
  return exit;
}
void BehaviorTreeControl::deleteRunningNodes()
{
  for(int i=0; i<running_nodes.size(); i++)
  {
      running_nodes.erase(running_nodes.begin()+i);
  }
}
void BehaviorTreeControl::changeVisual(){// this is the method that changes the visual components . This will be explain in the file explanation.txt
  sleep(0.5);
  switch(visualState){
    case 0://inicial landed
    ui->execute_tree_button->show();
    ui->pause_mission->hide();
    ui->pause_mission->setIcon(QIcon(":/img/img/pause.png"));
    ui->teleoperation_button->hide();
    ui->abort_tree_button->hide();
    ui->emergency_land_tree_button->hide();
    ui->emergency_land_tree_button->setText("Emergency land");
    ui->finish_teleoperation_button->hide();
    ui->groupBox_3->hide();
    ui->groupBox_4->hide();
    ui->groupBox_5->hide();
    ui->keys_teleop->hide();
    ui->teleoperation_button->setText("Change to teleoperation");
    hider=false;
    if (lastVisualState==3){
      //tree->et->setColor(itemPaused,"#c41306");
      //tree->et->setColorBackground(itemPaused,"#ffffff");
    }
    break;
    
    case 1://started
    ui->abort_tree_button->show();
    ui->emergency_land_tree_button->show();
    ui->pause_mission->setText("Pause mission");
    ui->pause_mission->setIcon(QIcon(":/img/img/pause.png"));
    ui->pause_mission->show();
    if (lastVisualState!=2){
      ui->execute_tree_button->hide();
    }
    ui->teleoperation_button->setText("Change to teleoperation");                
    ui->teleoperation_button->hide();
    ui->finish_teleoperation_button->hide();
    break;

    case 2://aborting
    tree->connectCustomContextMenu();
    ui->abort_tree_button->hide();
    ui->emergency_land_tree_button->hide();
    ui->finish_teleoperation_button->setText("Emergency land");
    ui->finish_teleoperation_button->show();
    ui->pause_mission->hide();
    ui->teleoperation_button->show();
    break ;

    case 3://paused
    tree->connectCustomContextMenu();
    ui->emergency_land_tree_button->setText("Emergency land");
    ui->emergency_land_tree_button->show();
    ui->abort_tree_button->show();
    ui->teleoperation_button->show();
    ui->pause_mission->setText("Continue mission");
    ui->pause_mission->setIcon(QIcon(":/img/img/play.png"));
    ui->pause_mission->show();
    ui->finish_teleoperation_button->hide();
    break;
    
    case 4://teleoperation off
    ui->groupBox_3->hide();
    ui->groupBox_4->hide();
    ui->groupBox_5->hide();
    ui->teleoperation_button->setText("Change to teleoperation");
    visualState=lastVisualState;
    ui->keys_teleop->hide();
    changeVisual();
    break;

    case 5://teleoperation on cancelled
    tree->disconnectCustomContextMenu();
    ui->groupBox_3->show();
    ui->groupBox_4->show();
    ui->groupBox_5->show();
    ui->teleoperation_button->setText("Return to mission control");
    ui->keys_teleop->show();
    break;
    
    case 6://continue
    ui->pause_mission->setText("Pause mission");
    ui->pause_mission->setIcon(QIcon(":/img/img/pause.png"));
    ui->pause_mission->show();
    ui->teleoperation_button->hide();
    break;

    case 7://teleoperation on paused
    tree->disconnectCustomContextMenu();
    ui->groupBox_3->show();
    ui->groupBox_4->show();
    ui->groupBox_5->show();
    ui->keys_teleop->show();
    ui->emergency_land_tree_button->hide();
    ui->pause_mission->hide();
    ui->abort_tree_button->hide();
    ui->teleoperation_button->setText("Return to mission control");
    ui->finish_teleoperation_button->setText("Finish teleoperation and land");
    ui->finish_teleoperation_button->show();
    break;
    
    case 8://disable
    ui->teleoperation_button->setEnabled(false);
    ui->emergency_land_tree_button->setEnabled(false);
    ui->pause_mission->setEnabled(false);
    ui->execute_tree_button->setEnabled(false);
    ui->abort_tree_button->setEnabled(false);
    ui->finish_teleoperation_button->setEnabled(false);
    break;

    case 9://enable
    ui->teleoperation_button->setEnabled(true);
    ui->emergency_land_tree_button->setEnabled(true);
    ui->pause_mission->setEnabled(true);
    ui->execute_tree_button->setEnabled(true);
    ui->abort_tree_button->setEnabled(true);
    ui->finish_teleoperation_button->setEnabled(true);
    break;

    case 10://finish
    lastVisualState=0;
    visualState=0;
    changeVisual();
    break;
  }

}

/*------------------------------------------------------------
--------------------------------------------------------------
                  Keyboard manager
--------------------------------------------------------------
------------------------------------------------------------*/

void BehaviorTreeControl::setKeyboard()
{ 
  isAKeyPressed = false;

  setFocusPolicy(Qt::StrongFocus);
  acceptedKeys.insert(0x01000012, false); //Tecla UP
  acceptedKeys.insert(0x01000013, false); //Tecla DOWN
  acceptedKeys.insert(0x01000014, false); //Tecla LEFT
  acceptedKeys.insert(0x01000015, false); //Tecla RIGHT
  acceptedKeys.insert(0x51, false); //Tecla Q
  acceptedKeys.insert(0x41, false); //Tecla A
  acceptedKeys.insert(0x54, false); //Tecla T
  acceptedKeys.insert(0x59, false); //Tecla Y
  acceptedKeys.insert(0x48, false); //Tecla H
  acceptedKeys.insert(0x5a, false); //Tecla Z
  acceptedKeys.insert(0x58, false); //Tecla X
  acceptedKeys.insert(0x52, false); //Tecla R
}

void BehaviorTreeControl::keyPressEvent(QKeyEvent *e)
{

  if(teleoperationActivated){


    QWidget::keyPressEvent(e);

    if(!isAKeyPressed && acceptedKeys.contains(e->key()))
    {
      isAKeyPressed = true;
      switch(e->key())
      {
        case Qt::Key_R:
        {
          if(!e->isAutoRepeat())
          {
            acceptedKeys.insert(0x52, true);
            onResetCommandButton();
          }
          break;
        }
        case Qt::Key_T:
        {
          if(!e->isAutoRepeat())
          {
            acceptedKeys.insert(0x54, true);
            onTakeOffButton();

          }
          break;
        }
        case Qt::Key_Y:
        {
          if(!e->isAutoRepeat())
          {
            acceptedKeys.insert(0x59, true);
            onLandButton();

          }
          break;
        }
        case Qt::Key_H:
        {
          acceptedKeys.insert(0x48, true);
          behavior_coordinator_msgs::StartTask::Request msg;
          behavior_coordinator_msgs::StartTask::Response res;
          behavior_coordinator_msgs::TaskCommand behavior;
          behavior.name = "HOVER";
          behavior.priority = 2;
          msg.task = behavior;
          activate_task_srv.call(msg,res);
          if(!res.ack)
            std::cout << res.error_message << std::endl;

          break;
        }
        case Qt::Key_Right:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x01000014, true);
            behavior_coordinator_msgs::StartTask::Request msg;
            behavior_coordinator_msgs::StartTask::Response res;
            behavior_coordinator_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.priority = 2;
            behavior.parameters = "direction: \"RIGHT\"\nspeed: 0.4";
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
   
          break;
        
        }
        case Qt::Key_Left:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x01000012, true);
            behavior_coordinator_msgs::StartTask::Request msg;
            behavior_coordinator_msgs::StartTask::Response res;
            behavior_coordinator_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.parameters = "direction: \"LEFT\"\nspeed: 0.4";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_Down:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x01000015, true);
            behavior_coordinator_msgs::StartTask::Request msg;
            behavior_coordinator_msgs::StartTask::Response res;
            behavior_coordinator_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.parameters = "direction: \"BACKWARD\"\nspeed: 0.4";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_Up:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x01000013, true);
            behavior_coordinator_msgs::StartTask::Request msg;
            behavior_coordinator_msgs::StartTask::Response res;
            behavior_coordinator_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.parameters = "direction: \"FORWARD\"\nspeed: 0.4";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_Q:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x51, true);
            behavior_coordinator_msgs::StartTask::Request msg;
            behavior_coordinator_msgs::StartTask::Response res;
            behavior_coordinator_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.parameters = "direction: \"UP\"\nspeed: 0.4";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_A:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x41, true);
            behavior_coordinator_msgs::StartTask::Request msg;
            behavior_coordinator_msgs::StartTask::Response res;
            behavior_coordinator_msgs::TaskCommand behavior;
            behavior.name = "MOVE_AT_SPEED";
            behavior.parameters = "direction: \"DOWN\"\nspeed: 0.4";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_Z:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x5a, true);
            behavior_coordinator_msgs::StartTask::Request msg;
            behavior_coordinator_msgs::StartTask::Response res;
            behavior_coordinator_msgs::TaskCommand behavior;
            behavior.name = "ROTATE";
            behavior.parameters = "relative_angle: +179";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;

          }
          break;
        }
        case Qt::Key_X:
        {
          if(!e->isAutoRepeat() && acceptedKeys.contains(e->key()) && !acceptedKeys.value(e->key()))
          {
            acceptedKeys.insert(0x58, true);
            behavior_coordinator_msgs::StartTask::Request msg;
            behavior_coordinator_msgs::StartTask::Response res;
            behavior_coordinator_msgs::TaskCommand behavior;
            behavior.name = "ROTATE";
            behavior.parameters = "relative_angle: -179";
            behavior.priority = 2;
            msg.task = behavior;
            activate_task_srv.call(msg,res);
            if(!res.ack)
              std::cout << res.error_message << std::endl;
         
          }
          break;
        }
      }
    }
  }
}

void BehaviorTreeControl::onTakeOffButton()
{
  behavior_coordinator_msgs::StartTask::Request msg;
  behavior_coordinator_msgs::StartTask::Response res;
  behavior_coordinator_msgs::TaskCommand behavior;
  behavior.name = "TAKE_OFF";
  behavior.priority = 2;
  msg.task = behavior;
  activate_task_srv.call(msg,res);
  if(!res.ack)
    std::cout << res.error_message << std::endl;
}

void BehaviorTreeControl::onLandButton()
{
  behavior_coordinator_msgs::StartTask::Request msg;
  behavior_coordinator_msgs::StartTask::Response res;
  behavior_coordinator_msgs::TaskCommand behavior;
  behavior.name = "LAND";
  behavior.priority = 2;
  msg.task = behavior;
  activate_task_srv.call(msg,res);
  if(!res.ack)
    std::cout << res.error_message << std::endl;
}

void BehaviorTreeControl::onResetCommandButton()
{
  /** NOTE:
  Shows strange behaviour when the drone has been ordered to rotate previously,
  and a stabilize command was not issued after the rotation.
 */
  behavior_coordinator_msgs::StartTask::Request msg;
  behavior_coordinator_msgs::StartTask::Response res;
  behavior_coordinator_msgs::TaskCommand behavior;
  behavior.name = "ROTATE";
  behavior.parameters = "angle: 0";
  behavior.priority = 2;
  msg.task = behavior;
  activate_task_srv.call(msg,res);
  if(!res.ack)
    std::cout << res.error_message << std::endl;
}

void BehaviorTreeControl::keyReleaseEvent(QKeyEvent *e)
{
  if(e->isAutoRepeat() || !acceptedKeys.contains(e->key()))
  {
    isAKeyPressed = false;
    e->ignore();
  }
  else if(acceptedKeys.contains(e->key()) && acceptedKeys.value(e->key()))
  {
    acceptedKeys.insert(e->key(), false);
    behavior_coordinator_msgs::StartTask::Request msg;
    behavior_coordinator_msgs::StartTask::Response res;
    behavior_coordinator_msgs::TaskCommand behavior;
    behavior.name = "HOVER";
    msg.task = behavior;
    behavior.priority = 1;
    if(e->key() != Qt::Key_Y && e->key() != Qt::Key_T && e->key() != Qt::Key_R)
      if(!res.ack)
        std::cout << res.error_message << std::endl;
      isAKeyPressed = false;
      QWidget::keyReleaseEvent(e);
    }
    else
    {
      isAKeyPressed = false;
      e->ignore();
      QWidget::keyReleaseEvent(e);
    }
  }

/*------------------------------------------------------------
--------------------------------------------------------------
                Messages output manager
--------------------------------------------------------------
------------------------------------------------------------*/
  void BehaviorTreeControl::windowManager(char type, std::string title, std::string message)
  {
    QMessageBox *msg_error = new QMessageBox(QMessageBox::Critical, title.c_str(), message.c_str(), QMessageBox::Ok,this);
    msg_error->setWindowFlags(msg_error->windowFlags() | Qt::WindowStaysOnTopHint);
    msg_error->exec();

  }

/*------------------------------------------------------------
--------------------------------------------------------------
                      Callback methods
--------------------------------------------------------------
------------------------------------------------------------*/

void BehaviorTreeControl::CallbackBehaviorActivationFinished(const behavior_coordinator_msgs::TaskStopped &msg)
{ 
  int termination_cause=msg.termination_cause;
  for(int i=0; i<running_nodes.size(); i++)
  {
    BT::BehaviorTask * node = (BT::BehaviorTask *) running_nodes[i];
    if(!node->getTaskName().compare(msg.name) && node->getStatus()==BT::RUNNING)
    {
      //termination_cause=1 --> GOAL_ACHIEVED
      if(termination_cause==behavior_coordinator_msgs::TaskStopped::GOAL_ACHIEVED )
      {
        running_nodes[i]->setStatus(BT::SUCCESSFUL_COMPLETION);
		    running_nodes[i]->setColor(COLOR_GREEN);
        running_nodes[i]->setColorBackground("#ffffff");
        running_nodes.erase(running_nodes.begin()+i);
      }
      else
      {
        if(termination_cause!=behavior_coordinator_msgs::TaskStopped::INTERRUPTED)  
        {
          running_nodes[i]->setStatus(BT::FAILURE_COMPLETION);
          running_nodes[i]->setColor(COLOR_RED);
          running_nodes[i]->setColorBackground("#ffffff");
          running_nodes.erase(running_nodes.begin()+i);
        }
      }
      
    }
  }

  //sleep(2);
}


  void BehaviorTreeControl::newBehaviorCallback(const behavior_coordinator_msgs::ListOfRunningTasks &msg)
  {
    if (msg.list_of_running_tasks.size() != 0)
    {
      for (int i = 0; i < msg.list_of_running_tasks.size(); i++)
      {
        if (msg.list_of_running_tasks[i].task_command.name == "TAKE_OFF")
        {
          this->current_time->setHMS(00, 00, 00);
          ui->value_flight_time->setText(this->current_time->toString());
          is_takenOff = true;
        }
        else if (msg.list_of_running_tasks[i].task_command.name == "LAND")
          is_takenOff = false;
      }
    //Battery
      if (battery_msgs.percentage*100 <= 25.0 && battery_msgs.percentage*100 != 0)
      {
        QPalette* palette = new QPalette();
        palette->setColor(QPalette::WindowText, Qt::red);
        ui->value_battery->setPalette(*palette);
      }
      ui->value_battery->setText(QString::number(battery_msgs.percentage*100) +  "%");

    }
  }

  void BehaviorTreeControl::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
  {
    battery_msgs = *msg;
    if (battery_msgs.percentage*100 <= 25.0 && battery_msgs.percentage*100 != 0)
    {
      QPalette* palette = new QPalette();
      palette->setColor(QPalette::WindowText, Qt::red);
      ui->value_battery->setPalette(*palette);
    }
    ui->value_battery->setText(QString::number(battery_msgs.percentage*100) +  "%");
  }
