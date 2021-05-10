#include "behavior_tree_control_view.h"
#include <QApplication>
#include <stdio.h>
#include <ros/ros.h>
#include "behavior_tree_control_view.h"
#include <signal.h>
#include "global.h"

//we need to declare all the global.h variables
BT::TreeNode* itemPaused;
BT::TreeNode* parentPaused;
std::vector<BT::TreeNode *> running_nodes;
bool paused;
bool expandedText;
bool cancelled;
int visualState;
int lastVisualState;
bool abortingWait;
bool processing_belief_query;
bool doneFirst;
bool stopMission;
bool aborted;
bool completed_mission;
bool mission_failed;
Variables * variables;
bool new_variable;

int main(int argc, char *argv[]) {
  ros::init(argc,argv,"behavior_tree_control_keyboard"); //ros node started.
  QApplication app(argc, argv);
  BehaviorTreeControlView w(argc, argv);
  
  w.show();

  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}