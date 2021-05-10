
#ifndef GLOBAL
#define GLOBAL

extern BT::TreeNode * itemPaused; //last item executed
extern BT::TreeNode * parentPaused;
/* this is a vector that saves the nodes that are currently doing a task */
extern std::vector<BT::TreeNode *> running_nodes;
extern bool stopMission;  //variable that stops the execution of the behavior tree
extern bool aborted;
extern bool paused;
extern bool expandedText;
extern bool cancelled;
extern int visualState;
extern int lastVisualState;
extern bool abortingWait;
extern bool doneFirst;
extern bool processing_belief_query;//we use this attribute tu assure that the query has done their job
extern bool completed_mission;
extern bool mission_failed;
extern Variables * variables;
extern bool new_variable;
#endif
