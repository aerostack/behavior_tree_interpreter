/*!
* \file tree_node.cpp
* \brief Generic node 
* \details This file is the definition of a tree_node the most generic version of a node in our tree.
* \author Oscar Cabrera
*/
#include "tree_node.h"
#include "add_belief.h"
#include "remove_belief.h"
#include "behavior_task.h"
#include "query_belief.h"
#include "sequence_node.h"
#include "fallback_node.h"
#include "inverter_node.h"
#include "parallel_node.h"
#include "repeat_until_fail_node.h"
#include "repeater_node.h"
#include "succeeder_node.h"
#include <string>

BT::TreeNode::TreeNode(std::string name,QPixmap icono_pixmap) : QTreeWidgetItem()
{
  this->name = name;
  this->has_parent = false;
  setStatus(BT::NON_INITIATED);

  this->setFlags(Qt::ItemIsEnabled);
  QIcon icono_action = QIcon(icono_pixmap);
  this->setIcon(0,icono_action);
  this->isAborted=false;
}

BT::TreeNode::~TreeNode() {}


bool BT::TreeNode::getHasParent()
{
  return has_parent;
}

void BT::TreeNode::setisAborted(bool value)
{
  isAborted = value;
}

bool BT::TreeNode::getisAborted()
{
  return isAborted;
}

void BT::TreeNode::setHasParent(bool value)
{
  has_parent = value;
}

BT::ReturnStatus BT::TreeNode::getStatus()
{
  return status;
}
void BT::TreeNode::setStatus(ReturnStatus new_status)
{
  status=new_status;
}
void BT::TreeNode::setName(std::string new_name)
{
  name = new_name;
}

std::string BT::TreeNode::getName()
{
  return name;
}


BT::NodeType BT::TreeNode::getType()
{
  return type;
}

void BT::TreeNode::setType(BT::NodeType TYPE)
{
  this->type=TYPE;
}

BT::NodeAction BT::TreeNode::getAction()
{
  return action;
}

void BT::TreeNode::setAction(BT::NodeAction action)
{
  this->action=action;
}

std::string BT::TreeNode::getBehaviorType()
{
  return behavior_type;
}

void BT::TreeNode::setBehaviorType(std::string btype)
{
  this->behavior_type=btype;
}

std::string BT::TreeNode::getNodeAttributes()
{
  return parameters;
}

void BT::TreeNode::setNodeAttributes(std::string attr)
{
  this->parameters=attr;
}

std::string BT::TreeNode::getPartialNodeName()
{
  return this->partial_node_name;
}
void BT::TreeNode::setPartialNodeName(std::string partial_node_name)
{
  this->partial_node_name=partial_node_name;
}

std::string BT::TreeNode::nodeTypeToString(BT::NodeType node)
{
  switch (node)
  {
	case BT::NodeType::ACTION_NODE:
	{
	  return "EXECUTION";
	}
	case BT::NodeType::CONTROL_NODE:
	{
	  return "CONTROL_FLOW";
	}	
  }
}

BT::NodeType BT::TreeNode::stringToNodeType(std::string str)
{
  std::for_each(str.begin(), str.end(), [](char & c) 
  {
    c = ::toupper(c);
  });

  if (str == "EXECUTION") 
  {
	  return NodeType::ACTION_NODE;
  }
  if (str == "CONTROL_FLOW") 
  {
	  return NodeType::CONTROL_NODE;
  }
}

std::string BT::TreeNode::nodeActionToString(BT::NodeAction node)
{
  switch (node)
  {
    case BT::NodeAction::ADD_BELIEF:
	  {
	    return "ADD_BELIEF";
	  }
	  case BT::NodeAction::BEHAVIOR_TASK:
	  {
	    return "TASK";
  	}
    case BT::NodeAction::FALLBACK_NODE:
	  {
	    return "FALLBACK";
	  }
    case BT::NodeAction::QUERY_BELIEF:
	  {
	    return "QUERY_BELIEF";
	  }
    case BT::NodeAction::REMOVE_BELIEF:
	  {
	    return "REMOVE_BELIEF";
	  }
    case BT::NodeAction::SEQUENCE_NODE:
	  {
	    return "SEQUENCE";
	  }
    case BT::NodeAction::INVERTER:
    {
      return "INVERTER";
    }
    case BT::NodeAction::PARALLEL:
    {
      return "PARALLEL";
    }
    case BT::NodeAction::REPEATER:
    {
      return "REPEATER";
    }
    case BT::NodeAction::REPEAT_UNTIL_FAIL:
    {
      return "REPEAT_UNTIL_FAIL";
    }
    case BT::NodeAction::SUCCEEDER:
    {
      return "SUCCEEDER";
    }
      
  }
}

BT::NodeAction BT::TreeNode::stringToNodeAction(std::string str)
{
  std::for_each(str.begin(), str.end(), [](char & c) 
  {
    c = ::toupper(c);
  });

  if (str == "ADD_BELIEF") 
  {
	return NodeAction::ADD_BELIEF;
  }
  if (str == "TASK") 
  {
	return NodeAction::BEHAVIOR_TASK;
  }
  if (str == "FALLBACK") 
  {
	return NodeAction::FALLBACK_NODE;
  }
  if (str == "QUERY_BELIEF") 
  {
 	return NodeAction::QUERY_BELIEF;
  }
  if (str == "REMOVE_BELIEF") 
  {
	return NodeAction::REMOVE_BELIEF;
  }
  if (str == "SEQUENCE") 
  {
	return NodeAction::SEQUENCE_NODE;
  }
  if (str == "INVERTER") 
  {
	return NodeAction::INVERTER;
  }
  if (str == "PARALLEL") 
  {
	return NodeAction::PARALLEL;
  }
  if (str == "REPEATER") 
  {
	return NodeAction::REPEATER;
  }
  if (str == "REPEAT_UNTIL_FAIL") 
  {
	return NodeAction::REPEAT_UNTIL_FAIL;
  }
  if (str == "SUCCEEDER") 
  {
	return NodeAction::SUCCEEDER;
  }
}

std::string BT::TreeNode::statusToString(ReturnStatus status)
{
  switch (status)
  {
  case BT::SUCCESSFUL_COMPLETION :
	  return "successful completion";
	  break;
  
  case BT::FAILURE_COMPLETION :
	  return "failure completion";
	  break;

  case BT::RUNNING :
	  return "Running node";
	  break;

  case BT::NON_INITIATED :
	  return "Non initiated node";
	  break;

  case BT::PAUSED :
	  return "Paused node";
	  break;

  case BT::ABORTED :
	  return "Aborted node";
	  break;
  }
}

BT::TreeNode* BT::TreeNode::modifyNode(std::string name, BT::NodeType type, BT::NodeAction subtype, 
  std::string task, std::string parameters, bool multivalued,int times,int threshold)
{
  QPixmap icono_pixmap = QPixmap(":/images/images/action.png");
  switch (subtype)
  {
	case BT::NodeAction::ADD_BELIEF:
	{
	  if(multivalued)
	    multivalued=true;
	  else
		multivalued=false;
	  
	  icono_pixmap = QPixmap(":/images/images/tree_action.png");
	  AddBelief * node = new AddBelief(name, parameters , multivalued, icono_pixmap);
	  node->partial_node_name = " [Add belief to the belief memory]";
	  
	  return node; 
	}
	case BT::NodeAction::REMOVE_BELIEF:
	{
	  icono_pixmap = QPixmap(":/images/images/tree_action.png");
	  RemoveBelief * node = new RemoveBelief(name, parameters, icono_pixmap); 
	  node->partial_node_name = " [Remove belief from the belief memory]";
	  
	  return node;
	}
	case BT::NodeAction::QUERY_BELIEF:
	{
	  icono_pixmap = QPixmap(":/images/images/query.png");
	  QueryBelief * node = new QueryBelief(name, parameters,icono_pixmap); 
	  node->partial_node_name = " [Query to the belief memory]";
	
	  return node;
	}
	case BT::NodeAction::BEHAVIOR_TASK:
	{
	  //tratamiento de texto
      //std::pair <std::string,double> res=BT::TreeNode::getTaskParameters(arguments);
	  icono_pixmap = QPixmap(":/images/images/action.png");
	  BehaviorTask * node = new BehaviorTask(name,task,parameters,1,icono_pixmap);

    node->partial_node_name = " [" + task + "]";
    	  
	  return node; 
	}
	case BT::NodeAction::SEQUENCE_NODE:
	{
	  icono_pixmap = QPixmap(":/images/images/sequence.png");
    SequenceNode* node=new SequenceNode(name, icono_pixmap);
	  node->partial_node_name = " [Execute all actions in sequence until one fails]";
	  
	  return node;
	}
	case BT::NodeAction::FALLBACK_NODE:
	{
	  icono_pixmap = QPixmap(":/images/images/selector.png");
	  FallbackNode* node=new FallbackNode(name, icono_pixmap);
	  node->partial_node_name = " [Execute all actions in sequence until one succeeds]";

	  return node;
	}
  case BT::NodeAction::INVERTER:
	{
	  icono_pixmap = QPixmap(":/images/images/inverter.png");
	  InverterNode* node=new InverterNode(name, icono_pixmap);
	  node->partial_node_name = " [Inverts the result of the child node]";

	  return node;
	}
  case BT::NodeAction::PARALLEL:
	{
	  icono_pixmap = QPixmap(":/images/images/parallel.png");
	  ParallelNode* node=new ParallelNode(name,threshold, icono_pixmap);
	  node->partial_node_name = " [Execute all actions in sequence until one succeeds]";

	  return node;
	}
  case BT::NodeAction::REPEATER:
	{
	  icono_pixmap = QPixmap(":/images/images/loop.png");
	  RepeaterNode* node=new RepeaterNode(name,times, icono_pixmap);
	  node->partial_node_name = " [Execute all actions in sequence in a loop until "+ std::to_string(times) + " iterations are executed]";

	  return node;
	}
  case BT::NodeAction::REPEAT_UNTIL_FAIL:
	{
	  icono_pixmap = QPixmap(":/images/images/loop.png");
	  RepeatFailNode* node=new RepeatFailNode(name, icono_pixmap);
	  node->partial_node_name = " [Execute all actions in sequence in a loop until one fails]";

	  return node;
	}
  case BT::NodeAction::SUCCEEDER:
	{
	  icono_pixmap = QPixmap(":/images/images/succeeder.png");
	  SucceederNode* node=new SucceederNode(name, icono_pixmap);
	  node->partial_node_name = " [Always succeeds]";

	  return node;
	}
	
  }
}  

std::pair<std::string,int> BT::TreeNode::getTaskParameters(std::string arguments)
{
  std::string delimiter = "|";

  std::string real_arguments;
  int priority;
  
  size_t pos = 0;
  pos = arguments.find(delimiter);  
  std::string token = arguments.substr(0, pos);
  arguments.erase(0, pos + 1);
  
  real_arguments=token;
  priority=std::stoi(arguments);

  return std::pair <std::string,int> (real_arguments,priority);
}

	
void BT::TreeNode::addChild(TreeNode* child)
{
  // Checking if the child has a parent already
  if (child->getHasParent())
  {
	throw std::exception();  
  }
  this->QTreeWidgetItem::addChild(child);
  child->setHasParent(true);
  children.push_back(child);
}

unsigned int BT::TreeNode::GetChildrenNumber()
{
  return children.size();
}

std::vector<BT::TreeNode*> BT::TreeNode::getChildren()
{
  return children;
}

void BT::TreeNode::setColor(std::string color) 
{
  QColor future_color = QColor(color.c_str());
  QBrush pincel = QBrush(future_color);
  this->setForeground(0, pincel);
}

void BT::TreeNode::setColorBackground(std::string color)
{
  QColor future_color = QColor(color.c_str());
  QBrush pincel = QBrush(future_color);
  this->setBackground(0, pincel);
}