#include "file_manager.h"
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>

FileManager::FileManager()
{
  ros::NodeHandle nh("~");
  nh.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
  nh.param<std::string>("catalog_path", behavior_catalog_path, "${AEROSTACK_PROJECT}/configs/mission/behavior_catalog.yaml");
  nh.param<std::string>("consult_available_behaviors", consult_available_behaviors, "consult_available_behaviors");
  nh.param<std::string>("check_behavior_format_srv", check_behavior_format, "check_behavior_format");

  check_behavior_format_srv=nh.serviceClient<aerostack_msgs::CheckBehaviorFormat>("/"+drone_id_namespace+"/"+check_behavior_format);

  sleep(3);
  bool loaded = loadConfiguration(behavior_catalog_path);

  available_behaviors= getBehaviors();
  for (int i = 0; i < available_behaviors.size(); i++) 
  {
    std::cout<<"behavior " << available_behaviors.at(i) << ' ';
  }
}

FileManager::~FileManager()
{
  
}

BT::TreeNode* FileManager::loadTree(std::string load_route)
{
  BT::TreeNode* root = nullptr;
  QMessageBox error_message;

  YAML::Node archive = YAML::LoadFile(load_route);
  YAML::Node nodes;
  
  aerostack_msgs::CheckBehaviorFormat::Request check_format_msg_req;
  
  if (!(nodes = archive["nodes"]))
  {
    error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
    error_message.setText(QString::fromStdString("The mission file is badly structured. Unable to import. You must specify nodes at the start of the file."));
    error_message.exec();
    return nullptr;
  }

  std::vector<BT::TreeNode*> item;
  std::map<int, BT::TreeNode*> m;
  std::map<int, int> map_parent;

  std::vector <std::string> node_types = {"CONTROL_FLOW", "EXECUTION"};
  std::vector <std::string> node_subtype={"ADD_BELIEF","REMOVE_BELIEF","QUERY_BELIEF","TASK","SEQUENCE","FALLBACK"};

  std::vector <std::string> task_no_params={"TAKE_OFF","LAND","HOVER","CLEAR_OCCUPANCY_GRID","PAY_ATTENTION_TO_QR_CODES","PAY_ATTENTION_TO_ROBOT_MESSAGES",
  "INFORM_POSITION_TO_ROBOTS"};

  int parent;
  if(archive.size() != 0) 
  { 
    for (std::size_t i=0;i<nodes.size();i++) 
    {
      YAML::Node node = nodes[i];
      try
      {
        parent = node["parent"].as<int>();
      }
      catch (const std::exception &e)
      {
        std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect parent format" << "\n";
        error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
        error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "." +"\n Every node must have a parent."));
        error_message.exec();
        return nullptr;
      }
                     
      if(parent == 0)
      {
        BT::TreeNode* item;
        int node_id;
        try
        {
          node_id = node["node"].as<int>();
        }
        catch (const std::exception &e)
        {
          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect node node format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+ "\n The parameter node must be an integer."));
          error_message.exec();
          return nullptr;
        }
        std::string name;
        std::string parameters;
        BT::NodeType type;
        BT::NodeAction subtype;
        std::string typeaux;
        std::string subtypeaaux;
        std::string task;
        bool wait_for_completion;
        bool multivalued;
        try
        {
          name = node["name"].as<std::string>(); 
        } catch (const std::exception &e)
        {
          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect node name format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\n The node name must be a string"));
          error_message.exec();
          return nullptr;
        }
        try
        {
          type = item->stringToNodeType(node["type"].as<std::string>()); 
          subtype = item->stringToNodeAction(node["subtype"].as<std::string>());

          typeaux = node["type"].as<std::string>();
          subtypeaaux = node["subtype"].as<std::string>(); 
          
          if(type==BT::CONTROL_NODE && (subtype!=BT::SEQUENCE_NODE && subtype!=BT::FALLBACK_NODE))
          {
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect type or subtype format" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\n Control flow nodes only accept subtype sequence or fallback"));
            error_message.exec();
            return nullptr;
          }
          if(type==BT::ACTION_NODE && subtype!=BT::ADD_BELIEF && subtype!=BT::REMOVE_BELIEF && subtype!=BT::QUERY_BELIEF && subtype!=BT::BEHAVIOR_TASK)
          {
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect type or subtype format" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\n Execution nodes can't be sequence or fallback"));
            error_message.exec();
            return nullptr;
          }
        } 
        catch (const std::exception &e)
        {
          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect behavior type or subtype format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nNode type and subtype must be strings."));
          error_message.exec();
          return nullptr;
        }
        if(type==BT::ACTION_NODE)
        {
          if(subtype==BT::ADD_BELIEF)
          {
            try
              {
                multivalued = node["multivalued"].as<bool>();   
              }
              catch(const std::exception& e)
              {
                std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect multivalued format" << "\n";
                error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
                error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nMultivalued parameter must be a boolean."));
                error_message.exec();
                return nullptr;
              }
          }
          else
          {
            //If it is not an add belief node it should not have multivalued parameter
            try
            {
              multivalued = node["multivalued"].as<bool>();
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Multivalued parameter is only for add belief nodes" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nMultivalued parameter is only for add belief nodes."));
              error_message.exec();
              return nullptr;   
            }
            catch(const std::exception& e){}
          }
          if(subtype==BT::BEHAVIOR_TASK)
          {
            try
            {
              task = node["task"].as<std::string>();
              if(std::find(available_behaviors.begin(), available_behaviors.end(), task) == available_behaviors.end())
              {
                for (int i = 0; i < available_behaviors.size(); i++) 
                {
						      std::cout << available_behaviors.at(i) << ' ';
				        }
                std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect behavior_type \"" << task << "\"\n";
                error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
                error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\n The task specified was not found on the catalog"));
                error_message.exec();
                return nullptr;
              }  
            }
            catch(const std::exception& e)
            {
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect task format" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nNode task must be a string."));
              error_message.exec();
              return nullptr;
            }
            std::for_each(task.begin(), task.end(), [](char & c) 
            {
              c = ::toupper(c);
            });
            if(std::find(task_no_params.begin(), task_no_params.end(), task) == task_no_params.end())
            {
              try
              {
                parameters = node["parameters"].as<std::string>();
                if(!checkParameters(task,parameters))
                {
                  std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect parameters for the behavior." << "\n";
                  error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
                  error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nThe parameter of the behavior is not valid."));
                  error_message.exec();
                  return nullptr;
                }    
              }
              catch(const std::exception& e)
              {
                std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect parameters format" << "\n";
                error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
                error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nNode parameters must be a string."));
                error_message.exec();
                return nullptr;
              }
            }
            else
            {
              //tasks that does not have parameters 
              try
              {
                parameters= node["parameters"].as<std::string>();
                std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "This type of task does not require parameters." << "\n";
                error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
                error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nThis type of task does not require parameters."));
                error_message.exec();
                return nullptr;
              }
              catch(const std::exception& e){}
            }
            try
            {
              wait_for_completion = node["wait_for_completion"].as<bool>(); 
            }
            catch(const std::exception& e)
            {
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect wait_for_completion format" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nThe wait for completion parameter must be a boolean."));
              error_message.exec();
              return nullptr;
            }            
          }
          else
          {
            //If it is not a behavior task node it shouldn't have task/wait_for_completion parameters
            try
            {
              task = node["task"].as<std::string>();
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Task parameter is only for task subtype nodes" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nTask parameter is only for task subtype nodes."));
              error_message.exec();
              return nullptr;
              
            }
            catch(const std::exception& e){}
            try
            {
              wait_for_completion = node["wait_for_completion"].as<bool>();
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Wait_for_completion parameter is only for task subtype nodes" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nWait_for_completion parameter is only for task subtype nodes."));
              error_message.exec();
              return nullptr; 
            }
            catch(const std::exception& e){}
            try
            {
              parameters = node["parameters"].as<std::string>();   
            }
            catch(const std::exception& e)
            {
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect parameters format" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nNode parameters must be a string."));
              error_message.exec();
              return nullptr;
            }
          }
        }
        else if(type==BT::CONTROL_NODE)
        {
          //These nodes should not have these parameters, in case they have some value they should fail
          try
          {
            task = node["task"].as<std::string>();
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Control nodes does not have task parameter" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nControl nodes does not have task parameter."));
            error_message.exec();
            return nullptr;  
          }
          catch(const std::exception& e){}
          try
          {
            parameters = node["parameters"].as<std::string>();
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Control nodes does not have parameters" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nControl nodes does not have parameters."));
            error_message.exec();
            return nullptr; 
          }
          catch(const std::exception& e){}
          try
          {
            wait_for_completion = node["wait_for_completion"].as<bool>();
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Control nodes does not have wait_for_completion parameter" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nControl nodes does not have wait_for_completion parameter."));
            error_message.exec();
            return nullptr; 
          }
          catch(const std::exception& e){} 
          try
          {
            multivalued = node["multivalued"].as<bool>();
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Control nodes does not have multivalued parameter" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nControl nodes does not have multivalued parameter."));
            error_message.exec();
            return nullptr; 
          }
          catch(const std::exception& e){}
        }
        
        
        BT::TreeNode* real_node=item->modifyNode(name, type, subtype, task, parameters,multivalued);
        m[node_id] = real_node;
        map_parent[node_id] = parent;
      } 
      else 
      {
        BT::TreeNode * item;
        YAML::Node node = nodes[i];
        int node_id;
        std::string name;
        std::string parameters;
        BT::NodeType type;
        BT::NodeAction subtype;
        std::string typeaux;
        std::string subtypeaaux;
        std::string task;
        bool wait_for_completion;
        bool multivalued;
        try
        {
          node_id = node["node"].as<int>();
        }
        catch (const std::exception &e)
        {
          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect node node format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+ "\n The parameter node must be an integer."));
          error_message.exec();
          return nullptr;
        }
        try
        {
          name = node["name"].as<std::string>(); 
        } catch (const std::exception &e)
        {
          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect node name format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\n The node name must be a string"));
          error_message.exec();
          return nullptr;
        }
        try
        {
          type = item->stringToNodeType(node["type"].as<std::string>()); 
          subtype = item->stringToNodeAction(node["subtype"].as<std::string>());

          typeaux = node["type"].as<std::string>();
          subtypeaaux = node["subtype"].as<std::string>(); 
          
          if(type==BT::CONTROL_NODE && (subtype!=BT::SEQUENCE_NODE && subtype!=BT::FALLBACK_NODE))
          {
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect type or subtype format" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\n Control flow nodes only accept subtype sequence or fallback"));
            error_message.exec();
            return nullptr;
          }
          if(type==BT::ACTION_NODE && subtype!=BT::ADD_BELIEF && subtype!=BT::REMOVE_BELIEF && subtype!=BT::QUERY_BELIEF && subtype!=BT::BEHAVIOR_TASK)
          {
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect type or subtype format" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\n Execution nodes can't be sequence or fallback"));
            error_message.exec();
            return nullptr;
          }
        } 
        catch (const std::exception &e)
        {
          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect behavior type or subtype format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nNode type and subtype must be strings."));
          error_message.exec();
          return nullptr;
        }
        if(type==BT::ACTION_NODE)
        {
          if(subtype==BT::ADD_BELIEF)
          {
            try
              {
                multivalued = node["multivalued"].as<bool>();   
              }
              catch(const std::exception& e)
              {
                std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect multivalued format" << "\n";
                error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
                error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nMultivalued parameter must be a boolean."));
                error_message.exec();
                return nullptr;
              }
          }
          else
          {
            //If it is not an add belief node it should not have multivalued parameter
            try
            {
              multivalued = node["multivalued"].as<bool>();
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Multivalued parameter is only for add belief nodes" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nMultivalued parameter is only for add belief nodes."));
              error_message.exec();
              return nullptr;   
            }
            catch(const std::exception& e){}
          }
          if(subtype==BT::BEHAVIOR_TASK)
          {
            try
            {
              task = node["task"].as<std::string>();
              if(std::find(available_behaviors.begin(), available_behaviors.end(), task) == available_behaviors.end())
              {
                for (int i = 0; i < available_behaviors.size(); i++) 
                {
						      std::cout << available_behaviors.at(i) << ' ';
				        }
                std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect behavior_type \"" << task << "\"\n";
                error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
                error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\n The task specified was not found on the catalog"));
                error_message.exec();
                return nullptr;
              }  
            }
            catch(const std::exception& e)
            {
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect task format" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nNode task must be a string."));
              error_message.exec();
              return nullptr;
            }
            std::for_each(task.begin(), task.end(), [](char & c) 
            {
              c = ::toupper(c);
            });
            if(std::find(task_no_params.begin(), task_no_params.end(), task) == task_no_params.end())
            {
              try
              {
                parameters = node["parameters"].as<std::string>();
                if(!checkParameters(task,parameters))
                {
                  std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect parameters for the behavior." << "\n";
                  error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
                  error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nThe parameter of the behavior is not valid."));
                  error_message.exec();
                  return nullptr;
                }    
              }
              catch(const std::exception& e)
              {
                std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect parameters format" << "\n";
                error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
                error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nNode parameters must be a string."));
                error_message.exec();
                return nullptr;
              }
            }
            else
            {
              //tasks that does not have parameters 
              try
              {
                parameters= node["parameters"].as<std::string>();
                std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "This type of task does not require parameters." << "\n";
                error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
                error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nThis type of task does not require parameters."));
                error_message.exec();
                return nullptr;
              }
              catch(const std::exception& e){}
            }
            try
            {
              wait_for_completion = node["wait_for_completion"].as<bool>(); 
            }
            catch(const std::exception& e)
            {
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect wait_for_completion format" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nThe wait for completion parameter must be a boolean."));
              error_message.exec();
              return nullptr;
            }            
          }
          else
          {
            //If it is not a behavior task node it shouldn't have parameters/task/wait_for_completion parameters
            try
            {
              task = node["task"].as<std::string>();
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Task parameter is only for task subtype nodes" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nTask parameter is only for task subtype nodes."));
              error_message.exec();
              return nullptr;
              
            }
            catch(const std::exception& e){}
            try
            {
              wait_for_completion = node["wait_for_completion"].as<bool>();
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Wait_for_completion parameter is only for task subtype nodes" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nWait_for_completion parameter is only for task subtype nodes."));
              error_message.exec();
              return nullptr; 
            }
            catch(const std::exception& e){}
            try
            {
              parameters = node["parameters"].as<std::string>();   
            }
            catch(const std::exception& e)
            {
              std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Incorrect parameters format" << "\n";
              error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
              error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nNode parameters must be a string."));
              error_message.exec();
              return nullptr;
            }
          }
        }
        else if(type==BT::CONTROL_NODE)
        {
          //These nodes should not have these parameters, in case they have some value they should fail
          try
          {
            task = node["task"].as<std::string>();
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Control nodes does not have task parameter" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nControl nodes does not have task parameter."));
            error_message.exec();
            return nullptr;  
          }
          catch(const std::exception& e){}
          try
          {
            parameters = node["parameters"].as<std::string>();
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Control nodes does not have parameters" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nControl nodes does not have parameters."));
            error_message.exec();
            return nullptr; 
          }
          catch(const std::exception& e){}
          try
          {
            wait_for_completion = node["wait_for_completion"].as<bool>();
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Control nodes does not have wait_for_completion parameter" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nControl nodes does not have wait_for_completion parameter."));
            error_message.exec();
            return nullptr; 
          }
          catch(const std::exception& e){} 
          try
          {
            multivalued = node["multivalued"].as<bool>();
            std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode " << std::to_string(i+1) << "\n" << "Control nodes does not have multivalued parameter" << "\n";
            error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
            error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."+"\nControl nodes does not have multivalued parameter."));
            error_message.exec();
            return nullptr; 
          }
          catch(const std::exception& e){}
        }
        
        BT::TreeNode* real_node=item->modifyNode(name, type, subtype, task, parameters, multivalued); 
        m[node_id] = real_node;
        map_parent[node_id] = parent;
      }
    }
    for (const auto& kv : map_parent) 
    {
      if (kv.second == 0) 
      {
        m[kv.first]->setHasParent(false);
        root = m[kv.first];
      }
      else 
      {
        m[kv.second]->addChild(m[kv.first]);
      }
    }
    return root;
  } 
  else 
  {
    error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
    error_message.setText(QString::fromStdString("The mission file is empty. Unable to import."));
    error_message.exec();
    return nullptr;
  }
}

bool FileManager::loadConfiguration(std::string file_path)
{ 
  std::cout<<"Catalog: "<<file_path<<std::endl;

try{
	  YAML::Node yaml_node;
  yaml_node = YAML::LoadFile(file_path);
  
  if (yaml_node["tasks"]){
  	for (YAML::const_iterator tasksi=yaml_node["tasks"].begin();tasksi!=yaml_node["tasks"].end();++tasksi) 
    {
  		std::map<std::string, std::vector<std::string>> parameters;
  		std::string name=(*tasksi)["task"].as<std::string>();
  		if((*tasksi)["task"])
      {
  			behaviors_loaded.push_back((*tasksi)["task"].as<std::string>());
  		}
  		if((*tasksi)["parameters"])
      {
  			for (YAML::const_iterator parametersIterator=(*tasksi)["parameters"].begin();
            parametersIterator!=(*tasksi)["parameters"].end();++parametersIterator)
        {
            std::string nameParameters;
            std::vector<std::string> allowed;
            if((*parametersIterator)["parameter"])
            {
                nameParameters = (*parametersIterator)["parameter"].as<std::string>();
            }
            if((*parametersIterator)["allowed_values"])
            {
            	try
              {
            		 allowed = (*parametersIterator)["allowed_values"].as<std::vector<std::string>>();
            	}
            	catch(const std::exception& e){}
            }
             parameters.insert(std::pair<std::string,std::vector<std::string>> (nameParameters,allowed));
        }
            behaviors_loaded_complete.insert(std::pair<std::string,std::map<std::string,std::vector<std::string>>>(name, parameters));
  		}
  		if (!(*tasksi)["parameters"])
  		{
  			behaviors_loaded_complete.insert(std::pair<std::string,std::map<std::string,std::vector<std::string>>>(name, parameters));
  		}
  	}
  	return true;
  }
}
catch (const std::exception& e) { // referencia a base de un objeto polimorfo
     std::cout << e.what();
     return false;
}
    
}

std::vector<std::string> FileManager::getBehaviors()
{
  return behaviors_loaded;
}

bool FileManager::checkParameters(std::string task,std::string parameters){
  std::string parametro=parameters;
  if(parameters.find('&'))
  {
    parametro=parameters.substr(0,parameters.find('&'));
    parameters=parameters.substr(parameters.find('&')+1,parameters.length());
  }
	  std::map<std::string,std::map<std::string, std::vector<std::string>>>::iterator it;

	  it = behaviors_loaded_complete.find(task);
      if (it == behaviors_loaded_complete.end()){
      	return false;
      }
      std::map<std::string,std::vector<std::string>> valid_parameters =behaviors_loaded_complete.at(task);
      if (valid_parameters.empty()){
      	  return true;
      }
      YAML::Node node= YAML::Load(parameters);

      for (auto tag : node){

      	 auto parameter_name = tag.first.as<std::string>();
      	 if (parameter_name=="direction"){
      	 	  auto parameter_value= tag.second.as<std::string>();
      	 	  if (parameter_value=="FORWARD"||parameter_value=="BACKWARD" || parameter_value=="LEFT" || 
      	 	  	parameter_value == "RIGHT")
      	 	  	{return true;}
      	 	  else{
      	 	  	return false;
      	 	  }

      	 }   	 
         std::map<std::string,std::vector<std::string>>::iterator it2=valid_parameters.find(parameter_name);
         if (it2 == valid_parameters.end()){
      	    return false;
         }

         std::vector<std::string> value= valid_parameters.at(parameter_name);//valores correctos
               	  
         if (value.empty()){
         	return true;
         }
         auto  parameter_value= tag.second.as<int>();

         std::vector<int>numbers;
         std::string aux = ""; 
         
         if (value.size()>0){
         	 for (int j =0; j<value.size();j++){
         	         numbers.push_back(stoi(value.at(j)));

             }
         int params = node[parameter_name].as<int>();
         if ((params<numbers[0] || params>numbers[1]))
         return false;
         }
         else{
         	return true;
         }
        

      }
      node= YAML::Load(parametro);

      for (auto tag : node){

      	 auto parameter_name = tag.first.as<std::string>();
      	 if (parameter_name=="direction"){
      	 	  auto parameter_value= tag.second.as<std::string>();
      	 	  if (parameter_value=="FORWARD"||parameter_value=="BACKWARD" || parameter_value=="LEFT" || 
      	 	  	parameter_value == "RIGHT")
      	 	  	{return true;}
      	 	  else{
      	 	  	return false;
      	 	  }

      	 }   	 
         std::map<std::string,std::vector<std::string>>::iterator it2=valid_parameters.find(parameter_name);
         if (it2 == valid_parameters.end()){
      	    return false;
         }

         std::vector<std::string> value= valid_parameters.at(parameter_name);//valores correctos
               	  
         if (value.empty()){
         	return true;
         }
         auto  parameter_value= tag.second.as<int>();

         std::vector<int>numbers;
         std::string aux = ""; 
         
         if (value.size()>0){
         	 for (int j =0; j<value.size();j++){
         	         numbers.push_back(stoi(value.at(j)));

             }
         int params = node[parameter_name].as<int>();
         if ((params<numbers[0] || params>numbers[1]))
         return false;
         }
         else{
         	return true;
         }
        

      }
      return true;


  }
