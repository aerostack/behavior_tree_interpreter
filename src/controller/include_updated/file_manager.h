/*!********************************************************************************
 * \brief     This is the header of the file_manager class 
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
#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

#include <iostream>
#include <fstream>
#include "tree_node.h"
#include <QTextEdit>
#include "behavior_tree.h"
#include "yaml-cpp/yaml.h"

#include <aerostack_msgs/CheckBehaviorFormat.h>
#include <aerostack_msgs/BehaviorCommandPriority.h>

class FileManager
  {
    public:
      FileManager();
      ~FileManager();
      
      BT::TreeNode* loadTree(std::string load_route);
      bool loadConfiguration(std::string file_path);
      std::vector<std::string> getBehaviors();
      bool checkParameters(std::string task, std::string parameters);
    private:
      std::string drone_id_namespace;
      std::string behavior_catalog_path;
      std::string consult_available_behaviors;
      std::string check_behavior_format;
      std::vector<std::string> available_behaviors;

      ros::ServiceClient check_behavior_format_srv;
      ros::ServiceClient consult_available_behaviors_srv;

      aerostack_msgs::BehaviorCommandPriority behavior_msg;
      std::vector<std::string> behaviors_loaded;
      std::map<std::string, std::map<std::string,std::vector<std::string>>> behaviors_loaded_complete;
  };

#endif