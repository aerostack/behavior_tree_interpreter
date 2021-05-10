/*!********************************************************************************
 * \brief     This is the header of the add_belief class 
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
#ifndef ADD_BELIEF_H
#define ADD_BELIEF_H

#include<leaf_node.h>
#include<string>
#include <belief_manager_msgs/AddBelief.h>
#include <belief_manager_msgs/CheckBeliefFormat.h>
#include <ros/ros.h>

#include <behavior_tree.h>

namespace BT
{
  class AddBelief : public LeafNode
  {
    private:
      std::string belief_expression;
      //whether to keep clashing beliefs in the memory or not
      bool belief_multivalued;
      
      ros::ServiceClient add_belief_srv;
      belief_manager_msgs::AddBelief::Response res_activate;
      belief_manager_msgs::AddBelief::Request req_activate;
      ros::ServiceClient check_belief_srv;
      belief_manager_msgs::CheckBeliefFormat::Response res_activate_check;
      belief_manager_msgs::CheckBeliefFormat::Request req_activate_check;
      std::string add_belief;
      std::string check_belief;
    
    public:
      explicit AddBelief(std::string name , std::string belief_expression , bool belief_multivalued, QPixmap icono_pixmap);
      ~AddBelief();
      
      BT::ReturnStatus executeStep();
      
      //this function calls the service add belief and returns its success
      bool addBelief();
      //this function checks if the parameter contains variables
      bool checkBeliefVariables(std::string parameters);
      //this function returns the string with the substitution done
      std::string substitutionVariables(std::string parameters);


  };
}
#endif