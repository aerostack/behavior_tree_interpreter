/*!********************************************************************************
 * \brief     This is the add_belief class, this is a leaf node that  
 *            adds an expression to the belief memory
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

#include <add_belief.h>
#include <string>

BT::AddBelief::AddBelief(std::string name, std::string belief_expression , bool multivalued,QPixmap icono_pixmap) : LeafNode::LeafNode(name,icono_pixmap)
{
  action=ADD_BELIEF;
  
  this->belief_expression=belief_expression;
  this->belief_multivalued=multivalued;
  
  setBehaviorType(belief_expression);
  if(belief_multivalued)
  {
    setNodeAttributes("TRUE");
  }
  else
  {
    setNodeAttributes("FALSE");
  }
  
  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1"); 
  n.param<std::string>("add_belief", add_belief, "add_belief"); 
  n.param<std::string>("check_belief_format", check_belief, "check_belief_format");           
}
BT::AddBelief::~AddBelief(){}

BT::ReturnStatus BT::AddBelief::executeStep()
{
  //sleep(2);
  
  if(status==BT::NON_INITIATED || status==BT::FAILURE_COMPLETION)
  {
    setColor(COLOR_BLUE);
    itemPaused=this;
    bool activation_result=addBelief();
    /*If activation_result=true the service was succesfully completed otherwise it was a failure completion*/
    if(activation_result)
    {
      setColor(COLOR_GREEN);
      status=BT::SUCCESSFUL_COMPLETION;
    }
    else
    {
      setColor(COLOR_RED);
      status=BT::FAILURE_COMPLETION;
    }
  }
  return status;
}

bool BT::AddBelief::addBelief()
{
  QMessageBox error_message;
  /* We call the service add_belief and receive the result of the call to the service*/
  add_belief_srv = n.serviceClient<belief_manager_msgs::AddBelief>('/' + drone_id_namespace + '/' + add_belief);
  req_activate.multivalued=belief_multivalued;
  if(checkBeliefVariables(belief_expression))
  {
    try
    { 
      req_activate_check.belief_expression=substitutionVariables(belief_expression);  
    }
    catch(const std::exception& e)
    {
      std::cout << "There is no value for variable \n";
      error_message.setWindowTitle(QString::fromStdString("Loading variable value"));
      error_message.setText(QString::fromStdString("There is no value for a variable.\n"));
      error_message.exec();
      
      return false;
    }
    
  }
  else
  {
    req_activate_check.belief_expression=belief_expression;
  }
  //check if the predicate is correct
  check_belief_srv = n.serviceClient<belief_manager_msgs::CheckBeliefFormat>('/' + drone_id_namespace + '/' + check_belief);
  check_belief_srv.call(req_activate_check,res_activate_check);
  if(!res_activate_check.success)
  {
    std::cout << "The predicate is not valid \n";
    error_message.setWindowTitle(QString::fromStdString("Loading variable value"));
    error_message.setText(QString::fromStdString("The predicate is not valid.\n"));
    error_message.exec();
      
    return false;
  }
  
  req_activate.belief_expression=req_activate_check.belief_expression;
  add_belief_srv.call(req_activate,res_activate);
  
  bool service_success=res_activate.success;
  return service_success;
   
}

bool BT::AddBelief::checkBeliefVariables(std::string parameters)
{
  bool variables=false;
  for(int i=0 ; i<parameters.size() && !variables ; i++)
  {
    if(parameters[i]=='+')
    {
      variables=true;
    }
  }
  return variables;
}

std::string BT::AddBelief::substitutionVariables(std::string parameters)
{
  std::string substitution=parameters;
  std::string aux1,aux2;
  for(int i=0 ; i<substitution.size()  ; i++)
  {
    if(substitution[i]=='+')
    {
      substitution[i]=' ';
      std::string value=variables->getValue(substitution[i+1]);
      substitution[i+1]=' ';
      
      aux1=substitution.substr(0,i);
      aux2=substitution.substr(i+2);
      aux1=aux1+value+aux2;
      substitution=aux1;
    }
  }
  return substitution;
}