/*!********************************************************************************
 * \brief     This is the query_belief class. This is a leaf node that asks a 
 *            query to the memory of beliefs. 
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

#include <query_belief.h>
#include <string>
#include<variables.h>

BT::QueryBelief::QueryBelief(std::string name,std::string query, QPixmap icono_pixmap) : LeafNode::LeafNode(name, icono_pixmap)
{
  action=QUERY_BELIEF;
  this->query=query;
  
  setBehaviorType(query);
  setNodeAttributes("");
  
  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1"); 
  n.param<std::string>("query_belief", query_belief, "query_belief");
}

BT::QueryBelief::~QueryBelief(){}

BT::ReturnStatus BT::QueryBelief::executeStep()
{
  
  if(status==BT::NON_INITIATED || status==BT::FAILURE_COMPLETION)
  {
    itemPaused=this;
    setColor(COLOR_BLUE);
    sleep(2);
    bool activation_result=askQueryBelief();
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

bool BT::QueryBelief::askQueryBelief()
{
  //variable obtention
  std::string belief_expression=query;
  std::vector< std::pair<char,double> > variables_pair=stringToPair(belief_expression);
  //for loop, we create pair <var_name,-1>  
  for(int i=0 ; i<variables_pair.size() ; i++)
  {
    variables->insert(variables_pair[i]);
  }
  //service call
  query_belief_srv = n.serviceClient<belief_manager_msgs::QueryBelief>('/' + drone_id_namespace + '/' + query_belief);
  req_activate.query=query;
  query_belief_srv.call(req_activate,res_activate);
  bool service_success=res_activate.success;
  if(service_success)
  {
    std::string query_result;
    query_result=res_activate.substitutions;
    //for loop, we update the value of the variables with the pair <var_name,substitution_value>
    std::vector< std::pair<char,double> > variables_match=parserResponse(query_result);
    for(int i=0 ; i<variables_match.size() ; i++)
    {
      variables->setValue(variables_match[i]);
    }
  }
  
  return service_success;
}

std::vector< std::pair<char,double> > BT::QueryBelief::stringToPair(std::string str)
{
  std::vector<std::pair<char,double>> res;
  for(int i=0; i<str.size();i++)
  {
    // initial case i.e. position(?R,(?X,?Y,?Z))
    if(str[i]=='?')
    {
      std::pair <char,double> variable (str[i+1],-1);
      res.push_back(variable);
       
    }
    //substitution case  i.e. {R:0,X:3,Y:4,Z:6}
    if(str[i]==':')
    {
      //revisar esto
      std::pair <char,double> variable (str[i-1],std::stod(str[i+1]+" "));
      res.push_back(variable);
    }
  }
  //return vector pair
  return res;
}

std::vector< std::pair<char,double> > BT::QueryBelief::parserResponse(std::string str)
{
  std::vector<std::pair<char,double>> res;
  
  std::string parsed;
  std::stringstream input_stringstream(str);

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
    res.push_back(variable);
  }
  new_variable=true;
  //return vector pair
  return res;
}