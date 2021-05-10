/*!********************************************************************************
 * \brief     This is the variables class, it is responsable of store the variables 
 *            of the querys and its values.
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

#include<variables.h>
#include<string>
#include<sstream>

Variables::Variables(){}
Variables::~Variables(){} 

void Variables::insert(std::pair<char,double> variable)
{
  Variables::variable_expression.insert(variable);
}

void Variables::setValue(std::pair<char,double> response)
{
  Variables::variable_expression.at(response.first)=response.second;
}
std::string Variables::getValue(char key)
{
  double value;
  value=Variables::variable_expression.at(key);
  
  std::ostringstream ss;
  ss << value;
  return ss.str();
}

void Variables::deleteVariables()
{
  variable_expression.erase( variable_expression.begin() , variable_expression.end() ); 
}

bool Variables::isEmpty()
{
  return variable_expression.empty();
}
std::string Variables::getValues()
{
  
  std::string res;
  for (auto& pair: Variables::variable_expression) 
  {
    std::ostringstream value;
    //double to string strs.str()
    value << pair.second;
    std::string key(1,pair.first);
    std::cout << value.str() << key << "\n";
    res=res+"\n"+key+":"+value.str(); 
    std::cout << res << "\n";
  }
  return res;
}
