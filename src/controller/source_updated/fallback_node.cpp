/*!********************************************************************************
 * \brief     This is the fallback_node class. This control node returns 
 *            success or running if one of the children returns success or running.
 *            Otherwise if all children return failure it returns failure. 
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
#include <fallback_node.h>
#include <string>

BT::FallbackNode::FallbackNode(std::string name, QPixmap icono_pixmap) : ControlNode::ControlNode(name, icono_pixmap) {
  action=FALLBACK_NODE;
}

BT::FallbackNode::~FallbackNode() {}

BT::ReturnStatus BT::FallbackNode::executeStep()
{
  number_of_children=children.size();
  for(int i=0 ; i<number_of_children ; i++)
  {
    //itemPaused=this;
    sleep(2);
    execution_result=children[i]->executeStep();
    switch (execution_result)
    {
    case BT::SUCCESSFUL_COMPLETION:
      setColor(COLOR_GREEN);
      setStatus(BT::SUCCESSFUL_COMPLETION);
      return BT::SUCCESSFUL_COMPLETION;
      break;
  
    case BT::FAILURE_COMPLETION:
      //If it is the last children and it failed we set the node to FAILURE_COMPLETION
      if(i==number_of_children-1)
      {
        setColor(COLOR_RED);
        setStatus(BT::FAILURE_COMPLETION);
        return BT::FAILURE_COMPLETION;
      }
      break;
    
    case BT::RUNNING:
      return BT::RUNNING;
      break;
    
    case BT::PAUSED:
      return BT::PAUSED;
      break;
    
    default:
      break;
    }
  }
}