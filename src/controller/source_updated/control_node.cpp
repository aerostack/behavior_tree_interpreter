/*!********************************************************************************
 * \brief     This is the control_node class, this is the father class of
 *            control nodes such as sequence_node or fallback_node.
 *            This nodes are responsable of controlling the execution flow. 
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

#include <control_node.h>
#include <string>
#include <vector>

BT::ControlNode::ControlNode(std::string name,QPixmap icono_pixmap) : TreeNode::TreeNode(name, icono_pixmap)
{
    type = BT::CONTROL_NODE;
    setBehaviorType("");
    setNodeAttributes("");
}
BT::ControlNode::~ControlNode() {}

void BT::ControlNode::resetStatus(BT::TreeNode* node)
{
  node->setStatus(BT::NON_INITIATED);
  std::vector<BT::TreeNode*> nodes =node->getChildren();
  for(int i=0 ; i<nodes.size() ; i++)
  {
    if(!nodes[i]->getChildren().empty())
    {
      resetStatus(nodes[i]);
    }
    nodes[i]->setStatus(BT::NON_INITIATED);
  }
}

void BT::ControlNode::setChildrenColorBlack()
{
  number_of_children=children.size();
  for(int i=0 ; i<number_of_children ; i++)
  {
    children[i]->setColor(COLOR_BLACK);
    children[i]->setColorBackground("#ffffff");
  }

}
