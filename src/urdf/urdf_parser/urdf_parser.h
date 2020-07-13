/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#ifndef URDF_PARSER_URDF_PARSER_H
#define URDF_PARSER_URDF_PARSER_H

#include <string>
#include <map>
#include <tinyxml.h>
#include <urdf_model/model.h>
#include <urdf_model/color.h>
#include <urdf_world/types.h>

#include "exportdecl.h"

namespace urdf_export_helpers {

URDFDOM_DLLAPI std::string values2str(unsigned int count, const double *values, double (*conv)(double) = NULL);
URDFDOM_DLLAPI std::string values2str(urdf::Vector3 vec);
URDFDOM_DLLAPI std::string values2str(urdf::Rotation rot);
URDFDOM_DLLAPI std::string values2str(urdf::Color c);
URDFDOM_DLLAPI std::string values2str(double d);

}

namespace urdf{

  URDFDOM_DLLAPI ModelInterfaceSharedPtr parseURDF(const std::string &xml_string);
  URDFDOM_DLLAPI ModelInterfaceSharedPtr parseURDFFile(const std::string &path);
  URDFDOM_DLLAPI tinyxml2::XMLDocument*  exportURDF(ModelInterfaceSharedPtr &model);
  URDFDOM_DLLAPI tinyxml2::XMLDocument*  exportURDF(const ModelInterface &model);
  URDFDOM_DLLAPI bool parsePose(Pose&, tinyxml2::XMLElement*);
}

#endif
