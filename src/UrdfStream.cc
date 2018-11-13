/*
 * Copyright 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "sdf/Collision.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Visual.hh"
#include "sdf/UrdfStream.hh"

using namespace sdf;

//////////////////////////////////////////////////
UrdfStream::UrdfStream(const sdf::Model &_model)
{
  std::ostringstream stream;
  stream << "<robot name='" << _model.Name() << "'>\n";

  // Process each link
  for (uint64_t linkIndex = 0; linkIndex < _model.LinkCount(); ++linkIndex)
  {
    const Link *link =_model.LinkByIndex(linkIndex);

    stream << "  " << "<link name='" << link->Name() << "'>\n";

    // Output each collision
    for (uint64_t colIndex = 0; colIndex < link->CollisionCount(); ++colIndex)
    {
      const Collision *col = link->CollisionByIndex(colIndex);

      stream << "    " << "<collision name='" << col->Name() << "'>\n";
      stream << "    " << "</collision>\n";
    }

    // Output each visual
    for (uint64_t visIndex = 0; visIndex < link->VisualCount(); ++visIndex)
    {
      const Visual *vis = link->VisualByIndex(visIndex);

      stream << "    " << "<visual name='" << vis->Name() << "'>\n";
      stream << "    " << "</visual>\n";
    }

    // Close the link
    stream << "  " << "</link>\n";
  }

  // Output each joint
  for (uint64_t jntIndex = 0; jntIndex < _model.JointCount(); ++jntIndex)
  {
    const Joint *joint = _model.JointByIndex(jntIndex);

    stream << "    " << "<joint name='" << joint->Name() << "'>\n";
    stream << "    " << "</joint>\n";
  }

  // Close the robot
  stream << "</robot>";

  std::cout << stream.str() << std::endl;
}

//////////////////////////////////////////////////
std::string UrdfStream::String() const
{
}
