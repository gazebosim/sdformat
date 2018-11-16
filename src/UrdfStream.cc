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

#include "sdf/Box.hh"
#include "sdf/Collision.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Geometry.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Sphere.hh"
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

    stream << "    " << "<inertial>\n";

    stream << this->PoseToOrigin("      ", link->Inertial().Pose()) << "\n";

    stream << "      " << "<mass value='"
      << link->Inertial().MassMatrix().Mass() << "'/>\n";
    stream << "      " << "<inertia\n";
    stream << "        ixx = '"
      << link->Inertial().MassMatrix().DiagonalMoments().X() << "'\n";
    stream << "        iyy = '"
      << link->Inertial().MassMatrix().DiagonalMoments().Y() << "'\n";
    stream << "        izz = '"
      << link->Inertial().MassMatrix().DiagonalMoments().Z() << "'\n";
    stream << "        ixy = '"
      << link->Inertial().MassMatrix().OffDiagonalMoments().X() << "'\n";
    stream << "        ixz = '"
      << link->Inertial().MassMatrix().OffDiagonalMoments().Y() << "'\n";
    stream << "        iyz = '"
      << link->Inertial().MassMatrix().OffDiagonalMoments().Z() << "'/>\n";
    stream << "    " << "</inertial>\n";

    // Output each collision
    for (uint64_t colIndex = 0; colIndex < link->CollisionCount(); ++colIndex)
    {
      const Collision *col = link->CollisionByIndex(colIndex);

      stream << "    " << "<collision name='" << col->Name() << "'>\n";
      stream << this->PoseToOrigin("      ", *col->Pose(link->Name())) << "\n";
      stream << this->Geometry("      ", col->Geom()) << "\n";
      stream << "    " << "</collision>\n";
    }

    // Output each visual
    for (uint64_t visIndex = 0; visIndex < link->VisualCount(); ++visIndex)
    {
      const Visual *vis = link->VisualByIndex(visIndex);

      stream << "    " << "<visual name='" << vis->Name() << "'>\n";
      stream << this->PoseToOrigin("      ", *vis->Pose(link->Name())) << "\n";
      stream << this->Geometry("      ", vis->Geom()) << "\n";
      stream << "    " << "</visual>\n";
    }

    // Close the link
    stream << "  " << "</link>\n";
  }

  // Output each joint
  for (uint64_t jntIndex = 0; jntIndex < _model.JointCount(); ++jntIndex)
  {
    const Joint *joint = _model.JointByIndex(jntIndex);

    stream << "  " << "<joint name='" << joint->Name() << "'>\n";
    stream << "  " << "</joint>\n";
  }

  // Close the robot
  stream << "</robot>";

  std::cout << stream.str() << std::endl;
}

//////////////////////////////////////////////////
std::string UrdfStream::String() const
{
  return "";
}

//////////////////////////////////////////////////
std::string UrdfStream::PoseToOrigin(const std::string &_prefix,
    const ignition::math::Pose3d &_pose) const
{
  std::ostringstream stream;

  stream << _prefix << "<origin xyz='"
    << _pose.Pos().X() << " "
    << _pose.Pos().Y() << " "
    << _pose.Pos().Z() << "' rpy='"
    << _pose.Rot().Euler().X() << " "
    << _pose.Rot().Euler().Y() << " "
    << _pose.Rot().Euler().Z() << "'/>";

  return stream.str();
}

//////////////////////////////////////////////////
std::string UrdfStream::Geometry(const std::string &_prefix,
    const sdf::Geometry *_geom) const
{
  std::ostringstream stream;
  stream << _prefix << "<geometry>\n";

  if (_geom->Type() == sdf::GeometryType::BOX)
  {
    const Box *shape = _geom->BoxShape();
    stream << _prefix << "  <box size='" << shape->Size() << "'/>\n";
  }
  else if (_geom->Type() == sdf::GeometryType::CYLINDER)
  {
    const Cylinder *shape = _geom->CylinderShape();
    stream << _prefix << "  <cylinder radius='" << shape->Radius()
           << "' length='" << shape->Length() << "'/>\n";
  }
  else if (_geom->Type() == sdf::GeometryType::SPHERE)
  {
    const Sphere *shape = _geom->SphereShape();
    stream << _prefix << "  <sphere radius='" << shape->Radius() << "'/>\n";
  }

  stream << _prefix << "</geometry>\n";

  return stream.str();
}
