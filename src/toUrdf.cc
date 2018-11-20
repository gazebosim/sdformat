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
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "sdf/Box.hh"
#include "sdf/Collision.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Geometry.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Link.hh"
#include "sdf/Mesh.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/Sphere.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"

namespace sdf
{

//////////////////////////////////////////////////
std::string PoseToOrigin(const std::string &_prefix,
                         const ignition::math::Pose3d &_pose)
{
  std::ostringstream stream;

  stream << _prefix << "<origin xyz='" << _pose.Pos()
         << "' rpy='" << _pose.Rot() << "'/>";

  return stream.str();
}

//////////////////////////////////////////////////
std::string GeometryStr(const std::string &_prefix, const sdf::Geometry *_geom)
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
  else if (_geom->Type() == sdf::GeometryType::MESH)
  {
    const Mesh *shape = _geom->MeshShape();
    stream << _prefix << "  <mesh>\n";
    stream << _prefix << "    <uri>" << shape->Uri() << "</uri>\n";
    stream << _prefix << "  </mesh>\n";
  }
  else if (_geom->Type() == sdf::GeometryType::EMPTY)
  {
    sdfwarn << "URDF does not support empty geometry type.\n";
  }
  else if (_geom->Type() == sdf::GeometryType::PLANE)
  {
    sdfwarn << "URDF does not support plane geometry type.\n";
  }
  else
  {
    sdfwarn << "Unknown geometry type of "
      << static_cast<int>(_geom->Type()) << ".\n";
  }


  stream << _prefix << "</geometry>";

  return stream.str();
}

//////////////////////////////////////////////////
std::string CollisionStr(const std::string &_prefix, const sdf::Link *_link,
                         uint64_t _colIndex)
{
  if (!_link)
  {
    sdferr << "Link is null\n";
    return "";
  }

  const Collision *col = _link->CollisionByIndex(_colIndex);
  if (!_link)
  {
    sdferr << "Collision is null in link[" << _link->Name() << "]\n";
    return "";
  }

  std::ostringstream stream;
  // Output the collision name
  stream << _prefix << "<collision name='" << col->Name() << "'>\n";

  // Output the collision pose in the link frame
  stream << PoseToOrigin(_prefix + "  ", *col->Pose(_link->Name())) << "\n";

  // Output the collision geometry
  stream << GeometryStr(_prefix + "  ", col->Geom()) << "\n";
  stream << _prefix << "</collision>";

  return stream.str();
}

//////////////////////////////////////////////////
std::string VisualStr(const std::string &_prefix, const sdf::Link *_link,
                      uint64_t _visIndex)
{
  if (!_link)
  {
    sdferr << "Link is null\n";
    return "";
  }

  const Visual *vis = _link->VisualByIndex(_visIndex);
  if (!_link)
  {
    sdferr << "Visual is null in link[" << _link->Name() << "]\n";
    return "";
  }

  std::ostringstream stream;

  // Output the visual name
  stream << _prefix << "<visual name='" << vis->Name() << "'>\n";

  // Output the visual pose in the link frame
  stream << PoseToOrigin(_prefix + "  ", *vis->Pose(_link->Name()))
         << std::endl;

  // Output the visual pose
  stream << GeometryStr(_prefix + "  ", vis->Geom()) << "\n";
  stream << _prefix << "</visual>";

  return stream.str();
}

//////////////////////////////////////////////////
std::string JointStr(const std::string &_prefix, const sdf::Joint *_joint,
    const sdf::Model *_model)
{
  if (!_joint)
  {
    sdferr << "Joint is null\n";
    return "";
  }

  std::string jointType = "";

  // Get the joint type
  switch (_joint->Type())
  {
    default:
    case sdf::JointType::INVALID:
      sdferr << "Joint with name[" << _joint->Name() << "] is Invalid\n";
      return "";
    case sdf::JointType::GEARBOX:
      sdferr << "Skipping joint with name[" << _joint->Name()
             << "] because gearbox joints are not supported by URDF.\n";
      return "";
    case sdf::JointType::REVOLUTE2:
      sdferr << "Skipping joint with name[" << _joint->Name()
             << "] because revolute2 joints are not supported by URDF.\n";
      return "";
    case sdf::JointType::SCREW:
      sdferr << "Skipping joint with name[" << _joint->Name()
             << "] because screw joints are not supported by URDF.\n";
      return "";
    case sdf::JointType::UNIVERSAL:
      sdferr << "Skipping joint with name[" << _joint->Name()
             << "] because universal joints are not supported by URDF.\n";
      return "";
    case sdf::JointType::REVOLUTE:
      jointType = "revolute";
      break;
    case sdf::JointType::CONTINUOUS:
      jointType = "continuous";
      break;
    case sdf::JointType::PRISMATIC:
      jointType = "prismatic";
      break;
    case sdf::JointType::FIXED:
      jointType = "fixed";
      break;
    case sdf::JointType::BALL:
      jointType = "floating";
      break;
  };

  std::ostringstream stream;

  // Output the joint name and type.
  stream << _prefix << "<joint name='" << _joint->Name() << "' type='"
         << jointType << "'>\n";

  // Output the joint pose in the parent link frame.
  ignition::math::Pose3d pose = *_joint->Pose(_joint->ParentLinkName());
  // Remove the model pose from the joint pose. URDF does not support
  // model/robot positioning.
  pose = pose - *_model->Pose();
  stream << PoseToOrigin(_prefix + "  ", pose) << std::endl;

  // Output the parent and child link names.
  stream << _prefix << "  <parent link='"
         << _joint->ParentLinkName() << "'/>\n";
  stream << _prefix << "  <child link='" << _joint->ChildLinkName() << "'/>\n";

  // Get the joint axis, and check that the pointer is valid
  const JointAxis *axis = _joint->Axis(0);
  if (axis)
  {
    // Output the axis xyz value.
    stream << _prefix << "  <axis xyz='" << axis->Xyz() << "'/>\n";

    // Output the axis limit
    stream << _prefix << "  <limit lower='" << axis->Lower() << "'\n"
           << _prefix << "         upper='" << axis->Upper() << "'\n"
           << _prefix << "         effort='" << axis->Effort() << "'\n"
           << _prefix << "         velocity='" << axis->MaxVelocity()
           << "'/>\n";

    // Output the axis dynamics
    stream << _prefix << "  <dynamics damping='" << axis->Lower() << "'\n"
           << _prefix << "            friction='" << axis->Friction()
           << "'/>\n";
  }

  stream << _prefix << "</joint>";

  return stream.str();
}

//////////////////////////////////////////////////
std::string InertialStr(const std::string &_prefix, const sdf::Link *_link)
{
  if (!_link)
  {
    sdferr << "Link is null\n";
    return "";
  }

  std::ostringstream stream;

  // Output <inertial>
  stream << _prefix << "<inertial>\n";

  // Pose of the inertial element
  stream << PoseToOrigin(_prefix + "  ", _link->Inertial().Pose()) << "\n";

  // Mass value
  stream << _prefix << "  <mass value='"
    << _link->Inertial().MassMatrix().Mass() << "'/>\n";

  // Inertia matrix
  stream << _prefix << "  <inertia\n";
  stream << _prefix << "    ixx = '"
    << _link->Inertial().MassMatrix().DiagonalMoments().X() << "'\n";
  stream << _prefix << "    iyy = '"
    << _link->Inertial().MassMatrix().DiagonalMoments().Y() << "'\n";
  stream << _prefix << "    izz = '"
    << _link->Inertial().MassMatrix().DiagonalMoments().Z() << "'\n";
  stream << _prefix << "    ixy = '"
    << _link->Inertial().MassMatrix().OffDiagonalMoments().X() << "'\n";
  stream << _prefix << "    ixz = '"
    << _link->Inertial().MassMatrix().OffDiagonalMoments().Y() << "'\n";
  stream << _prefix << "    iyz = '"
    << _link->Inertial().MassMatrix().OffDiagonalMoments().Z() << "'/>\n";
  stream << _prefix << "</inertial>\n";

  return stream.str();
}

//////////////////////////////////////////////////
std::string toUrdf(const std::string &_filename)
{
  if (_filename.empty())
  {
    sdferr << "Filename is empty\n";
    return "";
  }

  sdf::Root root;
  root.Load(_filename);
  const Model *model = root.ModelByIndex(0);

  // If there is no model, then get the first model in a world.
  if (!model)
  {
    // Iterate over each world to find the first <model> instance.
    for (uint64_t index = 0; index < root.WorldCount(); ++index)
    {
      // Get the first model in the world.
      model = root.WorldByIndex(index)->ModelByIndex(0);
      if (model)
        break;
    }

    if (!model)
      sdferr << "No <model> found in file[" << _filename << "]\n";
    return "";
  }

  return toUrdf(model);
}

//////////////////////////////////////////////////
std::string toUrdf(const sdf::Model *_model)
{
  if (!_model)
  {
    sdferr << "Model pointer is null\n";
    return "";
  }

  std::ostringstream stream;

  // Output the robot name
  stream << "<robot name='" << _model->Name() << "'>\n";

  // Process each link
  for (uint64_t linkIndex = 0; linkIndex < _model->LinkCount(); ++linkIndex)
  {
    const Link *link = _model->LinkByIndex(linkIndex);

    // Output the start of the <link> element.
    stream << "  " << "<link name='" << link->Name() << "'>\n";

    // Output the <inertial> element.
    stream << InertialStr("    ", link);

    // Output each collision.
    for (uint64_t index = 0; index < link->CollisionCount(); ++index)
      stream << CollisionStr("    ", link, index) << std::endl;

    // Output each visual.
    for (uint64_t index = 0; index < link->VisualCount(); ++index)
      stream << VisualStr("    ", link, index) << std::endl;

    // Close the link
    stream << "  " << "</link>\n";
  }

  // Output each joint
  for (uint64_t index = 0; index < _model->JointCount(); ++index)
    stream << JointStr("  ", _model->JointByIndex(index), _model) << std::endl;

  // Close the robot
  stream << "</robot>";

  return stream.str();
}
}
