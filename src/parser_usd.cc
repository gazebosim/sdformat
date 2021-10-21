/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include "parser_usd.hh"

#include "XmlUtils.hh"
#include "SDFExtension.hh"

using namespace sdf;

namespace sdf {
inline namespace SDF_VERSION_NAMESPACE {

  const int g_outputDecimalPrecision = 16;
  const char kLumpPrefix[] = "_fixed_joint_lump__";
  const char kVisualExt[] = "_visual";
  const char kCollisionExt[] = "_collision";

  typedef std::shared_ptr<SDFExtension> SDFExtensionPtr;

  typedef std::map<std::string, std::vector<SDFExtensionPtr> >
    StringSDFExtensionPtrMap;


  USD2SDF::USD2SDF()
  {
    g_fixedJointsTransformedInRevoluteJoints.clear();
    g_fixedJointsTransformedInFixedJoints.clear();
    g_enforceLimits = true;
    g_reduceFixedJoints = false;
    // g_extensions.clear();
  }

  ////////////////////////////////////////////////////////////////////////////////
  USD2SDF::~USD2SDF()
  {

  }

  ////////////////////////////////////////////////////////////////////////////////
  std::string USD2SDF::GetKeyValueAsString(tinyxml2::XMLElement* _elem)
  {
    std::string valueStr;
    if (_elem->Attribute("value"))
    {
      valueStr = _elem->Attribute("value");
    }
    else if (_elem->FirstChild())
    {
      // Check that this node is a XMLText
      if (_elem->FirstChild()->ToText())
      {
        valueStr = _elem->FirstChild()->Value();
      }
      else
      {
        sdferr << "Attribute value string not set\n";
      }
    }
    return trim(valueStr);
  }


  /////////////////////////////////////////////////
  /// \brief convert Vector3 to string
  /// \param[in] _vector a usd::Vector3
  /// \return a string
  std::string USD2SDF::Vector32Str(const usd::Vector3 _vector)
  {
    std::stringstream ss;
    ss << _vector.x;
    ss << " ";
    ss << _vector.y;
    ss << " ";
    ss << _vector.z;
    return ss.str();
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::string USD2SDF::Values2str(unsigned int _count, const double *_values)
  {
    std::stringstream ss;
    ss.precision(g_outputDecimalPrecision);
    for (unsigned int i = 0 ; i < _count ; ++i)
    {
      if (i > 0)
      {
        ss << " ";
      }
      if (std::fpclassify(_values[i]) == FP_ZERO)
        ss << 0;
      else
        ss << _values[i];
    }
    return ss.str();
  }

  /////////////////////////////////////////////////
  std::string USD2SDF::Values2str(unsigned int _count, const int *_values)
  {
    std::stringstream ss;
    for (unsigned int i = 0 ; i < _count ; ++i)
    {
      if (i > 0)
      {
        ss << " ";
      }
      ss << _values[i];
    }
    return ss.str();
  }

    ////////////////////////////////////////////////////////////////////////////////
    void USD2SDF::AddKeyValue(tinyxml2::XMLElement *_elem, const std::string &_key,
                     const std::string &_value)
    {
      tinyxml2::XMLElement *childElem = _elem->FirstChildElement(_key.c_str());
      if (childElem)
      {
        std::string oldValue = GetKeyValueAsString(childElem);
        if (oldValue != _value)
        {
          sdferr << "multiple inconsistent <" << _key
                  << "> exists due to fixed joint reduction"
                  << " overwriting previous value [" << oldValue
                  << "] with [" << _value << "].\n";
        }
        else
        {
           sdferr << "multiple consistent <" << _key
                  << "> exists with [" << _value
                  << "] due to fixed joint reduction.\n";
        }
        _elem->DeleteChild(childElem);  // remove old _elem
      }

      auto *doc = _elem->GetDocument();
      tinyxml2::XMLElement *ekey = doc->NewElement(_key.c_str());
      tinyxml2::XMLText *textEkey = doc->NewText(_value.c_str());
      ekey->LinkEndChild(textEkey);
      _elem->LinkEndChild(ekey);
    }

  ////////////////////////////////////////////////////////////////////////////////
  void USD2SDF::CreateGeometry(tinyxml2::XMLElement* _elem,
                      usd::GeometrySharedPtr _geometry)
  {
    sdferr << "CreateGeometry\n";

    auto* doc = _elem->GetDocument();
    tinyxml2::XMLElement *sdfGeometry = doc->NewElement("geometry");

    std::string type;
    tinyxml2::XMLElement *geometryType = nullptr;

    switch (_geometry->type)
    {
      case usd::Geometry::BOX:
        type = "box";
        {
          usd::BoxConstSharedPtr box =
            usd::dynamic_pointer_cast<usd::Box>(_geometry);
          int sizeCount = 3;
          double sizeVals[3];
          sizeVals[0] = box->dim.x;
          sizeVals[1] = box->dim.y;
          sizeVals[2] = box->dim.z;
          geometryType = doc->NewElement(type.c_str());
          AddKeyValue(geometryType, "size", Values2str(sizeCount, sizeVals));
        }
        break;
      case usd::Geometry::CYLINDER:
        type = "cylinder";
        {
          usd::CylinderConstSharedPtr cylinder =
            usd::dynamic_pointer_cast<usd::Cylinder>(_geometry);
          geometryType = doc->NewElement(type.c_str());
          AddKeyValue(geometryType, "length", Values2str(1, &cylinder->length));
          AddKeyValue(geometryType, "radius", Values2str(1, &cylinder->radius));
        }
        break;
      case usd::Geometry::SPHERE:
        type = "sphere";
        {
          usd::SphereConstSharedPtr sphere =
            usd::dynamic_pointer_cast<usd::Sphere>(_geometry);
          geometryType = doc->NewElement(type.c_str());
          AddKeyValue(geometryType, "radius", Values2str(1, &sphere->radius));
        }
        break;
      // case usd::Geometry::MESH:
      //   type = "mesh";
      //   {
      //     usd::MeshConstSharedPtr mesh =
      //       usd::dynamic_pointer_cast<usd::Mesh>(_geometry);
      //     geometryType = doc->NewElement(type.c_str());
      //     AddKeyValue(geometryType, "scale", Vector32Str(mesh->scale));
      //     // do something more to meshes
      //     {
      //       // set mesh file
      //       if (mesh->filename.empty())
      //       {
      //         sdferr << "usd2sdf: mesh geometry with no filename given.\n";
      //       }
      //
      //       // give some warning if file does not exist.
      //       // disabled while switching to uri
      //       // @todo: re-enable check
      //       // std::ifstream fin;
      //       // fin.open(mesh->filename.c_str(), std::ios::in);
      //       // fin.close();
      //       // if (fin.fail())
      //       //   sdferr << "filename referred by mesh ["
      //       //          << mesh->filename << "] does not appear to exist.\n";
      //
      //       // Convert package:// to model://,
      //       // in ROS, this will work if
      //       // the model package is in ROS_PACKAGE_PATH and has a manifest.xml
      //       // as a typical ros package does.
      //       std::string modelFilename = mesh->filename;
      //       std::string packagePrefix("package://");
      //       std::string modelPrefix("model://");
      //       size_t pos1 = modelFilename.find(packagePrefix, 0);
      //       if (pos1 != std::string::npos)
      //       {
      //         size_t repLen = packagePrefix.size();
      //         modelFilename.replace(pos1, repLen, modelPrefix);
      //         // sdferr << "ros style uri [package://] is"
      //         //   << "automatically converted: [" << modelFilename
      //         //   << "], make sure your ros package is in GAZEBO_MODEL_PATH"
      //         //   << " and switch your manifest to conform to sdf's"
      //         //   << " model database format.  See ["
      //         //   << "http://sdfsim.org/wiki/Model_database#Model_Manifest_XML"
      //         //   << "] for more info.\n";
      //       }
      //
      //       // add mesh filename
      //       AddKeyValue(geometryType, "uri", modelFilename);
      //     }
      //   }
        // break;
      default:
        sdferr << "Unknown body type: [" << static_cast<int>(_geometry->type)
                << "] skipped in geometry\n";
        break;
    }

    if (geometryType)
    {
      sdfGeometry->LinkEndChild(geometryType);
      _elem->LinkEndChild(sdfGeometry);
    }
  }


  ////////////////////////////////////////////////////////////////////////////////
  void USD2SDF::CreateVisual(tinyxml2::XMLElement *_elem, usd::LinkConstSharedPtr _link,
      usd::VisualSharedPtr _visual, const std::string &_oldLinkName)
  {
    sdferr << "CreateVisual\n";

    auto* doc = _elem->GetDocument();
    // begin create sdf visual node
    tinyxml2::XMLElement *sdfVisual = doc->NewElement("visual");

    // set its name
    if (_oldLinkName.compare(0, _link->name.size(), _link->name) == 0 ||
        _oldLinkName.empty())
    {
      sdfVisual->SetAttribute("name", _oldLinkName.c_str());
    }
    else
    {
      sdfVisual->SetAttribute("name",
          (_link->name + kLumpPrefix + _oldLinkName).c_str());
    }

    // add the visualisation transfrom
    double pose[6];
    pose[0] = _visual->origin.position.x;
    pose[1] = _visual->origin.position.y;
    pose[2] = _visual->origin.position.z;
    _visual->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
    AddKeyValue(sdfVisual, "pose", Values2str(6, pose));

    // insert geometry
    if (!_visual || !_visual->geometry)
    {
      sdferr << "usd2sdf: visual of link [" << _link->name
             << "] has no <geometry>.\n";
    }
    else
    {
      CreateGeometry(sdfVisual, _visual->geometry);
    }

    // set additional data from extensions
    // InsertSDFExtensionVisual(sdfVisual, _link->name);

    if (_visual->material)
    {
      // Refer to this comment in github to understand the ambient and diffuse
      // https://github.com/osrf/sdformat/pull/526#discussion_r623937715
      auto materialTag = sdfVisual->FirstChildElement("material");
      // If the material is not included by an extension then create it
      if (materialTag == nullptr)
      {
        AddKeyValue(sdfVisual, "material", "");
        materialTag = sdfVisual->FirstChildElement("material");
      }
      // If the specular and diffuse are defined by an extension then don't
      // use the color values
      if (materialTag->FirstChildElement("diffuse") == nullptr &&
          materialTag->FirstChildElement("ambient") == nullptr)
      {
        if (materialTag->FirstChildElement("diffuse") == nullptr)
        {
          double color_diffuse[4];
          color_diffuse[0] =
            ignition::math::clamp(_visual->material->color.r / 0.8, 0.0, 1.0);
          color_diffuse[1] =
            ignition::math::clamp(_visual->material->color.g / 0.8, 0.0, 1.0);
          color_diffuse[2] =
            ignition::math::clamp(_visual->material->color.b / 0.8, 0.0, 1.0);
          color_diffuse[3] = _visual->material->color.a;
          AddKeyValue(materialTag, "diffuse", Values2str(4, color_diffuse));
        }
        if (materialTag->FirstChildElement("ambient") == nullptr)
        {
          double color_ambient[4];
          color_ambient[0] =
            ignition::math::clamp(
              0.5 * _visual->material->color.r / 0.4, 0.0, 1.0);
          color_ambient[1] =
            ignition::math::clamp(
              0.5 * _visual->material->color.g / 0.4, 0.0, 1.0);
          color_ambient[2] =
            ignition::math::clamp(
              0.5 * _visual->material->color.b / 0.4, 0.0, 1.0);
          color_ambient[3] = _visual->material->color.a;
          AddKeyValue(materialTag, "ambient", Values2str(4, color_ambient));
        }
      }
    }
    // end create _visual node
    _elem->LinkEndChild(sdfVisual);
  }

  ////////////////////////////////////////////////////////////////////////////////
  void USD2SDF::CreateVisuals(tinyxml2::XMLElement* _elem,
                     usd::LinkConstSharedPtr _link)
  {
    sdferr << "CreateVisuals\n";

    // loop through all visuals in
    //   visual_array (urdf 0.3.x)
    //   visual_groups (urdf 0.2.x)
    // and create visual sdf blocks.
    // Note, well as additional visual from
    //   lumped meshes (fixed joint reduction)
    unsigned int visualCount = 0;
    for (std::vector<usd::VisualSharedPtr>::const_iterator
        visual = _link->visual_array.begin();
        visual != _link->visual_array.end();
        ++visual)
    {
      sdferr << "creating visual for link [" << _link->name
             << "] visual [" << (*visual)->name << "]\n";

      // visual sdf has a name if it was lumped/reduced
      // otherwise, use the link name
      std::string visualName = (*visual)->name;
      if (visualName.empty())
      {
        visualName = _link->name;
      }

      // add _visual extension
      visualName = visualName + kVisualExt;

      if (visualCount > 0)
      {
        std::ostringstream visualNameStream;
        visualNameStream << visualName
                         << "_" << visualCount;
        visualName = visualNameStream.str();
      }

      // make a <visual> block
      CreateVisual(_elem, _link, *visual, visualName);

      ++visualCount;
    }
  }


  ////////////////////////////////////////////////////////////////////////////////
  bool USD2SDF::FixedJointShouldBeReduced(usd::JointSharedPtr _jnt)
  {
      // A joint should be lumped only if its type is fixed and
      // the disabledFixedJointLumping or preserveFixedJoint
      // joint options are not set
      return (_jnt->type == usd::Joint::FIXED &&
                (g_fixedJointsTransformedInRevoluteJoints.find(_jnt->name) ==
                   g_fixedJointsTransformedInRevoluteJoints.end()) &&
                (g_fixedJointsTransformedInFixedJoints.find(_jnt->name) ==
                   g_fixedJointsTransformedInFixedJoints.end()));
  }

  ////////////////////////////////////////////////////////////////////////////////
  ignition::math::Pose3d USD2SDF::CopyPose(usd::Pose _pose)
  {
    ignition::math::Pose3d p;
    p.Pos().X() = _pose.position.x;
    p.Pos().Y() = _pose.position.y;
    p.Pos().Z() = _pose.position.z;
    p.Rot().X() = _pose.rotation.x;
    p.Rot().Y() = _pose.rotation.y;
    p.Rot().Z() = _pose.rotation.z;
    p.Rot().W() = _pose.rotation.w;
    return p;
  }

  ////////////////////////////////////////////////////////////////////////////////
  usd::Pose USD2SDF::CopyPose(ignition::math::Pose3d _pose)
  {
    usd::Pose p;
    p.position.x = _pose.Pos().X();
    p.position.y = _pose.Pos().Y();
    p.position.z = _pose.Pos().Z();
    p.rotation.x = _pose.Rot().X();
    p.rotation.y = _pose.Rot().Y();
    p.rotation.z = _pose.Rot().Z();
    p.rotation.w = _pose.Rot().W();
    return p;
  }


  ////////////////////////////////////////////////////////////////////////////////
  void USD2SDF::InsertSDFExtensionJoint(tinyxml2::XMLElement *_elem,
                               const std::string &_jointName)
  {
    // auto* doc = _elem->GetDocument();
    // for (StringSDFExtensionPtrMap::iterator
    //     sdfIt = g_extensions.begin();
    //     sdfIt != g_extensions.end(); ++sdfIt)
    // {
    //   if (sdfIt->first == _jointName)
    //   {
    //     for (std::vector<SDFExtensionPtr>::iterator
    //         ge = sdfIt->second.begin();
    //         ge != sdfIt->second.end(); ++ge)
    //     {
    //       tinyxml2::XMLElement *physics = _elem->FirstChildElement("physics");
    //       bool newPhysics = false;
    //       if (physics == nullptr)
    //       {
    //         physics = doc->NewElement("physics");
    //         newPhysics = true;
    //       }
    //
    //       tinyxml2::XMLElement *physicsOde = physics->FirstChildElement("ode");
    //       bool newPhysicsOde = false;
    //       if (physicsOde == nullptr)
    //       {
    //         physicsOde = doc->NewElement("ode");
    //         newPhysicsOde = true;
    //       }
    //
    //       tinyxml2::XMLElement *limit = physicsOde->FirstChildElement("limit");
    //       bool newLimit = false;
    //       if (limit == nullptr)
    //       {
    //         limit = doc->NewElement("limit");
    //         newLimit = true;
    //       }
    //
    //       tinyxml2::XMLElement *axis = _elem->FirstChildElement("axis");
    //       bool newAxis = false;
    //       if (axis == nullptr)
    //       {
    //         axis = doc->NewElement("axis");
    //         newAxis = true;
    //       }
    //
    //       tinyxml2::XMLElement *dynamics = axis->FirstChildElement("dynamics");
    //       bool newDynamics = false;
    //       if (dynamics == nullptr)
    //       {
    //         dynamics = doc->NewElement("dynamics");
    //         newDynamics = true;
    //       }
    //
    //       // insert stopCfm, stopErp, fudgeFactor
    //       if ((*ge)->isStopCfm)
    //       {
    //         AddKeyValue(limit, "cfm", Values2str(1, &(*ge)->stopCfm));
    //       }
    //       if ((*ge)->isStopErp)
    //       {
    //         AddKeyValue(limit, "erp", Values2str(1, &(*ge)->stopErp));
    //       }
    //       if ((*ge)->isSpringReference)
    //       {
    //         AddKeyValue(dynamics, "spring_reference",
    //                     Values2str(1, &(*ge)->springReference));
    //       }
    //       if ((*ge)->isSpringStiffness)
    //       {
    //         AddKeyValue(dynamics, "spring_stiffness",
    //                     Values2str(1, &(*ge)->springStiffness));
    //       }
    //
    //       // insert provideFeedback
    //       if ((*ge)->isProvideFeedback)
    //       {
    //         if ((*ge)->provideFeedback)
    //         {
    //           AddKeyValue(physics, "provide_feedback", "true");
    //           AddKeyValue(physicsOde, "provide_feedback", "true");
    //         }
    //         else
    //         {
    //           AddKeyValue(physics, "provide_feedback", "false");
    //           AddKeyValue(physicsOde, "provide_feedback", "false");
    //         }
    //       }
    //
    //       // insert implicitSpringDamper
    //       if ((*ge)->isImplicitSpringDamper)
    //       {
    //         if ((*ge)->implicitSpringDamper)
    //         {
    //           AddKeyValue(physicsOde, "implicit_spring_damper", "true");
    //           /// \TODO: deprecating cfm_damping, transitional tag below
    //           AddKeyValue(physicsOde, "cfm_damping", "true");
    //         }
    //         else
    //         {
    //           AddKeyValue(physicsOde, "implicit_spring_damper", "false");
    //           /// \TODO: deprecating cfm_damping, transitional tag below
    //           AddKeyValue(physicsOde, "cfm_damping", "false");
    //         }
    //       }
    //
    //       // insert fudgeFactor
    //       if ((*ge)->isFudgeFactor)
    //       {
    //         AddKeyValue(physicsOde, "fudge_factor",
    //                     Values2str(1, &(*ge)->fudgeFactor));
    //       }
    //
    //       if (newDynamics)
    //       {
    //         axis->LinkEndChild(dynamics);
    //       }
    //       if (newAxis)
    //       {
    //         _elem->LinkEndChild(axis);
    //       }
    //
    //       if (newLimit)
    //       {
    //         physicsOde->LinkEndChild(limit);
    //       }
    //       if (newPhysicsOde)
    //       {
    //         physics->LinkEndChild(physicsOde);
    //       }
    //       if (newPhysics)
    //       {
    //         _elem->LinkEndChild(physics);
    //       }
    //
    //       // insert all additional blobs into joint
    //       for (auto blobIt = (*ge)->blobs.begin();
    //           blobIt != (*ge)->blobs.end(); ++blobIt)
    //       {
    //         CopyBlob((*blobIt)->FirstChildElement(), _elem);
    //       }
    //     }
    //   }
    // }
  }

  void USD2SDF::CopyBlob(tinyxml2::XMLElement *_src, tinyxml2::XMLElement *_blob_parent)
  {
    if (_blob_parent == nullptr)
    {
      sdferr << "blob parent is null\n";
      return;
    }

    tinyxml2::XMLNode *clone = DeepClone(_blob_parent->GetDocument(), _src);
    if (clone == nullptr)
    {
      sdferr << "Unable to deep copy blob\n";
    }
    else
    {
      _blob_parent->LinkEndChild(clone);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  void USD2SDF::AddTransform(tinyxml2::XMLElement *_elem,
                    const ignition::math::Pose3d &_transform)
  {
    ignition::math::Vector3d e = _transform.Rot().Euler();
    double cpose[6] = { _transform.Pos().X(), _transform.Pos().Y(),
                        _transform.Pos().Z(), e.X(), e.Y(), e.Z() };

    // set geometry transform
    AddKeyValue(_elem, "pose", Values2str(6, cpose));
  }



  ////////////////////////////////////////////////////////////////////////////////
  void USD2SDF::CreateCollision(tinyxml2::XMLElement* _elem,
                       usd::LinkConstSharedPtr _link,
                       usd::CollisionSharedPtr _collision,
                       const std::string &_oldLinkName)
  {
    auto* doc = _elem->GetDocument();
    // begin create geometry node, skip if no collision specified
    tinyxml2::XMLElement *sdfCollision = doc->NewElement("collision");

    // std::cerr << "CreateCollision link [" << _link->name
    //           << "] old [" << _oldLinkName
    //           << "]\n";
    // set its name, if lumped, add original link name
    // for meshes in an original mesh, it's likely
    // _link->name + mesh count
    if (_oldLinkName.compare(0, _link->name.size(), _link->name) == 0 ||
        _oldLinkName.empty())
    {
      sdfCollision->SetAttribute("name", _oldLinkName.c_str());
    }
    else
    {
      sdfCollision->SetAttribute("name", (_link->name
          + kLumpPrefix + _oldLinkName).c_str());
    }

    // std::cerr << "collision [" << sdfCollision->Attribute("name") << "]\n";

    // set transform
    double pose[6];
    pose[0] = _collision->origin.position.x;
    pose[1] = _collision->origin.position.y;
    pose[2] = _collision->origin.position.z;
    _collision->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
    AddKeyValue(sdfCollision, "pose", Values2str(6, pose));

    // add geometry block
    if (!_collision || !_collision->geometry)
    {
      sdfdbg << "urdf2sdf: collision of link [" << _link->name
             << "] has no <geometry>.\n";
    }
    else
    {
      CreateGeometry(sdfCollision, _collision->geometry);
    }

    // set additional data from extensions
    // InsertSDFExtensionCollision(sdfCollision, _link->name);

    // add geometry to body
    _elem->LinkEndChild(sdfCollision);
  }

  ////////////////////////////////////////////////////////////////////////////////
  void USD2SDF::CreateCollisions(tinyxml2::XMLElement* _elem,
                        usd::LinkConstSharedPtr _link)
  {
    // loop through all collisions in
    //   collision_array (urdf 0.3.x)
    //   collision_groups (urdf 0.2.x)
    // and create collision sdf blocks.
    // Note, well as additional collision from
    //   lumped meshes (fixed joint reduction)
    unsigned int collisionCount = 0;
    for (std::vector<usd::CollisionSharedPtr>::const_iterator
        collision = _link->collision_array.begin();
        collision != _link->collision_array.end();
        ++collision)
    {
      sdfdbg << "creating collision for link [" << _link->name
             << "] collision [" << (*collision)->name << "]\n";

      // collision sdf has a name if it was lumped/reduced
      // otherwise, use the link name
      std::string collisionName = (*collision)->name;
      if (collisionName.empty())
      {
        collisionName = _link->name;
      }

      // add _collision extension
      collisionName = collisionName + kCollisionExt;

      if (collisionCount > 0)
      {
        std::ostringstream collisionNameStream;
        collisionNameStream << collisionName
                            << "_" << collisionCount;
        collisionName = collisionNameStream.str();
      }

      // make a <collision> block
      CreateCollision(_elem, _link, *collision, collisionName);

      ++collisionCount;
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  void USD2SDF::CreateInertial(tinyxml2::XMLElement *_elem,
                      usd::LinkConstSharedPtr _link)
  {
    auto* doc = _elem->GetDocument();
    tinyxml2::XMLElement *inertial = doc->NewElement("inertial");

    // set mass properties
    // check and print a warning message
    double roll, pitch, yaw;
    _link->inertial->origin.rotation.getRPY(roll, pitch, yaw);

    /// add pose
    ignition::math::Pose3d pose = CopyPose(_link->inertial->origin);
    AddTransform(inertial, pose);

    // add mass
    AddKeyValue(inertial, "mass",
                Values2str(1, &_link->inertial->mass));

    // add inertia (ixx, ixy, ixz, iyy, iyz, izz)
    tinyxml2::XMLElement *inertia = doc->NewElement("inertia");
    AddKeyValue(inertia, "ixx",
                Values2str(1, &_link->inertial->ixx));
    AddKeyValue(inertia, "ixy",
                Values2str(1, &_link->inertial->ixy));
    AddKeyValue(inertia, "ixz",
                Values2str(1, &_link->inertial->ixz));
    AddKeyValue(inertia, "iyy",
                Values2str(1, &_link->inertial->iyy));
    AddKeyValue(inertia, "iyz",
                Values2str(1, &_link->inertial->iyz));
    AddKeyValue(inertia, "izz",
                Values2str(1, &_link->inertial->izz));
    inertial->LinkEndChild(inertia);

    _elem->LinkEndChild(inertial);
  }

  ////////////////////////////////////////////////////////////////////////////////
  void USD2SDF::CreateJoint(tinyxml2::XMLElement *_root,
                   usd::LinkConstSharedPtr _link,
                   ignition::math::Pose3d &/*_currentTransform*/)
  {
    sdferr << "createJoint" << '\n';
    // compute the joint tag
    std::string jtype;
    jtype.clear();
    std::cerr << "_link->parent_joint != nullptr " << (_link->parent_joint == nullptr) << '\n';
    if (_link->parent_joint != nullptr)
    {
      switch (_link->parent_joint->type)
      {
        case usd::Joint::CONTINUOUS:
        case usd::Joint::REVOLUTE:
          jtype = "revolute";
          break;
        case usd::Joint::PRISMATIC:
          jtype = "prismatic";
          break;
        case usd::Joint::FLOATING:
        case usd::Joint::PLANAR:
          break;
        case usd::Joint::FIXED:
          jtype = "fixed";
          break;
        default:
          sdferr << "Unknown joint type: ["
                  << static_cast<int>(_link->parent_joint->type)
                  << "] in link [" << _link->name << "]\n";
          break;
      }
    }

    // If this is a fixed joint and the legacy option disableFixedJointLumping
    // is present and the new option preserveFixedJoint is not, then the fixed
    // joint should be converted to a revolute joint with max and mim position
    // limits set to (0, 0) for backward compatibility
    bool fixedJointConvertedToRevoluteJoint = false;
    if (jtype == "fixed")
    {
      fixedJointConvertedToRevoluteJoint =
        (g_fixedJointsTransformedInRevoluteJoints.find(_link->parent_joint->name)
         != g_fixedJointsTransformedInRevoluteJoints.end());
    }

    // skip if joint type is fixed and it is lumped
    //   skip/return with the exception of root link being world,
    //   because there's no lumping there
    // std::cerr << "FixedJointShouldBeReduced(_link->parent_joint) " << FixedJointShouldBeReduced(_link->parent_joint) << '\n';
    if (_link->getParent() && _link->getParent()->name != "/World/world"
        && FixedJointShouldBeReduced(_link->parent_joint)
        && g_reduceFixedJoints)
    {
      std::cerr << "return ?" << '\n';
      return;
    }

    if (!jtype.empty())
    {
      auto* doc = _root->GetDocument();
      tinyxml2::XMLElement *joint = doc->NewElement("joint");
      if (jtype == "fixed" && fixedJointConvertedToRevoluteJoint)
      {
        joint->SetAttribute("type", "revolute");
      }
      else
      {
        joint->SetAttribute("type", jtype.c_str());
      }
      joint->SetAttribute("name", _link->parent_joint->name.c_str());
      // Add joint pose relative to parent link
      AddTransform(
          joint, CopyPose(_link->parent_joint->parent_to_joint_origin_transform));
      auto pose = joint->FirstChildElement("pose");
      std::string relativeToAttr = _link->getParent()->name;
      if ("/World/world" == relativeToAttr )
      {
        relativeToAttr = "__model__";
      }
      pose->SetAttribute("relative_to", relativeToAttr.c_str());

      AddKeyValue(joint, "parent", _link->getParent()->name);
      AddKeyValue(joint, "child", _link->name);

      tinyxml2::XMLElement *jointAxis = doc->NewElement("axis");
      tinyxml2::XMLElement *jointAxisLimit = doc->NewElement("limit");
      tinyxml2::XMLElement *jointAxisDynamics = doc->NewElement("dynamics");
      if (jtype == "fixed" && fixedJointConvertedToRevoluteJoint)
      {
        AddKeyValue(jointAxisLimit, "lower", "0");
        AddKeyValue(jointAxisLimit, "upper", "0");
        AddKeyValue(jointAxisDynamics, "damping", "0");
        AddKeyValue(jointAxisDynamics, "friction", "0");
      }
      else if (jtype != "fixed")
      {
        double jointAxisXyzArray[3] =
        { _link->parent_joint->axis.x,
          _link->parent_joint->axis.y,
          _link->parent_joint->axis.z};
        AddKeyValue(jointAxis, "xyz",
                    Values2str(3, jointAxisXyzArray));
        if (_link->parent_joint->dynamics)
        {
          AddKeyValue(jointAxisDynamics, "damping",
                      Values2str(1, &_link->parent_joint->dynamics->damping));
          AddKeyValue(jointAxisDynamics, "friction",
                      Values2str(1, &_link->parent_joint->dynamics->friction));
        }

        if (g_enforceLimits && _link->parent_joint->limits)
        {
          if (jtype == "slider")
          {
            AddKeyValue(jointAxisLimit, "lower",
                        Values2str(1, &_link->parent_joint->limits->lower));
            AddKeyValue(jointAxisLimit, "upper",
                        Values2str(1, &_link->parent_joint->limits->upper));
          }
          else if (_link->parent_joint->type != usd::Joint::CONTINUOUS)
          {
            double *lowstop  = &_link->parent_joint->limits->lower;
            double *highstop = &_link->parent_joint->limits->upper;
            // enforce ode bounds, this will need to be fixed
            if (*lowstop > *highstop)
            {
              sdferr << "usd2sdf: revolute joint ["
                      << _link->parent_joint->name
                      << "] with limits: lowStop[" << *lowstop
                      << "] > highStop[" << *highstop
                      << "], switching the two.\n";
              double tmp = *lowstop;
              *lowstop = *highstop;
              *highstop = tmp;
            }
            AddKeyValue(jointAxisLimit, "lower",
                        Values2str(1, &_link->parent_joint->limits->lower));
            AddKeyValue(jointAxisLimit, "upper",
                        Values2str(1, &_link->parent_joint->limits->upper));
          }
          AddKeyValue(jointAxisLimit, "effort",
                      Values2str(1, &_link->parent_joint->limits->effort));
          AddKeyValue(jointAxisLimit, "velocity",
                      Values2str(1, &_link->parent_joint->limits->velocity));
        }
      }

      if (jtype == "fixed" && !fixedJointConvertedToRevoluteJoint)
      {
        doc->DeleteNode(jointAxisLimit);
        jointAxisLimit = 0;
        doc->DeleteNode(jointAxisDynamics);
        jointAxisDynamics = 0;
        doc->DeleteNode(jointAxis);
        jointAxis = 0;
      }
      else
      {
        jointAxis->LinkEndChild(jointAxisLimit);
        jointAxis->LinkEndChild(jointAxisDynamics);
        joint->LinkEndChild(jointAxis);
      }

      // copy sdf extensions data
      InsertSDFExtensionJoint(joint, _link->parent_joint->name);

      // add joint to document
      _root->LinkEndChild(joint);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  void USD2SDF::CreateLink(tinyxml2::XMLElement *_root,
                  usd::LinkConstSharedPtr _link,
                  ignition::math::Pose3d &_currentTransform)
  {
    sdferr << "CreateLink\n";

    // create new body
    tinyxml2::XMLElement *elem = _root->GetDocument()->NewElement("link");

    // set body name
    elem->SetAttribute("name", _link->name.c_str());

    // compute global transform
    ignition::math::Pose3d localTransform;
    // this is the transform from parent link to current _link
    // this transform does not exist for the root link
    if (_link->parent_joint)
    {
      tinyxml2::XMLElement *pose = _root->GetDocument()->NewElement("pose");
      pose->SetAttribute("relative_to", _link->parent_joint->name.c_str());
      elem->LinkEndChild(pose);
    }
    else
    {
      sdferr << "[" << _link->name << "] has no parent joint\n";

      if (_currentTransform != ignition::math::Pose3d::Zero)
      {
        // create origin tag for this element
        AddTransform(elem, _currentTransform);
      }
    }

    // create new inerial block
    CreateInertial(elem, _link);

    // create new collision block
    CreateCollisions(elem, _link);

    // create new visual block
    CreateVisuals(elem, _link);

    // make a <joint:...> block
    CreateJoint(_root, _link, _currentTransform);

    // add body to document
    _root->LinkEndChild(elem);
  }

  ////////////////////////////////////////////////////////////////////////////////
  void USD2SDF::CreateSDF(tinyxml2::XMLElement *_root,
                 usd::LinkConstSharedPtr _link,
                 const ignition::math::Pose3d &_transform)
  {
    sdferr << "createSDF\n";
    ignition::math::Pose3d _currentTransform = _transform;

    // must have an <inertial> block and cannot have zero mass.
    //  allow det(I) == zero, in the case of point mass geoms.
    // @todo:  keyword "/World/world" should be a constant defined somewhere else
    if (_link->name != "/World/world" &&
        ((!_link->inertial) || ignition::math::equal(_link->inertial->mass, 0.0)))
    {
      if (!_link->child_links.empty())
      {
        sdferr << "usd2sdf: link[" << _link->name
               << "] has no inertia, ["
               << static_cast<int>(_link->child_links.size())
               << "] children links ignored.\n";
      }

      if (!_link->child_joints.empty())
      {
        sdferr << "usd2sdf: link[" << _link->name
               << "] has no inertia, ["
               << static_cast<int>(_link->child_links.size())
               << "] children joints ignored.\n";
      }

      if (_link->parent_joint)
      {
        sdferr << "usd2sdf: link[" << _link->name
               << "] has no inertia, "
               << "parent joint [" << _link->parent_joint->name
               << "] ignored.\n";
      }

      sdferr << "usd2sdf: link[" << _link->name
             << "] has no inertia, not modeled in sdf\n";
      return;
    }

    // create <body:...> block for non fixed joint attached bodies
    if ((_link->getParent() && _link->getParent()->name == "/World/world") ||
        !g_reduceFixedJoints ||
        (!_link->parent_joint ||
         !FixedJointShouldBeReduced(_link->parent_joint)))
    {
      CreateLink(_root, _link, _currentTransform);
    }

    // recurse into children
    for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
    {
      std::cerr << "recurse into children " << _link->name << '\n';
      CreateSDF(_root, _link->child_links[i], _currentTransform);
    }
    sdferr << "createSDF end\n";

  }

  ////////////////////////////////////////////////////////////////////////////////
  bool USD2SDF::IsUSD(const std::string &_filename)
  {
    sdferr << "IsUSD" << '\n';
    usd::ModelInterfaceSharedPtr robotModel = usd::parseUSDFile(_filename);
    sdferr << "IsUSD2" << '\n';
    return robotModel != nullptr;
  }

  //parser_urdf.cc InitModelString
  void USD2SDF::read(const std::string &_filename,
    tinyxml2::XMLDocument* _sdfXmlOut)
  {
    usd::ModelInterfaceSharedPtr robotModel = usd::parseUSDFile(_filename);
    if (!robotModel)
    {
      sdferr << "Unable to call parseURDF on robot model\n";
      return;
    }
    // create root element and define needed namespaces
    tinyxml2::XMLElement *robot = _sdfXmlOut->NewElement("model");

    // set model name to urdf robot name if not specified
    robot->SetAttribute("name", robotModel->getName().c_str());

    usd::LinkConstSharedPtr rootLink = robotModel->getRoot();
    tinyxml2::XMLElement *sdf;

    ignition::math::Pose3d transform;

    // g_extensions.clear();
    g_fixedJointsTransformedInFixedJoints.clear();
    g_fixedJointsTransformedInRevoluteJoints.clear();
    g_initialRobotPose.position = usd::Vector3(0, 0, 0);
    g_initialRobotPose.rotation.setFromRPY(0, 0, 0);
    g_initialRobotPoseValid = true;

    std::cerr << "rootLink->name " << rootLink->name << '\n';

    if (rootLink->name == "/World/world")
    {
      // convert all children link
      for (std::vector<usd::LinkSharedPtr>::const_iterator
          child = rootLink->child_links.begin();
          child != rootLink->child_links.end(); ++child)
      {
        std::cerr << "child->name " << (*child)->name << '\n';

        CreateSDF(robot, (*child), transform);
      }
    }
    else
    {
      // convert, starting from root link
      CreateSDF(robot, rootLink, transform);
    }

    sdf = _sdfXmlOut->NewElement("sdf");
    sdf->SetAttribute("version", "1.7");
    sdf->LinkEndChild(robot);

    _sdfXmlOut->LinkEndChild(sdf);
  }
}
}
