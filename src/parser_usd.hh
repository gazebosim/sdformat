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

#ifndef SDFORMAT_USD2SDF_HH_
#define SDFORMAT_USD2SDF_HH_

#include <tinyxml2.h>
#include <sdf/sdf_config.h>

#include "sdf/Console.hh"
#include "sdf/Types.hh"
#include "usd_parser/parser_usd.hh"

#include "ignition/math/Pose3.hh"

#include <string>
#include <set>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
    //
    /// \brief USD to SDF converter
    ///
    /// This is now deprecated for external usage and will be removed in the next
    /// major version of libsdformat. Instead, consider using `usd::readFile` or
    /// `usd::readString`, which automatically convert URDF to SDF.
    class USD2SDF
    {
      /// \brief constructor
      public: USD2SDF();

      /// \brief destructor
      public: ~USD2SDF();

      std::string GetKeyValueAsString(tinyxml2::XMLElement* _elem);
      std::string Vector32Str(const ignition::math::Vector3d _vector);
      std::string Values2str(unsigned int _count, const double *_values);
      std::string Values2str(unsigned int _count, const int *_values);
      void AddKeyValue(tinyxml2::XMLElement *_elem, const std::string &_key,
                       const std::string &_value);
      void CreateGeometry(tinyxml2::XMLElement* _elem,
                          const Geometry * _geometry);

      void CreateVisual(tinyxml2::XMLElement *_elem, usd::LinkConstSharedPtr _link,
          std::shared_ptr<sdf::Visual> _visual, const std::string &_oldLinkName);

      void CreateVisuals(tinyxml2::XMLElement* _elem,
                         usd::LinkConstSharedPtr _link);
      void CreateLink(tinyxml2::XMLElement *_root,
                      usd::LinkConstSharedPtr _link,
                      ignition::math::Pose3d &_currentTransform);

      void CreateJoint(tinyxml2::XMLElement *_root,
                       usd::LinkConstSharedPtr _link,
                       ignition::math::Pose3d &_currentTransform);

      void CreateSDF(tinyxml2::XMLElement *_root,
                     usd::LinkConstSharedPtr _link,
                     const ignition::math::Pose3d &_transform);

      void CreateInertial(tinyxml2::XMLElement *_elem,
                          usd::LinkConstSharedPtr _link);

      void CreateCollisions(tinyxml2::XMLElement* _elem,
                            usd::LinkConstSharedPtr _link);

     void InsertSDFExtensionJoint(tinyxml2::XMLElement *_elem,
                                  const std::string &_jointName);

     void AddTransform(tinyxml2::XMLElement *_elem,
                       const ignition::math::Pose3d &_transform);

     void CopyBlob(tinyxml2::XMLElement *_src, tinyxml2::XMLElement *_blob_parent);

     /// reduced fixed joints:  check if a fixed joint should be lumped
     ///   checking both the joint type and if disabledFixedJointLumping
     ///   option is set
     bool FixedJointShouldBeReduced(usd::JointSharedPtr _jnt);

     void CreateCollision(tinyxml2::XMLElement* _elem,
                          usd::LinkConstSharedPtr _link,
                          std::shared_ptr<sdf::Collision> _collision,
                          const std::string &_oldLinkName);

      /// \brief Return true if the filename is a USD model.
      /// \param[in] _filename File to check.
      /// \return True if _filename is a URDF model.
      public: static bool IsUSD(const std::string &_filename);

      public: void read(const std::string &_filename,
        tinyxml2::XMLDocument* _sdfXmlOut);

    private:
      std::set<std::string> g_fixedJointsTransformedInRevoluteJoints;
      std::set<std::string> g_fixedJointsTransformedInFixedJoints;
      bool g_reduceFixedJoints;
      bool g_enforceLimits;
      ignition::math::Pose3d g_initialRobotPose;
      bool g_initialRobotPoseValid = false;

    };
  }
}

#endif
