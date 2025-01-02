/*
 * Copyright 2015 Open Source Robotics Foundation
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

#ifndef SDFORMAT_SDFEXTENSION_HH_
#define SDFORMAT_SDFEXTENSION_HH_

#include <tinyxml2.h>

#include <memory>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>

#include <sdf/config.hh>
#include "sdf/Types.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  using XMLDocumentPtr = std::shared_ptr<tinyxml2::XMLDocument>;
  using XMLElementPtr = std::shared_ptr<tinyxml2::XMLElement>;

  /// \internal
  /// \brief A class for holding sdf extension elements in urdf
  class SDFExtension
  {
    /// \brief Constructor
    public: SDFExtension();

    // for reducing fixed joints and removing links
    public: std::string oldLinkName;
    public: gz::math::Pose3d reductionTransform;

    // visual material
    public: std::string material;

    /// \brief blobs of xml to be copied into the visual sdf element
    public: std::vector<XMLDocumentPtr> visual_blobs;

    /// \brief blobs of xml to be copied into the collision sdf element
    /// An example might be:
    /// <gazebo reference="link-1">
    ///   <collision>
    ///     <max_contacts>10</max_contacts>
    ///     <surface>
    ///       <contact>
    ///         <ode>
    ///           <kp>1e+06</kp>
    ///           <kd>100</kd>
    ///           <max_vel>100</max_vel>
    ///           <min_depth>0.001</min_depth>
    ///         </ode>
    ///       </contact>
    ///       <friction>
    ///         <ode>
    ///           <mu>1</mu>
    ///           <mu2>1</mu2>
    ///         </ode>
    ///       </friction>
    ///     </surface>
    ///   </collision>
    /// </gazebo>
    /// where all the contents of `<collision>` element is copied into the
    /// resulting collision sdf.
    public: std::vector<XMLDocumentPtr> collision_blobs;

    // body, default off
    public: bool isSetStaticFlag;
    public: bool setStaticFlag;
    public: bool isGravity;
    public: bool gravity;
    public: bool isDampingFactor;
    public: double dampingFactor;
    public: bool isMaxContacts;
    public: int maxContacts;
    public: bool isMaxVel;
    public: double maxVel;
    public: bool isMinDepth;
    public: double minDepth;
    public: bool isSelfCollide;
    public: bool selfCollide;

    // geom, contact dynamics
    public: bool isMu1, isMu2, isKp, isKd;
    public: double mu1, mu2, kp, kd;
    public: std::string fdir1;
    public: bool isLaserRetro;
    public: double laserRetro;

    // joint, joint limit dynamics
    public: bool isStopCfm, isStopErp, isFudgeFactor;
    public: double stopCfm, stopErp, fudgeFactor;
    public: bool isSpringReference, isSpringStiffness;
    public: double springReference, springStiffness;
    public: bool isProvideFeedback;
    public: bool provideFeedback;
    public: bool isImplicitSpringDamper;
    public: bool implicitSpringDamper;

    // blobs into body or robot
    public: std::vector<XMLDocumentPtr> blobs;

    friend class URDF2SDF;
  };
  }
}
#endif
