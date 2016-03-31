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

#ifndef _SDFORMAT_SDFEXTENSION_HH_
#define _SDFORMAT_SDFEXTENSION_HH_

#include <tinyxml.h>
#include <memory>
#include <string>
#include <vector>
#include <ignition/math/Pose3.hh>

/// \todo Remove this diagnositic push/pop in version 5
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include "sdf/Types.hh"
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

namespace sdf
{
  /// \internal
  /// \brief A class for holding sdf extension elements in urdf
  class SDFExtension
  {
    /// \brief Constructor
    public: SDFExtension();

    /// \brief Copy constructor
    /// \param[in] _ge SDFExtension to copy.
    public: SDFExtension(const SDFExtension &_ge);

    /// \brief Destructor
    public: virtual ~SDFExtension() = default;

    // for reducing fixed joints and removing links
    public: std::string oldLinkName;
    public: ignition::math::Pose3d reductionTransform;

    // visual material
    public: std::string material;

    /// \brief blobs of xml to be copied into the visual sdf element
    public: std::vector<std::shared_ptr<TiXmlElement> > visual_blobs;

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
    public: std::vector<std::shared_ptr<TiXmlElement> > collision_blobs;

    // body, default off
    public: bool setStaticFlag;
    public: bool gravity;
    public: bool isDampingFactor;
    public: double dampingFactor;
    public: bool isMaxContacts;
    public: int maxContacts;
    public: bool isMaxVel;
    public: double maxVel;
    public: bool isMinDepth;
    public: double minDepth;
    public: bool selfCollide;

    // geom, contact dynamics
    public: bool isMu1, isMu2, isKp, isKd;
    public: double mu1, mu2, kp, kd;
    public: std::string fdir1;
    public: bool isLaserRetro;
    public: double laserRetro;

    // joint, joint limit dynamics
    public: bool isStopCfm, isStopErp, isInitialJointPosition, isFudgeFactor;
    public: double stopCfm, stopErp, initialJointPosition, fudgeFactor;
    public: bool isSpringReference, isSpringStiffness;
    public: double springReference, springStiffness;
    public: bool isProvideFeedback;
    public: bool provideFeedback;
    public: bool isImplicitSpringDamper;
    public: bool implicitSpringDamper;
    public: bool isStopKp, isStopKd;
    public: double stopKp, stopKd;

    // blobs into body or robot
    public: std::vector<std::shared_ptr<TiXmlElement> > blobs;

    friend class SDFORMAT_VISIBLE URDF2SDF;
  };
}
#endif
