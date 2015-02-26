/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef URDF2SDF_HH
#define URDF2SDF_HH

#include <tinyxml.h>
#include <vector>
#include <string>
#include <map>

#include "sdf/Types.hh"
#include "sdf/Console.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  /// \addtogroup sdf
  /// \{

  /// \brief A class for holding sdf extension elements in urdf
  class SDFORMAT_VISIBLE SDFExtension
  {
    private: SDFExtension()
    {
      material.clear();
      visual_blobs.clear();
      setStaticFlag = false;
      gravity = true;
      isDampingFactor = false;
      isMaxContacts = false;
      isMaxVel = false;
      isMinDepth = false;
      fdir1.clear();
      isMu1 = false;
      isMu2 = false;
      isKp = false;
      isKd = false;
      selfCollide = false;
      isLaserRetro = false;
      isStopCfm = false;
      isStopErp = false;
      isStopKp = false;
      isStopKd = false;
      isInitialJointPosition = false;
      isFudgeFactor = false;
      isProvideFeedback = false;
      isImplicitSpringDamper = false;
      blobs.clear();

      dampingFactor = 0;
      maxContacts = 0;
      maxVel = 0;
      minDepth = 0;
      mu1 = 0;
      mu2 = 0;
      kp = 100000000;
      kd = 1;
      laserRetro = 101;
      stopCfm = 0;
      stopErp = 0.1;
      stopKp = 100000000;
      stopKd = 1;
      initialJointPosition = 0;
      fudgeFactor = 1;

      provideFeedback = false;
      implicitSpringDamper = false;
    };

    private: SDFExtension(const SDFExtension &ge)
    {
      material = ge.material;
      visual_blobs = ge.visual_blobs;
      setStaticFlag = ge.setStaticFlag;
      gravity = ge.gravity;
      isDampingFactor = ge.isDampingFactor;
      isMaxContacts = ge.isMaxContacts;
      isMaxVel = ge.isMaxVel;
      isMinDepth = ge.isMinDepth;
      fdir1 = ge.fdir1;
      isMu1 = ge.isMu1;
      isMu2 = ge.isMu2;
      isKp = ge.isKp;
      isKd = ge.isKd;
      selfCollide = ge.selfCollide;
      isLaserRetro = ge.isLaserRetro;
      isStopKp = ge.isStopKp;
      isStopKd = ge.isStopKd;
      isStopCfm = ge.isStopCfm;
      isStopErp = ge.isStopErp;
      isInitialJointPosition = ge.isInitialJointPosition;
      isFudgeFactor = ge.isFudgeFactor;
      isProvideFeedback = ge.isProvideFeedback;
      isImplicitSpringDamper = ge.isImplicitSpringDamper;
      provideFeedback = ge.provideFeedback;
      implicitSpringDamper = ge.implicitSpringDamper;
      oldLinkName = ge.oldLinkName;
      reductionTransform = ge.reductionTransform;
      blobs = ge.blobs;

      dampingFactor = ge.dampingFactor;
      maxContacts = ge.maxContacts;
      maxVel = ge.maxVel;
      minDepth = ge.minDepth;
      mu1 = ge.mu1;
      mu2 = ge.mu2;
      kp = ge.kp;
      kd = ge.kd;
      laserRetro = ge.laserRetro;
      stopKp = ge.stopKp;
      stopKd = ge.stopKd;
      stopCfm = ge.stopCfm;
      stopErp = ge.stopErp;
      initialJointPosition = ge.initialJointPosition;
      fudgeFactor = ge.fudgeFactor;
    };

    // for reducing fixed joints and removing links
    public: std::string oldLinkName;
    public: sdf::Pose reductionTransform;

    // visual
    public: std::string material;
    public: std::vector<boost::shared_ptr<TiXmlElement> > visual_blobs;

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
    public: bool isProvideFeedback;
    public: bool provideFeedback;
    public: bool isImplicitSpringDamper;
    public: bool implicitSpringDamper;
    private: bool isStopKp, isStopKd;
    private: double stopKp, stopKd;

    // blobs into body or robot
    public: std::vector<boost::shared_ptr<TiXmlElement> > blobs;

    friend class SDFORMAT_VISIBLE URDF2SDF;
  };

  /// \brief URDF to SDF converter
  class SDFORMAT_VISIBLE URDF2SDF
  {
    /// \brief constructor
    public: URDF2SDF();

    /// \brief destructor
    public: ~URDF2SDF();

    /// \brief convert urdf xml document string to sdf xml document
    /// \param[in] _xmlDoc a tinyxml document containing the urdf model
    /// \return a tinyxml document containing sdf of the model
    public: TiXmlDocument InitModelDoc(TiXmlDocument* _xmlDoc);

    /// \brief convert urdf file to sdf xml document
    /// \param[in] _urdfStr a string containing filename of the urdf model
    /// \return a tinyxml document containing sdf of the model
    public: TiXmlDocument InitModelFile(const std::string &_filename);

    /// \brief convert urdf string to sdf xml document, with option to enforce
    /// limits.
    /// \param[in] _urdfStr a string containing model urdf
    /// \param[in] _enforceLimits option to enforce joint limits
    /// \return a tinyxml document containing sdf of the model
    public: TiXmlDocument InitModelString(const std::string &_urdfStr,
                                          bool _enforceLimits = true);

    /// things that do not belong in urdf but should be mapped into sdf
    /// @todo: do this using sdf definitions, not hard coded stuff
    private: void ParseSDFExtension(TiXmlDocument &_urdfXml);

    /// list extensions for debugging
    private: void ListSDFExtensions();

    /// list extensions for debugging
    private: void ListSDFExtensions(const std::string &_reference);
  };
  /// \}
}

#endif
