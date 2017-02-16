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

#include "sdf/SDFExtension.hh"

using namespace sdf;

/////////////////////////////////////////////////
SDFExtension::SDFExtension()
{
  this->material.clear();
  this->visual_blobs.clear();
  this->collision_blobs.clear();
  this->setStaticFlag = false;
  this->gravity = true;
  this->isDampingFactor = false;
  this->isMaxContacts = false;
  this->isMaxVel = false;
  this->isMinDepth = false;
  this->fdir1.clear();
  this->isMu1 = false;
  this->isMu2 = false;
  this->isKp = false;
  this->isKd = false;
  this->selfCollide = false;
  this->isLaserRetro = false;
  this->isSpringReference = false;
  this->isSpringStiffness = false;
  this->isStopCfm = false;
  this->isStopErp = false;
  this->isStopKp = false;
  this->isStopKd = false;
  this->isInitialJointPosition = false;
  this->isFudgeFactor = false;
  this->isProvideFeedback = false;
  this->isImplicitSpringDamper = false;
  this->blobs.clear();

  this->dampingFactor = 0;
  this->maxContacts = 0;
  this->maxVel = 0;
  this->minDepth = 0;
  this->mu1 = 0;
  this->mu2 = 0;
  this->kp = 100000000;
  this->kd = 1;
  this->laserRetro = 101;
  this->springReference = 0;
  this->springStiffness = 0;
  this->stopCfm = 0;
  this->stopErp = 0.1;
  this->stopKp = 100000000;
  this->stopKd = 1;
  this->initialJointPosition = 0;
  this->fudgeFactor = 1;

  this->provideFeedback = false;
  this->implicitSpringDamper = false;
}

/////////////////////////////////////////////////
SDFExtension::SDFExtension(const SDFExtension &_ge)
{
  this->material = _ge.material;
  this->visual_blobs = _ge.visual_blobs;
  this->collision_blobs = _ge.collision_blobs;
  this->setStaticFlag = _ge.setStaticFlag;
  this->gravity = _ge.gravity;
  this->isDampingFactor = _ge.isDampingFactor;
  this->isMaxContacts = _ge.isMaxContacts;
  this->isMaxVel = _ge.isMaxVel;
  this->isMinDepth = _ge.isMinDepth;
  this->fdir1 = _ge.fdir1;
  this->isMu1 = _ge.isMu1;
  this->isMu2 = _ge.isMu2;
  this->isKp = _ge.isKp;
  this->isKd = _ge.isKd;
  this->selfCollide = _ge.selfCollide;
  this->isLaserRetro = _ge.isLaserRetro;
  this->isSpringReference = _ge.isSpringReference;
  this->isSpringStiffness = _ge.isSpringStiffness;
  this->isStopKp = _ge.isStopKp;
  this->isStopKd = _ge.isStopKd;
  this->isStopCfm = _ge.isStopCfm;
  this->isStopErp = _ge.isStopErp;
  this->isInitialJointPosition = _ge.isInitialJointPosition;
  this->isFudgeFactor = _ge.isFudgeFactor;
  this->isProvideFeedback = _ge.isProvideFeedback;
  this->isImplicitSpringDamper = _ge.isImplicitSpringDamper;
  this->provideFeedback = _ge.provideFeedback;
  this->implicitSpringDamper = _ge.implicitSpringDamper;
  this->oldLinkName = _ge.oldLinkName;
  this->reductionTransform = _ge.reductionTransform;
  this->blobs = _ge.blobs;

  this->dampingFactor = _ge.dampingFactor;
  this->maxContacts = _ge.maxContacts;
  this->maxVel = _ge.maxVel;
  this->minDepth = _ge.minDepth;
  this->mu1 = _ge.mu1;
  this->mu2 = _ge.mu2;
  this->kp = _ge.kp;
  this->kd = _ge.kd;
  this->laserRetro = _ge.laserRetro;
  this->springReference = _ge.springReference;
  this->springStiffness = _ge.springStiffness;
  this->stopKp = _ge.stopKp;
  this->stopKd = _ge.stopKd;
  this->stopCfm = _ge.stopCfm;
  this->stopErp = _ge.stopErp;
  this->initialJointPosition = _ge.initialJointPosition;
  this->fudgeFactor = _ge.fudgeFactor;
}
