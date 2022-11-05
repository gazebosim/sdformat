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

#include "SDFExtension.hh"

using namespace sdf;

/////////////////////////////////////////////////
SDFExtension::SDFExtension()
{
  this->material.clear();
  this->visual_blobs.clear();
  this->collision_blobs.clear();
  this->isSetStaticFlag = false;
  this->setStaticFlag = false;
  this->isGravity = false;
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
  this->isSelfCollide = false;
  this->selfCollide = false;
  this->isLaserRetro = false;
  this->isSpringReference = false;
  this->isSpringStiffness = false;
  this->isStopCfm = false;
  this->isStopErp = false;
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
  this->fudgeFactor = 1;

  this->provideFeedback = false;
  this->implicitSpringDamper = false;
}
