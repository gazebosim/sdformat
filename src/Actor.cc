/*
 * Copyright 2019 Open Source Robotics Foundation
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
#include <string>
#include <vector>
#include <ignition/math/Pose3.hh>
#include "sdf/Actor.hh"
#include "sdf/Error.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Animation private data.
class sdf::Animation::Implementation
{
  /// \brief Unique name for animation.
  public: std::string name = "__default__";

  /// \brief Path to animation file.
  public: std::string filename = "__default__";

  /// \brief The path to the file where this animation was defined.
  public: std::string filePath = "";

  /// \brief Scale for animation skeleton.
  public: double scale = 1.0;

  /// \brief True if the animation is interpolated on X.
  public: bool interpolateX = false;
};

/// \brief Waypoint private data.
class sdf::Waypoint::Implementation
{
  /// \brief Time to indicate when the pose should be reached.
  public: double time = 0.0;

  /// \brief Pose to be reached.
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;
};

/// \brief Trajectory private data.
class sdf::Trajectory::Implementation
{
    /// \brief Unique id for a trajectory.
    public: uint64_t id = 0;

    /// \brief String to indicate the animation type.
    public: std::string type = "__default__";

    /// \brief Tension of the trajectory spline.
    public: double tension = 0.0;

    /// \brief Each points in the trajectory.
    public: std::vector<Waypoint> waypoints;
};

/// \brief Actor private data.
class sdf::Actor::Implementation
{
  /// \brief Name of the actor.
  public: std::string name = "__default__";

  /// \brief Pose of the actor.
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the actor.
  public: std::string poseRelativeTo = "";

  /// \brief Filename of the actor skin.
  public: std::string skinFilename = "__default__";

  /// \brief The path to the file where this actor was defined.
  public: std::string filePath = "";

  /// \brief Scale of the actor skin.
  public: double skinScale = 1.0;

  /// \brief Animations loaded for the actor.
  public: std::vector<Animation> animations;

  /// \brief True if the animation plays in loop.
  public: bool scriptLoop = true;

  /// \brief Time to wait before starting the script.
  public: double scriptDelayStart = 0.0;

  /// \brief True if the animation strat with the simulation.
  public: bool scriptAutoStart = true;

  /// \brief Trajectories for the actor.
  public: std::vector<Trajectory> trajectories;

  /// \brief Links for the actor.
  public: std::vector<Link> links;

  /// \brief Joints for the actor.
  public: std::vector<Joint> joints;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Animation::Animation()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Animation::Load(ElementPtr _sdf)
{
  Errors errors;

  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
          "An <animation> requires a name attribute."});
  }

  this->dataPtr->filePath = _sdf->FilePath();

  std::pair filenameValue = _sdf->Get<std::string>("filename",
        this->dataPtr->filename);

  if (!filenameValue.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
          "An <animation> requires a <filename>."});
  }
  this->dataPtr->filename = filenameValue.first;

  this->dataPtr->scale = _sdf->Get<double>("scale", this->dataPtr->scale).first;

  this->dataPtr->interpolateX = _sdf->Get<bool>("interpolate_x",
        this->dataPtr->interpolateX).first;

  return errors;
}

/////////////////////////////////////////////////
const std::string &Animation::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Animation::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const std::string &Animation::Filename() const
{
  return this->dataPtr->filename;
}

/////////////////////////////////////////////////
void Animation::SetFilename(const std::string &_filename)
{
  this->dataPtr->filename = _filename;
}

/////////////////////////////////////////////////
double Animation::Scale() const
{
  return this->dataPtr->scale;
}

/////////////////////////////////////////////////
void Animation::SetScale(double _scale)
{
  this->dataPtr->scale = _scale;
}

/////////////////////////////////////////////////
bool Animation::InterpolateX() const
{
  return this->dataPtr->interpolateX;
}

/////////////////////////////////////////////////
void Animation::SetInterpolateX(bool _interpolateX)
{
  this->dataPtr->interpolateX = _interpolateX;
}

//////////////////////////////////////////////////
const std::string &Animation::FilePath() const
{
  return this->dataPtr->filePath;
}

//////////////////////////////////////////////////
void Animation::SetFilePath(const std::string &_filePath)
{
  this->dataPtr->filePath = _filePath;
}

/////////////////////////////////////////////////
Waypoint::Waypoint()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Waypoint::Load(ElementPtr _sdf)
{
  Errors errors;

  std::pair timeValue = _sdf->Get<double>("time", this->dataPtr->time);
  if (!timeValue.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A <waypoint> requires a <time>."});
  }
  this->dataPtr->time = timeValue.first;

  std::pair posePair = _sdf->Get<ignition::math::Pose3d>
                        ("pose", this->dataPtr->pose);
  if (!posePair.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A <waypoint> requires a <pose>."});
  }
  this->dataPtr->pose = posePair.first;

  return errors;
}

/////////////////////////////////////////////////
double Waypoint::Time() const
{
  return this->dataPtr->time;
}

/////////////////////////////////////////////////
void Waypoint::SetTime(double _time)
{
  this->dataPtr->time = _time;
}

/////////////////////////////////////////////////
ignition::math::Pose3d Waypoint::Pose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
void Waypoint::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
Trajectory::Trajectory()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Trajectory::Load(ElementPtr _sdf)
{
  Errors errors;

  std::pair idValue = _sdf->Get<uint64_t>("id", this->dataPtr->id);
  if (!idValue.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A <trajectory> requires a <id>."});
  }
  this->dataPtr->id = idValue.first;

  std::pair typePair = _sdf->Get<std::string>("type", this->dataPtr->type);
  if (!typePair.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A <trajectory> requires a <type>."});
  }
  this->dataPtr->type = typePair.first;

  this->dataPtr->tension = _sdf->Get<double>
          ("tension", this->dataPtr->tension).first;

  Errors waypointLoadErrors = loadRepeated<Waypoint>(_sdf, "waypoint",
    this->dataPtr->waypoints);

  errors.insert(errors.end(), waypointLoadErrors.begin(),
                    waypointLoadErrors.end());

  return errors;
}

/////////////////////////////////////////////////
uint64_t Trajectory::Id() const
{
  return this->dataPtr->id;
}

/////////////////////////////////////////////////
void Trajectory::SetId(uint64_t _id)
{
  this->dataPtr->id = _id;
}

/////////////////////////////////////////////////
const std::string &Trajectory::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void Trajectory::SetType(const std::string &_type)
{
  this->dataPtr->type = _type;
}

/////////////////////////////////////////////////
double Trajectory::Tension() const
{
  return this->dataPtr->tension;
}

/////////////////////////////////////////////////
void Trajectory::SetTension(double _tension)
{
  this->dataPtr->tension = _tension;
}

/////////////////////////////////////////////////
uint64_t Trajectory::WaypointCount() const
{
  return this->dataPtr->waypoints.size();
}

/////////////////////////////////////////////////
const Waypoint *Trajectory::WaypointByIndex(uint64_t _index) const
{
  if (_index < this->dataPtr->waypoints.size())
    return &this->dataPtr->waypoints[_index];
  return nullptr;
}

/////////////////////////////////////////////////
void Trajectory::AddWaypoint(const Waypoint &_waypoint)
{
  this->dataPtr->waypoints.push_back(_waypoint);
}

/////////////////////////////////////////////////
Actor::Actor()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Actor::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;
  this->dataPtr->filePath = _sdf->FilePath();

  if (_sdf->GetName() != "actor")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load an actor, but the provided SDF element is not an"
        "<actor>."});
    return errors;
  }

  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                      "An actor name is required, but the name is not set."});
  }

  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

  sdf::ElementPtr skinElem = _sdf->GetElement("skin");

  if (skinElem)
  {
    std::pair<std::string, bool> filenamePair = skinElem->Get<std::string>
                ("filename", this->dataPtr->skinFilename);

    if (!filenamePair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
            "A <skin> requires a <filename>."});
    }

    this->dataPtr->skinFilename = filenamePair.first;

    this->dataPtr->skinScale = skinElem->Get<double>("scale",
        this->dataPtr->skinScale).first;
  }

  Errors animationLoadErrors = loadUniqueRepeated<Animation>(_sdf, "animation",
     this->dataPtr->animations);

  errors.insert(errors.end(), animationLoadErrors.begin(),
                    animationLoadErrors.end());

  sdf::ElementPtr scriptElem = _sdf->GetElement("script");

  if (!scriptElem)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
          "An <actor> requires a <script>."});
  }
  else
  {
    this->dataPtr->scriptLoop = scriptElem->Get<bool>("loop",
        this->dataPtr->scriptLoop).first;

    this->dataPtr->scriptDelayStart = scriptElem->Get<double>("delay_start",
        this->dataPtr->scriptDelayStart).first;

    this->dataPtr->scriptAutoStart = scriptElem->Get<bool>("auto_start",
        this->dataPtr->scriptAutoStart).first;

    Errors trajectoryLoadErrors = loadRepeated<Trajectory>(scriptElem,
        "trajectory", this->dataPtr->trajectories);

    errors.insert(errors.end(), trajectoryLoadErrors.begin(),
                      trajectoryLoadErrors.end());
  }

  Errors linkLoadErrors = loadRepeated<Link>(_sdf, "link",
    this->dataPtr->links);

  errors.insert(errors.end(), linkLoadErrors.begin(),
                    linkLoadErrors.end());

  Errors jointLoadErrors = loadRepeated<Joint>(_sdf, "joint",
    this->dataPtr->joints);

  errors.insert(errors.end(), jointLoadErrors.begin(),
                    jointLoadErrors.end());

  return errors;
}

/////////////////////////////////////////////////
const std::string &Actor::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Actor::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Actor::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Actor::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Actor::SetRawPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Actor::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
sdf::ElementPtr Actor::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
const std::string &Actor::SkinFilename() const
{
  return this->dataPtr->skinFilename;
}

/////////////////////////////////////////////////
void Actor::SetSkinFilename(std::string _skinFilename)
{
  this->dataPtr->skinFilename = _skinFilename;
}

/////////////////////////////////////////////////
double Actor::SkinScale() const
{
  return this->dataPtr->skinScale;
}

/////////////////////////////////////////////////
void Actor::SetSkinScale(double _skinScale)
{
  this->dataPtr->skinScale = _skinScale;
}

/////////////////////////////////////////////////
uint64_t Actor::AnimationCount() const
{
  return this->dataPtr->animations.size();
}

/////////////////////////////////////////////////
const Animation *Actor::AnimationByIndex(uint64_t _index) const
{
  if (_index < this->dataPtr->animations.size())
    return &this->dataPtr->animations[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Actor::AnimationNameExists(const std::string &_name) const
{
  for (const auto &a : this->dataPtr->animations)
  {
    if (a.Name() == _name)
      return true;
  }
  return false;
}

/////////////////////////////////////////////////
void Actor::AddAnimation(const Animation &_anim)
{
  this->dataPtr->animations.push_back(_anim);
}

/////////////////////////////////////////////////
bool Actor::ScriptLoop() const
{
  return this->dataPtr->scriptLoop;
}

/////////////////////////////////////////////////
void Actor::SetScriptLoop(bool _scriptLoop)
{
  this->dataPtr->scriptLoop = _scriptLoop;
}

/////////////////////////////////////////////////
double Actor::ScriptDelayStart() const
{
  return this->dataPtr->scriptDelayStart;
}

/////////////////////////////////////////////////
void Actor::SetScriptDelayStart(double _scriptDelayStart)
{
  this->dataPtr->scriptDelayStart = _scriptDelayStart;
}

/////////////////////////////////////////////////
bool Actor::ScriptAutoStart() const
{
  return this->dataPtr->scriptAutoStart;
}

/////////////////////////////////////////////////
void Actor::SetScriptAutoStart(bool _scriptAutoStart)
{
  this->dataPtr->scriptAutoStart = _scriptAutoStart;
}

/////////////////////////////////////////////////
uint64_t Actor::TrajectoryCount() const
{
  return this->dataPtr->trajectories.size();
}

/////////////////////////////////////////////////
const Trajectory *Actor::TrajectoryByIndex(uint64_t _index) const
{
  if (_index < this->dataPtr->trajectories.size())
    return &this->dataPtr->trajectories[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Actor::TrajectoryIdExists(uint64_t _id) const
{
  for (const auto &t : this->dataPtr->trajectories)
  {
    if (t.Id() == _id)
      return true;
  }
  return false;
}

/////////////////////////////////////////////////
void Actor::AddTrajectory(const Trajectory &_traj)
{
  this->dataPtr->trajectories.push_back(_traj);
}

/////////////////////////////////////////////////
uint64_t Actor::LinkCount() const
{
  return this->dataPtr->links.size();
}

/////////////////////////////////////////////////
const Link *Actor::LinkByIndex(uint64_t _index) const
{
  if (_index < this->dataPtr->links.size())
    return &this->dataPtr->links[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Actor::LinkNameExists(const std::string &_name) const
{
  for (const auto &l : this->dataPtr->links)
  {
    if (l.Name() == _name)
      return true;
  }
  return false;
}

/////////////////////////////////////////////////
uint64_t Actor::JointCount() const
{
  return this->dataPtr->joints.size();
}

/////////////////////////////////////////////////
const Joint *Actor::JointByIndex(uint64_t _index) const
{
  if (_index < this->dataPtr->joints.size())
    return &this->dataPtr->joints[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Actor::JointNameExists(const std::string &_name) const
{
  for (const auto &j : this->dataPtr->joints)
  {
    if (j.Name() == _name)
      return true;
  }
  return false;
}

//////////////////////////////////////////////////
const std::string &Actor::FilePath() const
{
  return this->dataPtr->filePath;
}

//////////////////////////////////////////////////
void Actor::SetFilePath(const std::string &_filePath)
{
  this->dataPtr->filePath = _filePath;
}
