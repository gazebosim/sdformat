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
#include "sdf/Error.hh"
#include "sdf/Actor.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Animation private data.
class sdf::AnimationPrivate
{
  /// \brief Unique name for animation.
  public: std::string name = "__default__";

  /// \brief Path to animation file.
  public: std::string filename = "__default__";

  /// \brief Scale for animation skeleton.
  public: double scale = 1.0;

  /// \brief True if the animation is interpolated on X.
  public: bool interpolate_x = false;
};

/// \brief Waypoint private data.
class sdf::WaypointPrivate
{
  /// \brief Time to indicate when the pose should be reached.
  public: double time = 0.0;

  /// \brief Pose to be reached.
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;
};

/// \brief Trajectory private data.
class sdf::TrajectoryPrivate
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
class sdf::ActorPrivate
{
  /// \brief Name of the actor.
  public: std::string name = "__default__";

  /// \brief Pose of the actor.
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the actor.
  public: std::string poseFrame = "";

  /// \brief True if the actor is static.
  public: bool actor_static = true;

  /// \brief Filename of the actor skin.
  public: std::string skin_filename = "__default__";

  /// \brief Scale of the actor skin.
  public: double skin_scale = 1.0;

  /// \brief Animations loaded for the actor.
  public: std::vector<Animation> animations;

  /// \brief True if the animation plays in loop.
  public: bool script_loop = true;

  /// \brief Time to wait before starting the script.
  public: double script_delay_start = 0.0;

  /// \brief True if the animation strat with the simulation.
  public: bool script_auto_start = true;

  /// \brief Trajectories for the actor.
  public: std::vector<Trajectory> trajectories;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Animation::Animation()
  : dataPtr(new AnimationPrivate)
{
}

void Animation::CopyFrom(const Animation &_animation)
{
  this->dataPtr->name = _animation.dataPtr->name;
  this->dataPtr->filename = _animation.dataPtr->filename;
  this->dataPtr->scale = _animation.dataPtr->scale;
  this->dataPtr->interpolate_x = _animation.dataPtr->interpolate_x;
}

/////////////////////////////////////////////////
Animation::Animation(Animation &&_animation) noexcept
{
  this->dataPtr = _animation.dataPtr;
  _animation.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Animation::~Animation()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Animation::Animation(const Animation &_animation)
  : dataPtr(new AnimationPrivate)
{
  this->CopyFrom(_animation);
}

//////////////////////////////////////////////////
Animation &Animation::operator=(const Animation &_animation)
{
  if (!this->dataPtr)
  {
    this->dataPtr = new AnimationPrivate;
  }
  this->CopyFrom(_animation);
  return *this;
}

//////////////////////////////////////////////////
Animation &Animation::operator=(Animation &&_animation)
{
  this->dataPtr = _animation.dataPtr;
  _animation.dataPtr = nullptr;
  return *this;
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

  std::pair filenameValue = _sdf->Get<std::string>("filename",
        this->dataPtr->filename);

  if (!filenameValue.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
          "An <animation> requires a <filename>."});
  }
  this->dataPtr->filename = filenameValue.first;

  this->dataPtr->scale = _sdf->Get<double>("scale", this->dataPtr->scale).first;

  this->dataPtr->interpolate_x = _sdf->Get<bool>("interpolate_x",
        this->dataPtr->interpolate_x).first;

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
void Animation::SetScale(const double _scale)
{
  this->dataPtr->scale = _scale;
}

/////////////////////////////////////////////////
bool Animation::InterpolateX() const
{
  return this->dataPtr->interpolate_x;
}

/////////////////////////////////////////////////
void Animation::SetInterpolateX(const bool _interpolateX)
{
  this->dataPtr->interpolate_x = _interpolateX;
}

/////////////////////////////////////////////////
Waypoint::Waypoint()
  : dataPtr(new WaypointPrivate)
{
}

void Waypoint::CopyFrom(const Waypoint &_waypoint)
{
  this->dataPtr->time = _waypoint.dataPtr->time;
  this->dataPtr->pose = _waypoint.dataPtr->pose;
}

/////////////////////////////////////////////////
Waypoint::Waypoint(Waypoint &&_waypoint) noexcept
{
  this->dataPtr = _waypoint.dataPtr;
  _waypoint.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Waypoint::~Waypoint()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Waypoint::Waypoint(const Waypoint &_waypoint)
  : dataPtr(new WaypointPrivate)
{
  this->CopyFrom(_waypoint);
}

//////////////////////////////////////////////////
Waypoint &Waypoint::operator=(const Waypoint &_waypoint)
{
  this->CopyFrom(_waypoint);
  return *this;
}

//////////////////////////////////////////////////
Waypoint &Waypoint::operator=(Waypoint &&_waypoint)
{
  this->dataPtr = _waypoint.dataPtr;
  _waypoint.dataPtr = nullptr;
  return *this;
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
void Waypoint::SetTime(const double _time)
{
  this->dataPtr->time = _time;
}

/////////////////////////////////////////////////
ignition::math::Pose3d Waypoint::Pose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
void Waypoint::SetPose(ignition::math::Pose3d _pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
Trajectory::Trajectory()
  : dataPtr(new TrajectoryPrivate)
{
}

void Trajectory::CopyFrom(const Trajectory &_trajectory)
{
  this->dataPtr->id = _trajectory.dataPtr->id;
  this->dataPtr->type = _trajectory.dataPtr->type;
  this->dataPtr->tension = _trajectory.dataPtr->tension;
  this->dataPtr->waypoints = _trajectory.dataPtr->waypoints;
}

/////////////////////////////////////////////////
Trajectory::Trajectory(Trajectory &&_trajectory) noexcept
{
  this->dataPtr = _trajectory.dataPtr;
  _trajectory.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Trajectory::~Trajectory()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Trajectory::Trajectory(const Trajectory &_trajectory)
  : dataPtr(new TrajectoryPrivate)
{
  this->CopyFrom(_trajectory);
}

//////////////////////////////////////////////////
Trajectory &Trajectory::operator=(const Trajectory &_trajectory)
{
  this->CopyFrom(_trajectory);
  return *this;
}

//////////////////////////////////////////////////
Trajectory &Trajectory::operator=(Trajectory &&_trajectory)
{
  this->dataPtr = _trajectory.dataPtr;
  _trajectory.dataPtr = nullptr;
  return *this;
}

/////////////////////////////////////////////////
uint64_t Trajectory::Id() const
{
  return this->dataPtr->id;
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

  return waypointLoadErrors;
}

/////////////////////////////////////////////////
const std::string &Trajectory::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void Trajectory::SetType(std::string &_type)
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
const Waypoint *Trajectory::WaypointByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->waypoints.size())
    return &this->dataPtr->waypoints[_index];
  return nullptr;
}


/////////////////////////////////////////////////
Actor::Actor()
  : dataPtr(new ActorPrivate)
{
}

/////////////////////////////////////////////////
Actor::Actor(Actor &&_actor) noexcept
{
  this->dataPtr = _actor.dataPtr;
  _actor.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Actor::~Actor()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Actor::Actor(const Actor &_actor)
  : dataPtr(new ActorPrivate)
{
  this->CopyFrom(_actor);
}

//////////////////////////////////////////////////
Actor &Actor::operator=(const Actor &_actor)
{
  if (!this->dataPtr)
  {
    this->dataPtr = new ActorPrivate;
  }
  this->CopyFrom(_actor);
  return *this;
}

//////////////////////////////////////////////////
Actor &Actor::operator=(Actor &&_actor)
{
  this->dataPtr = _actor.dataPtr;
  _actor.dataPtr = nullptr;
  return *this;
}

//////////////////////////////////////////////////
void Actor::CopyFrom(const Actor &_actor)
{
  this->dataPtr->name = _actor.dataPtr->name;
  this->dataPtr->pose = _actor.dataPtr->pose;
  this->dataPtr->poseFrame = _actor.dataPtr->poseFrame;
  this->dataPtr->actor_static = _actor.dataPtr->actor_static;
  this->dataPtr->skin_filename = _actor.dataPtr->skin_filename;
  this->dataPtr->skin_scale = _actor.dataPtr->skin_scale;
  this->dataPtr->animations = _actor.dataPtr->animations;
  this->dataPtr->script_loop = _actor.dataPtr->script_loop;
  this->dataPtr->script_delay_start = _actor.dataPtr->script_delay_start;
  this->dataPtr->script_auto_start = _actor.dataPtr->script_auto_start;
  this->dataPtr->trajectories = _actor.dataPtr->trajectories;
}

/////////////////////////////////////////////////
Errors Actor::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

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

  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseFrame);

  this->dataPtr->actor_static = _sdf->Get<bool>("static",
      this->dataPtr->actor_static).first;

  sdf::ElementPtr skinElem = _sdf->GetElement("skin");

  if (skinElem)
  {
    std::pair<std::string, bool> filenamePair = skinElem->Get<std::string>
                ("filename", this->dataPtr->skin_filename);

    if (!filenamePair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
            "A <skin> requires a <filename>."});
    }

    this->dataPtr->skin_filename = filenamePair.first;

    this->dataPtr->skin_scale = skinElem->Get<double>("scale",
        this->dataPtr->skin_scale).first;
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
    this->dataPtr->script_loop = scriptElem->Get<bool>("loop",
        this->dataPtr->script_loop).first;

    this->dataPtr->script_delay_start = scriptElem->Get<double>("delay_start",
        this->dataPtr->script_delay_start).first;

    this->dataPtr->script_auto_start = scriptElem->Get<bool>("auto_start",
        this->dataPtr->script_auto_start).first;

    Errors trajectoryLoadErrors = loadUniqueRepeated<Trajectory>(scriptElem,
        "trajectory", this->dataPtr->trajectories);

    errors.insert(errors.end(), trajectoryLoadErrors.begin(),
                      trajectoryLoadErrors.end());
  }

  return errors;
}

/////////////////////////////////////////////////
std::string &Actor::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Actor::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
bool Actor::ActorStatic() const
{
  return this->dataPtr->actor_static;
}

/////////////////////////////////////////////////
void Actor::SetActorStatic(const bool _actorStatic)
{
  this->dataPtr->actor_static = _actorStatic;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Actor::Pose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Actor::PoseFrame() const
{
  return this->dataPtr->poseFrame;
}

/////////////////////////////////////////////////
void Actor::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Actor::SetPoseFrame(const std::string &_frame)
{
  this->dataPtr->poseFrame = _frame;
}

/////////////////////////////////////////////////
sdf::ElementPtr Actor::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
const std::string &Actor::SkinFilename() const
{
  return this->dataPtr->skin_filename;
}

/////////////////////////////////////////////////
void Actor::SetSkinFilename(std::string _skin_filename)
{
  this->dataPtr->skin_filename = _skin_filename;
}

/////////////////////////////////////////////////
double Actor::SkinScale() const
{
  return this->dataPtr->skin_scale;
}

/////////////////////////////////////////////////
void Actor::SetSkinScale(double _skin_scale)
{
  this->dataPtr->skin_scale = _skin_scale;
}

/////////////////////////////////////////////////
uint64_t Actor::AnimationCount() const
{
  return this->dataPtr->animations.size();
}

/////////////////////////////////////////////////
const Animation *Actor::AnimationByIndex(const uint64_t _index) const
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
bool Actor::ScriptLoop() const
{
  return this->dataPtr->script_loop;
}

/////////////////////////////////////////////////
void Actor::SetScriptLoop(bool _scriptLoop)
{
  this->dataPtr->script_loop = _scriptLoop;
}

/////////////////////////////////////////////////
double Actor::ScriptDelayStart() const
{
  return this->dataPtr->script_delay_start;
}

/////////////////////////////////////////////////
void Actor::SetScriptDelayStart(double _script_delay_start)
{
  this->dataPtr->script_delay_start = _script_delay_start;
}

/////////////////////////////////////////////////
bool Actor::ScriptAutoStart() const
{
  return this->dataPtr->script_auto_start;
}

/////////////////////////////////////////////////
void Actor::SetScriptAutoStart(bool _script_auto_start)
{
  this->dataPtr->script_auto_start = _script_auto_start;
}

/////////////////////////////////////////////////
uint64_t Actor::TrajectoryCount() const
{
  return this->dataPtr->trajectories.size();
}

/////////////////////////////////////////////////
const Trajectory *Actor::TrajectoryByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->trajectories.size())
    return &this->dataPtr->trajectories[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Actor::TrajectoryIdExists(const uint64_t _id) const
{
  for (const auto &t : this->dataPtr->trajectories)
  {
    if (t.Id() == _id)
      return true;
  }
  return false;
}
