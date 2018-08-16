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
#ifndef SDF_MODEL_HH_
#define SDF_MODEL_HH_

#include <string>
#include <ignition/math/Pose3.hh>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declarations.
  class Joint;
  class Link;
  class ModelPrivate;

  class SDFORMAT_VISIBLE Model
  {
    /// \brief Default constructor
    public: Model();

    /// \brief Move constructor
    /// \param[in] _model Model to move.
    public: Model(Model &&_model);

    /// \brief Destructor
    public: ~Model();

    /// \brief Load the model based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the model.
    /// The name of the model should be unique within the scope of a World.
    /// \return Name of the model.
    public: std::string Name() const;

    /// \brief Set the name of the model.
    /// The name of the model should be unique within the scope of a World.
    /// \param[in] _name Name of the model.
    public: void SetName(const std::string &_name);

    /// \brief Check if this model should be static.
    /// A static model is one that is not subject to physical forces (in other
    /// words, it's purely kinematic instead of dynamic).
    /// \return true if the model should be static, false if it is dynamic.
    public: bool Static() const;

    /// \brief Set this model to be static or not static.
    /// \param[in] _static True or false depending on whether the model should
    /// be static.
    /// \sa Static()
    public: void SetStatic(bool _static);

    /// \brief Check if this model should self-collide.
    /// A self-colliding model is a model whose links will collide if they come
    /// into contact. If a model is not self-colliding, its links will pass
    /// through each other.
    /// \return true if the model should self-collide, false otherwise.
    public: bool SelfCollide() const;

    /// \brief Set this model to self-collide or not self-collide.
    /// \param[in] _selfCollide True or false depending on whether the model
    /// should self-collide.
    /// \sa SelfCollide()
    public: void SetSelfCollide(bool _selfCollide);

    /// \brief Check if this model should be allowed to auto-disable.
    /// If auto-disable is allowed, a model that is at rest can choose to not
    /// update its dynamics.
    /// \return true if auto-disable is allowed for this model
    public: bool AllowAutoDisable() const;

    /// \brief Set this model to allow auto-disabling.
    /// \param[in] _allowAutoDisable True or false depending on whether the
    /// model should be allowed to auto-disable.
    /// \sa AllowAutoDisable()
    public: void SetAllowAutoDisable(bool _allowAutoDisable);

    /// \brief Check if this model should be subject to wind.
    /// If true, all links in the model should be affected by the wind. This can
    /// be overridden per link.
    /// \return true if the model should be subject to wind, false otherwise.
    public: bool EnableWind() const;

    /// \brief Set whether this model should be subject to wind.
    /// \param[in] _enableWind True or false depending on whether the model
    /// should be subject to wind.
    public: void SetEnableWind(bool _enableWind);

    /// \brief Get the number of links.
    /// \return Number of links contained in this Model object.
    public: uint64_t LinkCount() const;

    /// \brief Get a link based on an index.
    /// \param[in] _index Index of the link. The index should be in the
    /// range [0..LinkCount()).
    /// \return Pointer to the link. Nullptr if the index does not exist.
    /// \sa uint64_t LinkCount() const
    public: const Link *LinkByIndex(const uint64_t _index) const;

    /// \brief Get a link based on a name.
    /// \param[in] _name Name of the link.
    /// \return Pointer to the link. Nullptr if the name does not exist.
    public: const Link *LinkByName(const std::string &_name) const;

    /// \brief Get whether a link name exists.
    /// \param[in] _name Name of the link to check.
    /// \return True if there exists a link with the given name.
    public: bool LinkNameExists(const std::string &_name) const;

    /// \brief Get the number of joints.
    /// \return Number of joints contained in this Model object.
    public: uint64_t JointCount() const;

    /// \brief Get a joint based on an index.
    /// \param[in] _index Index of the joint. The index should be in the
    /// range [0..JointCount()).
    /// \return Pointer to the joint. Nullptr if the index does not exist.
    /// \sa uint64_t JointCount() const
    public: const Joint *JointByIndex(const uint64_t _index) const;

    /// \brief Get whether a joint name exists.
    /// \param[in] _name Name of the joint to check.
    /// \return True if there exists a joint with the given name.
    public: bool JointNameExists(const std::string &_name) const;

    /// \brief Get a joint based on a name.
    /// \param[in] _name Name of the joint.
    /// \return Pointer to the joint. Nullptr if a joint with the given name
    ///  does not exist.
    /// \sa bool JointNameExists(const std::string &_name) const
    public: const Joint *JointByName(const std::string &_name) const;

    /// \brief Get the pose of the model. This is the pose of the model
    /// as specified in SDF (<model> <pose> ... </pose></model>), and is
    /// typically used to express the position and rotation of a model in a
    /// global coordinate frame.
    /// \return The pose of the model.
    public: const ignition::math::Pose3d &Pose() const;

    /// \brief Set the pose of the model.
    /// \sa const ignition::math::Pose3d &Pose() const
    /// \param[in] _pose The new model pose.
    public: void SetPose(const ignition::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame in which this model's
    /// pose is expressed. A empty value indicates that the frame is the
    /// global/world coordinate frame.
    /// \return The name of the pose frame.
    public: const std::string &PoseFrame() const;

    /// \brief Set the name of the coordinate frame in which this model's
    /// pose is expressed. A empty value indicates that the frame is the
    /// global/world coordinate frame.
    /// \param[in] _frame The name of the pose frame.
    public: void SetPoseFrame(const std::string &_frame);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: ModelPrivate *dataPtr = nullptr;
  };
}
#endif
