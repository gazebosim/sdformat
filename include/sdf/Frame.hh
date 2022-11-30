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
#ifndef SDF_FRAME_HH_
#define SDF_FRAME_HH_

#include <memory>
#include <string>
#include <gz/math/Pose3.hh>
#include "sdf/Element.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  // Forward declaration.
  class FramePrivate;
  struct FrameAttachedToGraph;
  struct PoseRelativeToGraph;

  /// \brief A Frame element descibes the properties associated with an
  /// explicit frame defined in a Model or World.
  class SDFORMAT_VISIBLE Frame
  {
    /// \brief Default constructor
    public: Frame();

    /// \brief Copy constructor
    /// \param[in] _frame Frame to copy.
    public: Frame(const Frame &_frame);

    /// \brief Move constructor
    /// \param[in] _frame Frame to move.
    public: Frame(Frame &&_frame);

    /// \brief Destructor
    public: ~Frame();

    /// \brief Move assignment operator.
    /// \param[in] _frame Frame to move.
    /// \return Reference to this.
    public: Frame &operator=(Frame &&_frame);

    /// \brief Assignment operator.
    /// \param[in] _frame The frame to set values from.
    /// \return *this
    public: Frame &operator=(const Frame &_frame);

    /// \brief Load the frame based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the frame.
    /// The name of the frame must be unique within the scope of its siblings.
    /// \return Name of the frame.
    public: const std::string &Name() const;

    /// \brief Set the name of the frame.
    /// The name of the frame must be unique within the scope of its siblings.
    /// \param[in] _name Name of the frame.
    public: void SetName(const std::string &_name) const;

    /// \brief Get the name of the coordinate frame to which this
    /// frame is attached. The interpretation of an empty value depends
    /// on whether the frame is contained in a model or world element. For a
    /// frame that is a child of <model>, an empty value indicates that the
    /// frame is attached to the canonical link of the model. For a frame that
    /// is a child of <world> an empty value indicates that the frame is
    /// attached to the world frame.
    /// \return The name of the attached-to frame.
    public: const std::string &AttachedTo() const;

    /// \brief Set the name of the coordinate frame to which this
    /// frame is attached. The interpretation of an empty value depends
    /// on whether the frame is contained in a model or world element. For a
    /// frame that is a child of <model>, an empty value indicates that the
    /// frame is attached to the canonical link of the model. For a frame that
    /// is a child of <world> an empty value indicates that the frame is
    /// attached to the world frame.
    /// \param[in] _frame The name of the attached-to frame.
    public: void SetAttachedTo(const std::string &_frame);

    /// \brief Get the pose of the frame object. This is the pose of the
    /// frame as specified in SDF
    /// (<frame><pose> ... </pose></frame>).
    /// Use SemanticPose to compute the pose relative to a specific frame.
    /// \return The pose of the frame object.
    public: const gz::math::Pose3d &RawPose() const;

    /// \brief Set the raw pose of the frame object. This is interpreted
    /// relative to the frame named in the //pose/@relative_to attribute.
    /// \sa const gz::math::Pose3d &RawPose() const
    /// \param[in] _pose The pose of the frame object.
    public: void SetRawPose(const gz::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame relative to which this
    /// frame's pose is expressed. An empty value indicates that the frame is
    /// expressed relative to the attached-to link.
    /// \return The name of the relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// frame's pose is expressed. An empty value indicates that the frame is
    /// expressed relative to the attached-to link.
    /// \param[in] _frame The name of the relative-to frame.
    public: void SetPoseRelativeTo(const std::string &_frame);

    /// \brief Get a pointer to the SDF element that was used during load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Resolve the attached-to body of this frame from the
    /// FrameAttachedToGraph. If this is in a __model__ scope, it returns
    /// the name of a link. In the world scope, it returns the name of a
    /// model or the world.
    /// \param[out] _body Name of body to which this frame is attached.
    /// \return Errors.
    public: Errors ResolveAttachedToBody(std::string &_body) const;

    /// \brief Get SemanticPose object of this object to aid in resolving
    /// poses.
    /// \return SemanticPose object for this link.
    public: sdf::SemanticPose SemanticPose() const;

    /// \brief Give a weak pointer to the FrameAttachedToGraph to be used
    /// for resolving attached bodies. This is private and is intended to
    /// be called by Model::Load or World::Load.
    /// \param[in] _graph Weak pointer to FrameAttachedToGraph.
    private: void SetFrameAttachedToGraph(
        std::weak_ptr<const FrameAttachedToGraph> _graph);

    /// \brief Give a weak pointer to the PoseRelativeToGraph to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Model::Load or World::Load.
    /// \param[in] _graph Weak pointer to PoseRelativeToGraph.
    private: void SetPoseRelativeToGraph(
        std::weak_ptr<const PoseRelativeToGraph> _graph);

    /// \brief Allow Model::Load and World::Load to call SetPoseRelativeToGraph.
    friend class Model;
    friend class World;

    /// \brief Private data pointer.
    private: FramePrivate *dataPtr = nullptr;
  };
  }
}
#endif
