/*
 * Copyright 2023 Open Source Robotics Foundation
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
#ifndef SDF_PROJECTOR_HH_
#define SDF_PROJECTOR_HH_

#include <memory>
#include <string>

#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "sdf/Plugin.hh"
#include "sdf/SemanticPose.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  // Forward declarations.
  struct PoseRelativeToGraph;

  /// \brief A description of a projector, which can be attached
  /// to a link. A projector can be used to project texture onto other
  /// visuals
  class SDFORMAT_VISIBLE Projector
  {
    /// \brief Default constructor
    public: Projector();

    /// \brief Get the schema file name accessor
    public: static inline std::string_view SchemaFile();
    
    /// \brief Load the projector based on an element pointer. This is
    /// *not* the usual entry point. Typical usage of the SDF DOM is through
    /// the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the projector.
    /// The name of the projector should be unique within the scope of
    /// a Link.
    /// \return Name of the projector.
    public: std::string Name() const;

    /// \brief Set the name of the projector.
    /// The name of the projector should be unique within the scope of
    /// a Link.
    /// \param[in] _name Name of the projector.
    public: void SetName(const std::string &_name);

    /// \brief Get the near clip distance.
    /// \return The near clip distance.
    public: double NearClip() const;

    /// \brief Set the near clip distance.
    /// \param[in] _near The near clip distance.
    public: void SetNearClip(double _near);

    /// \brief Get the far clip distance.
    /// \return The far clip distance.
    public: double FarClip() const;

    /// \brief Set the far clip distance.
    /// \param[in] _far The far clip distance.
    public: void SetFarClip(double _far);

    /// \brief Get the horizontal field of view in radians.
    /// \return The horizontal field of view in radians.
    public: gz::math::Angle HorizontalFov() const;

    /// \brief Set the horizontal field of view in radians.
    /// \param[in] _hfov The horizontal field of view in radians.
    public: void SetHorizontalFov(const gz::math::Angle &_hfov);

    /// \brief Get the visibility flags of a projector
    /// \return visibility flags
    public: uint32_t VisibilityFlags() const;

    /// \brief Set the visibility flags of a projector
    /// \param[in] _flags visibility flags
    public: void SetVisibilityFlags(uint32_t _flags);

    /// \brief Get the texture filename. This will be an empty string if
    /// a texture has not been set.
    /// \return Filename of the texture, or empty string if a texture
    /// has not been specified.
    public: std::string Texture() const;

    /// \brief Set the texture filename.
    /// \param[in] _map Filename of the texture
    public: void SetTexture(const std::string &_map);

    /// \brief Get the plugins attached to this projector.
    /// \return A vector of Plugin, which will be empty if there are no
    /// plugins.
    public: const sdf::Plugins &Plugins() const;

    /// \brief Get a mutable vector of plugins attached to this projector.
    /// \return A vector of Plugin, which will be empty if there are no
    /// plugins.
    public: sdf::Plugins &Plugins();

    /// \brief Remove all plugins
    public: void ClearPlugins();

    /// \brief Add a plugin to this projector.
    /// \param[in] _plugin Plugin to add.
    public: void AddPlugin(const Plugin &_plugin);

    /// \brief Get the pose of the projector. This is the pose of the
    /// projector as specified in SDF
    /// (<projector><pose> ... </pose></projector>).
    /// \return The pose of the projector.
    public: const gz::math::Pose3d &RawPose() const;

    /// \brief Set the pose of the projector object.
    /// \sa const gz::math::Pose3d &RawPose() const
    /// \param[in] _pose The pose of the projector.
    public: void SetRawPose(const gz::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame relative to which this
    /// projector's pose is expressed. An empty value indicates that the frame
    /// is relative to the parent link.
    /// \return The name of the pose relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// projector's pose is expressed. An empty value indicates that the frame
    /// is relative to the parent link.
    /// \param[in] _frame The name of the pose relative-to frame.
    public: void SetPoseRelativeTo(const std::string &_frame);

    /// \brief Get SemanticPose object of this object to aid in resolving poses.
    /// \return SemanticPose object for this projector.
    public: sdf::SemanticPose SemanticPose() const;

    /// \brief Get a pointer to the SDF element that was used during load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief The path to the file where this element was loaded from.
    /// \return Full path to the file on disk.
    public: const std::string &FilePath() const;

    /// \brief Set the path to the file where this element was loaded from.
    /// \paramp[in] _filePath Full path to the file on disk.
    public: void SetFilePath(const std::string &_filePath);

    /// \brief Create and return an SDF element filled with data from this
    /// projector.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated projector values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Set the name of the xml parent of this object, to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph.
    /// \param[in] _xmlParentName Name of xml parent object.
    private: void SetXmlParentName(const std::string &_xmlParentName);

    /// \brief Set a weak pointer to the PoseRelativeToGraph to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph.
    /// \param[in] _graph scoped PoseRelativeToGraph object.
    private: void SetPoseRelativeToGraph(
                 sdf::ScopedGraph<PoseRelativeToGraph> _graph);

    /// \brief Allow Link::SetPoseRelativeToGraph to call SetXmlParentName
    /// and SetPoseRelativeToGraph, but Link::SetPoseRelativeToGraph is
    /// a private function, so we need to befriend the entire class.
    friend class Link;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
