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
#ifndef SDF_SCENE_HH_
#define SDF_SCENE_HH_

#include <gz/math/Color.hh>

#include "sdf/Element.hh"
#include "sdf/Sky.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declarations.
  class ScenePrivate;

  class SDFORMAT_VISIBLE Scene
  {
    /// \brief Default constructor
    public: Scene();

    /// \brief Copy constructor
    /// \param[in] _scene Scene element to copy.
    public: Scene(const Scene &_scene);

    /// \brief Move constructor
    /// \param[in] _scene Scene to move.
    public: Scene(Scene &&_scene) noexcept;

    /// \brief Destructor
    public: ~Scene();

    /// \brief Assignment operator.
    /// \param[in] _scene The scene to set values from.
    /// \return *this
    public: Scene &operator=(const Scene &_scene);

    /// \brief Move assignment operator.
    /// \param[in] _workflow The scene to move from.
    /// \return *this
    public: Scene &operator=(Scene &&_scene);

    /// \brief Load the scene based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the ambient color of the scene
    /// \return Scene ambient color
    public: gz::math::Color Ambient() const;

    /// \brief Set the ambient color of the scene
    /// \param[in] _ambient Ambient color to set to
    public: void SetAmbient(const gz::math::Color &_ambient);

    /// \brief Get the background color of the scene
    /// \return Scene background color
    public: gz::math::Color Background() const;

    /// \brief Set the background color of the scene
    /// \param[in] _background Background color to set to
    public: void SetBackground(const gz::math::Color &_background);

    /// \brief Get whether grid is enabled
    /// \return True if grid is enabled
    public: bool Grid() const;

    /// \brief Set whether the grid should be enabled
    /// \param[in] enabled True to enable grid
    public: void SetGrid(const bool _enabled);

    /// \brief Get whether origin visual is enabled
    /// \return True if origin visual is enabled
    public: bool OriginVisual() const;

    /// \brief Set whether the origin visual should be enabled
    /// \param[in] enabled True to enable origin visual
    public: void SetOriginVisual(const bool _enabled);

    /// \brief Get whether shadows are enabled
    /// \return True if shadows are enabled
    public: bool Shadows() const;

    /// \brief Set whether shadows should be enabled
    /// \param[in] enabled True to enable shadows
    public: void SetShadows(const bool _shadows);

    /// \brief Set sky
    /// \param[in] _sky Sky to set to
    public: void SetSky(const Sky &_sky);

    /// \brief Get sky
    /// \return Sky
    public: const sdf::Sky *Sky() const;

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: ScenePrivate *dataPtr = nullptr;
  };
  }
}
#endif
