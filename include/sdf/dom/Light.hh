/*
 * Copyright 2017 Open Source Robotics Foundation
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
#ifndef SDF_DOM_LIGHT_HH_
#define SDF_DOM_LIGHT_HH_

#include <string>

#include "sdf/Element.hh"
#include "sdf/system_util.hh"
#include "sdf/dom/Entity.hh"

namespace sdf
{
  // Forward declare private data class.
  class LightPrivate;

  /// \brief A light represents a source of visible light.
  class SDFORMAT_VISIBLE Light : public Entity
  {
    /// \enum LightType
    /// \brief This enum contains the set of supported light types. A light
    /// type defines how light should be generated.
    public: enum class LightType
    {
      /// \brief A directional light has parallel light rays.
      DIRECTIONAL,

      /// \brief A point light acts like a light buld.
      POINT,

      /// \brief A spot light generates a cone of light.
      SPOT,

      /// \brief An unknown or unsupported light type.
      UNKNOWN
    };

    /// \brief Constructor
    public: Light();

    /// \brief Destructor
    public: ~Light();

    /// \brief Load the light based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf An SDF Element pointer to a <light> element.
    /// \return True when no errors were encountered.
    public: bool Load(sdf::ElementPtr _sdf);

    /// \brief Get the type of the light.
    /// \return Type of the light.
    public: LightType Type() const;

    /// \brief Get a string version of the light type.
    /// \return Name of the light type.
    public: std::string Typename() const;

    /// \brief Set the type of the light.
    /// \param[in] _type Type of the light.
    public: void SetType(const LightType _type);

    /// \brief Get whether this light casts shadows.
    /// \return True if the light is set to cast shadows.
    public: bool CastShadows() const;

    /// \brief Set whether this light casts shadows.
    /// \param[in] _shadows True if the light should cast shadows.
    public: void SetCastShadows(const bool _shadows);

    /// \brief Get the diffuse color of the light.
    /// \return The diffuse light color.
    public: ignition::math::Color Diffuse() const;

    /// \brief Set the diffuse color of the light.
    /// \param[in] _color The diffuse light color.
    public: void SetDiffuse(const ignition::math::Color &_color);

    /// \brief Get the specular color of the light.
    /// \return The specular light color.
    public: ignition::math::Color Specular() const;

    /// \brief Set the specular color of the light.
    /// \param[in] _color The specular light color.
    public: void SetSpecular(const ignition::math::Color &_color);

    /// \brief Print debug information to standard out.
    /// \param[in] _prefix String to prefix all output.
    public: void Print(const std::string &_prefix = "") const;

    /// \brief Private data pointer.
    private: LightPrivate *dataPtr;
  };
}
#endif
