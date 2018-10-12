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
#ifndef SDF_GUI_HH_
#define SDF_GUI_HH_

#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declarations.
  class GuiPrivate;

  class SDFORMAT_VISIBLE Gui
  {
    /// \brief Default constructor
    public: Gui();

    /// \brief Copy constructor
    /// \param[in] _gui Gui element to copy.
    public: Gui(const Gui &_gui);

    /// \brief Move constructor
    /// \param[in] _gui Gui to move.
    public: Gui(Gui &&_gui);

    /// \brief Destructor
    public: ~Gui();

    /// \brief Load the gui based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get whether the Gui should be fullscreen.
    /// \return True if the Gui should be fullscreen.
    public: bool Fullscreen() const;

    /// \brief Set whether the Gui should be full screen.
    /// \param[in] _fullscreen True indicates that the Gui should be
    /// fullscreen.
    public: void SetFullscreen(const bool _fullscreen) const;

    /// \brief Equality operator that returns true if this Gui
    /// instance equals the given Gui instance.
    /// \param[in] _gui Gui instance to compare.
    /// \return True if this instance equals the given Gui.
    public: bool operator==(const Gui &_gui) const;

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: GuiPrivate *dataPtr = nullptr;
  };
}
#endif
