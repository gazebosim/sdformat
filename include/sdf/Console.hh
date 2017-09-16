/*
 * Copyright 2011 Nate Koenig
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

#ifndef _SDF_CONSOLE_HH_
#define _SDF_CONSOLE_HH_

#include <iostream>
#include <fstream>
#include <string>

#include <memory>

#include "sdf/system_util.hh"

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

namespace sdf
{
  /// \addtogroup sdf SDF
  /// \{

  /// \brief Output a debug message
  #define sdfdbg (sdf::Console::Instance()->Log("Dbg", \
    __FILE__, __LINE__))

  /// \brief Output a message
  #define sdfmsg (sdf::Console::Instance()->ColorMsg("Msg", \
    __FILE__, __LINE__, 32))

  /// \brief Output a warning message
  #define sdfwarn (sdf::Console::Instance()->ColorMsg("Warning", \
        __FILE__, __LINE__, 33))

  /// \brief Output an error message
  #define sdferr (sdf::Console::Instance()->ColorMsg("Error", \
        __FILE__, __LINE__, 31))

  class ConsolePrivate;
  class Console;

  /// \def ConsolePtr
  /// \brief Shared pointer to a Console Element
  typedef std::shared_ptr<Console> ConsolePtr;

  /// \brief Message, error, warning, and logging functionality
  class SDFORMAT_VISIBLE Console
  {
    /// \brief An ostream-like class that we'll use for logging.
    public: class SDFORMAT_VISIBLE ConsoleStream
    {
      /// \brief Constructor.
      /// \param[in] _stream Pointer to an output stream operator. Can bee
      /// NULL.
      public: ConsoleStream(std::ostream *_stream) :
              stream(_stream) {}

      /// \brief Redirect whatever is passed in to both our ostream
      ///        (if non-NULL) and the log file (if open).
      /// \param[in] _rhs Content to be logged.
      /// \return Reference to myself.
      public: template <class T>
        ConsoleStream &operator<<(const T &_rhs);

      /// \brief Print a prefix to both terminal and log file.
      /// \param[in] _lbl Text label
      /// \param[in] _file File containing the error
      /// \param[in] _line Line containing the error
      /// \param[in] _color Color to make the label.  Used only on terminal.
      public: void Prefix(const std::string &_lbl,
                          const std::string &_file,
                          unsigned int _line, int _color);

      /// \brief The ostream to log to; can be NULL.
      private: std::ostream *stream;
    };

    /// \brief Default constructor
    private: Console();

    /// \brief Destructor
    public: virtual ~Console();

    /// \brief Return an instance to this class.
    public: static ConsolePtr Instance();

    /// \brief Set quiet output
    /// \param[in] q True to prevent warning
    public: void SetQuiet(bool _q);

    /// \brief Use this to output a colored message to the terminal
    /// \param[in] _lbl Text label
    /// \param[in] _file File containing the error
    /// \param[in] _line Line containing the error
    /// \param[in] _color Color to make the label
    /// \return Reference to an output stream
    public: ConsoleStream &ColorMsg(const std::string &lbl,
                                    const std::string &file,
                                    unsigned int line, int color);

    /// \brief Use this to output a message to a log file
    /// \return Reference to output stream
    public: ConsoleStream &Log(const std::string &lbl,
                               const std::string &file,
                               unsigned int line);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ConsolePrivate> dataPtr;
  };

  /// \internal
  /// \brief Private data for Console
  class ConsolePrivate
  {
    /// \brief Constructor
    public: ConsolePrivate() : msgStream(&std::cerr), logStream(NULL) {}

    /// \brief message stream
    public: Console::ConsoleStream msgStream;

    /// \brief log stream
    public: Console::ConsoleStream logStream;

    /// \brief logfile stream
    public: std::ofstream logFileStream;
  };

  ///////////////////////////////////////////////
  template <class T>
  Console::ConsoleStream &Console::ConsoleStream::operator<<(const T &_rhs)
  {
    if (this->stream)
      *this->stream << _rhs;

    if (Console::Instance()->dataPtr->logFileStream.is_open())
    {
      Console::Instance()->dataPtr->logFileStream << _rhs;
      Console::Instance()->dataPtr->logFileStream.flush();
    }

    return *this;
  }

  /// \}
}

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
