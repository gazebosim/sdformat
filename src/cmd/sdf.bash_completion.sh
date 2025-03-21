#!/usr/bin/env bash
#
# Copyright (C) 2022 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# bash tab-completion

# This is a per-library function definition, used in conjunction with the
# top-level entry point in gz-tools.

GZ_SDF_COMPLETION_LIST="
  -k --check
  -d --describe
  -p --print
  -g --graph
  --inertial-stats
  -h --help
  --force-version
  --versions
"

function _gz_sdf
{
  if [[ ${COMP_WORDS[COMP_CWORD]} == -* ]]; then
    # Specify options (-*) word list for this subcommand
    COMPREPLY=($(compgen -W "$GZ_SDF_COMPLETION_LIST" \
      -- "${COMP_WORDS[COMP_CWORD]}" ))
    return
  else
    # Just use bash default auto-complete, because we never have two
    # subcommands in the same line. If that is ever needed, change here to
    # detect subsequent subcommands
    COMPREPLY=($(compgen -o default -- "${COMP_WORDS[COMP_CWORD]}"))
    return
  fi
}

function _gz_sdf_flags
{
  for word in $GZ_SDF_COMPLETION_LIST; do
    echo "$word"
  done
}
