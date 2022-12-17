#!/usr/bin/env python3
import inspect
from pathlib import Path, PurePosixPath

""""Script for generating a C++ file that contains the content from all SDF files"""

# The list of supported SDF specification versions. This will let us drop
# versions without removing the directories.
SUPPORTED_SDF_VERSIONS = ['1.10', '1.9', '1.8', '1.7', '1.6', '1.5', '1.4', '1.3', '1.2']

# The list of supported SDF conversions. This list includes versions that
# a user can convert an existing SDF version to.
SUPPORTED_SDF_CONVERSIONS = ['1.10', '1.9', '1.8', '1.7', '1.6', '1.5', '1.4', '1.3']

# whitespace indentation for C++ code
INDENTATION = '  '
# newline character
NEWLINE = '\n'

def get_copyright_notice() -> str:
    """
    Provides the copyrigt notice for the C++ file

    :returns: copyright notice
    """
    res = inspect.cleandoc("""
    /*
     * Copyright 2022 Open Source Robotics Foundation
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
    """)
    return res + 2*NEWLINE


def get_file_header_prolog() -> str:
    """
    Provides the include statement, namespace and variable declaration of the C++ file

    :returns: prolog of the C++ file
    """
    res = inspect.cleandoc("""
    #include "EmbeddedSdf.hh"

    namespace sdf
    {
    inline namespace SDF_VERSION_NAMESPACE
    {
    /////////////////////////////////////////////////
    const std::map<std::string, std::string> &GetEmbeddedSdf()
    {
        static const std::map<std::string, std::string> result {
    """)
    return res + NEWLINE


def embed_sdf_content(arg_path: str, arg_file_content: str) -> str:
    """
    Generates a string pair with the folder and filename as well as the content of the file

    :param arg_path: Foldername and filename of the SDF
    :param arg_file_content: Content of the provided file
    :returns: raw string literal mapping pair for the std::map
    """
    res = []
    res.append('// NOLINT')
    res.append('{')
    res.append(f'"{arg_path}",')
    res.append('R"__sdf_literal__(')
    res.append(f'{arg_file_content}')
    res.append(')__sdf_literal__"')
    res.append('},')
    res.append('')
    return NEWLINE.join(res)


def get_map_content(arg_pathlist: Path) -> str:
    """
    Generates a string pair with the folder and filename as well as the content
    of the file in ascending order

    :param arg_pathlist: Foldername and all filenames inside it
    :returns: mapping pairs for the std::map
    """
    map_str = ''
    files = []
    for path in arg_pathlist:
        # dir separator is hardcoded to '/' in C++ mapping
        posix_path = PurePosixPath(path)
        files.append(str(posix_path))
    # get ascending order
    files.sort()
    for file in files:
        with Path(file).open() as f:
            content = f.read()
            map_str += embed_sdf_content(file, content)
    return map_str


def get_file_header_epilog() -> str:
    """
    Provides the return statement and the closing brackets of the C++ file

    :returns: epilog of the C++ file
    """
    res = inspect.cleandoc("""
        };

        return result;
    }

    }
    }  // namespace sdf

    """)
    return NEWLINE + res


if __name__ == "__main__":
    copyright = get_copyright_notice()
    prolog = get_file_header_prolog()

    map_str = ""
    for sdf_version in SUPPORTED_SDF_VERSIONS:
        pathlist = Path(sdf_version).glob('*.sdf')
        map_str += get_map_content(pathlist)

    for sdf_conversion in SUPPORTED_SDF_CONVERSIONS:
        pathlist = Path(sdf_conversion).glob('*.convert')
        map_str += get_map_content(pathlist)

    # remove the last comma
    map_str = map_str[:-2]

    epilog = get_file_header_epilog()

    output = copyright + prolog + map_str + epilog

    # output to stdin so that CMake can read it and create the appropriate file
    print(output)
