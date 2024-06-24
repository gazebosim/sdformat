#!/usr/bin/env python3
"""
Script for generating a C++ file that contains the content from all SDF files
"""

from typing import List, Optional

import argparse
import inspect
import sys
from pathlib import Path


# The list of supported SDF specification versions. This will let us drop
# versions without removing the directories.
SUPPORTED_SDF_VERSIONS = [
    "1.10",
    "1.9",
    "1.8",
    "1.7",
    "1.6",
    "1.5",
    "1.4",
    "1.3",
    "1.2",
]

# The list of supported SDF conversions. This list includes versions that
# a user can convert an existing SDF version to.
SUPPORTED_SDF_CONVERSIONS = ["1.10", "1.9", "1.8", "1.7", "1.6", "1.5", "1.4", "1.3"]

# whitespace indentation for C++ code
INDENTATION = "  "

# newline character
NEWLINE = "\n"


def get_copyright_notice() -> str:
    """
    Provides the copyright notice for the C++ file

    :returns: copyright notice
    """
    res = inspect.cleandoc(
        """
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
    """
    )
    return res + 2 * NEWLINE


def get_file_header_prolog() -> str:
    """
    Provides the include statement, namespace and variable declaration of the C++ file

    :returns: prolog of the C++ file
    """
    res = inspect.cleandoc(
        """
    #include "EmbeddedSdf.hh"

    namespace sdf
    {
    inline namespace SDF_VERSION_NAMESPACE
    {
    /////////////////////////////////////////////////
    const std::map<std::string, std::string> &GetEmbeddedSdf()
    {
        static const std::map<std::string, std::string> result {
    """
    )
    return res + NEWLINE


def embed_sdf_content(arg_path: str, arg_file_content: str) -> str:
    """
    Generates a string pair with the folder and filename
    as well as the content of the file

    :param arg_path: Foldername and filename of the SDF
    :param arg_file_content: Content of the provided file
    :returns: raw string literal mapping pair for the std::map
    """
    res = []
    res.append("// NOLINT")
    res.append("{")
    res.append(f'"{arg_path}",')
    res.append('R"__sdf_literal__(')
    res.append(f"{arg_file_content}")
    res.append(')__sdf_literal__"')
    res.append("}")
    return NEWLINE.join(res)


def get_file_header_epilog() -> str:
    """
    Provides the return statement and the closing brackets of the C++ file

    :returns: epilog of the C++ file
    """
    res = inspect.cleandoc(
        """
        };

        return result;
    }

    }
    }  // namespace sdf

    """
    )
    return NEWLINE + res


def write_output(file_content: str, output_filename: str) -> None:
    """
    Print the content of the EmbeddedSdf.cc to a file
    """
    copyright_notice = get_copyright_notice()
    prolog = get_file_header_prolog()
    epilog = get_file_header_epilog()
    output_content = copyright_notice + prolog + file_content + epilog

    with open(output_filename, "w", encoding="utf8") as output_file:
        output_file.write(output_content)


def collect_file_locations() -> List[Path]:
    paths: List[Path] = []

    for sdf_version in SUPPORTED_SDF_VERSIONS:
        paths.extend(Path(sdf_version).glob("*.sdf"))
    for sdf_conversion in SUPPORTED_SDF_CONVERSIONS:
        paths.extend(Path(sdf_conversion).glob("*.convert"))
    return paths


def generate_map_content(paths: List[Path], relative_to: Optional[str] = None) -> str:
    '''
    Generate the EmbeddedSdf.cc content
    '''
    content = []
    for path in paths:
        with open(path, "r", encoding="utf8") as input_sdf:
            file_content = input_sdf.read()
            # Strip relative path if requested
            if relative_to is not None:
                _, relative_path = str(path).split(relative_to)
                path = relative_path
            content.append(embed_sdf_content(str(path), file_content))
    return ",".join(content)


def main(args=None) -> int:
    '''
    Main entrypoint
    '''
    if args is None:
        args = sys.argv[1:]

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--sdf-root",
        default=None,
        help="Directory containing sdf description files for each version",
    )
    parser.add_argument(
        "--input-files", nargs="*", help="List of input files to be embedded"
    )
    parser.add_argument(
        "--output-file", help="File to output embeddedsdf.cc content to"
    )

    args = parser.parse_args(args)
    if not args.input_files or len(args.input_files) == 0:
        paths = collect_file_locations()
    else:
        paths = [Path(f) for f in args.input_files]
    content = generate_map_content(paths, args.sdf_root)
    write_output(content, args.output_file)
    return 0


if __name__ == "__main__":
    sys.exit(main())
