#!/usr/bin/env python3

import argparse
import os
import sys

from typing import List

import xml.etree.ElementTree

def get_output_filename(
        input_file: str,
        output_dir: str,
        ) -> str:
    filename = os.path.basename(input_file)
    filename = filename.replace('.sdf', '.xsd')
    return os.path.join(output_dir, filename)


def printDocumentation(elem: xml.etree.ElementTree.Element, indent: int) -> List[str]:
    lines = []
    return lines

def printXSD(elem: xml.etree.ElementTree.Element, indent: int) -> List[str]:
    lines = []


    return lines

def process(
    source_dir: str,
    input_file: str,
    output_dir: str) -> int:
    """
    Process XML file
    """

    if not os.path.exists(input_file):
        print(f'Input file {input_file} does not exist')
        return -1

    output_file = get_output_filename(input_file, output_dir)
    tree = xml.etree.ElementTree.parse(input_file)
    root = tree.getroot()

    elements = []
    for element in root.iter('element'):
        elements.append(element)
        print(element.__dict__)

    with open(output_file, 'w') as f:
        print(f'Writing to: {output_file}')

        f.write("<?xml version='1.0' encoding='UTF-8'?>\n")
        f.write("<xsd:schema xmlns:xsd='http://www.w3.org/2001/XMLSchema'>\n")

        for element in elements:
            printXSD(element, 0)
        f.write("</xsd:schema>\n")
        pass

    return 0



def main(argv: List[str]=sys.argv[1:]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dir", type=str)
    parser.add_argument("--input-file", type=str)
    parser.add_argument("--output-dir", type=str)

    args = parser.parse_args(argv)
    process(args.source_dir, args.input_file, args.output_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
