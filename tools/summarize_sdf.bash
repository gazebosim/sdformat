#!/bin/bash

echo Summarizing content of $1
echo
echo Name of Root Model:
xmllint --xpath '//sdf/model/@name' $1
echo
echo Names of Worlds:
xmllint --xpath '//sdf/world/@name' $1
echo Names of Models contained by a World:
xmllint --xpath '//world/model/@name' $1
echo
echo Names of Nested models:
xmllint --xpath '//model/model/@name' $1
echo
echo Names and Filenames of Plugins:
echo Names:
xmllint --xpath '//plugin/@name' $1
echo Filenames:
xmllint --xpath '//plugin/@filename' $1
echo
echo Frame Graph:
gz sdf --graph frame $1
