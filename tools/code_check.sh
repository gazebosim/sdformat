#!/bin/sh

# Jenkins will pass -xml, in which case we want to generate XML output
xmlout=0
if test "$1" = "-xmldir" -a -n "$2"; then
  xmlout=1
  xmldir=$2
  mkdir -p $xmldir
  rm -rf $xmldir/*.xml
  # Assuming that Jenkins called, the `build` directory is a sibling to the src dir
  builddir=../build
else
  # This is a heuristic guess; not every developer puts the `build` dir in the src dir
  builddir=./build
fi

echo "*:src/urdf*" > /tmp/sdf_cpp_check.suppress

#cppcheck
if [ $xmlout -eq 1 ]; then
  (cppcheck --enable=all -q --suppressions-list=/tmp/sdf_cpp_check.suppress --xml `find ./src -name "*.cc"` -I include -I . -I src/urdf -I $builddir -I $builddir/include  --check-config) 2> $xmldir/cppcheck.xml
else
  cppcheck --enable=all -q --suppressions-list=/tmp/sdf_cpp_check.suppress `find ./src -name "*.cc"` -I include -I . -I src/urdf -I $builddir -I $builddir/include --check-config
fi

# cpplint
if [ $xmlout -eq 1 ]; then
  (find ./src ./include -name "*.cc" -o -name "*.hh"  -print0 | xargs -0 python tools/cpplint.py 2>&1) | python tools/cpplint_to_cppcheckxml.py 2> $xmldir/cpplint.xml
else
  find ./src ./include -name "*.cc" -o -name "*.hh" -print0 | xargs -0 python tools/cpplint.py 2>&1
fi
