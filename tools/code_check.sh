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

CHECK_FILE_DIRS="./src ./include ./test/performance ./test/integration"
#cppcheck
CPPCHECK_BASE="cppcheck -q --suppressions-list=/tmp/sdf_cpp_check.suppress"
CPPCHECK_FILES=`find $CHECK_FILE_DIRS -name "*.cc"`
CPPCHECK_INCLUDES="-I include -I . -I src/urdf -I $builddir -I $builddir/include"
CPPCHECK_COMMAND1="-j 4 --enable=style,performance,portability,information $CPPCHECK_FILES"
# Unused function checking must happen in one job
CPPCHECK_COMMAND2="--enable=unusedFunction $CPPCHECK_FILES"
CPPCHECK_COMMAND3="-j 4 --enable=missingInclude $CPPCHECK_FILES $CPPCHECK_INCLUDES --check-config"
if [ $xmlout -eq 1 ]; then
  # Performance, style, portability, and information
  ($CPPCHECK_BASE --xml $CPPCHECK_COMMAND1) 2> $xmldir/cppcheck.xml

  # Unused function checking
  ($CPPCHECK_BASE --xml $CPPCHECK_COMMAND2) 2> $xmldir/cppcheck-unused-functions.xml

  # Check the configuration
  ($CPPCHECK_BASE --xml $CPPCHECK_COMMAND3) 2> $xmldir/cppcheck-configuration.xml
else
  # Performance, style, portability, and information
  $CPPCHECK_BASE $CPPCHECK_COMMAND1 2>&1

  # Unused function checking
  $CPPCHECK_BASE $CPPCHECK_COMMAND2 2>&1

  # Check the configuration
  $CPPCHECK_BASE $CPPCHECK_COMMAND3 2>&1
fi

# cpplint
# exclude urdf files for now, since they generate lots of errors
CPPLINT_FILES=`find $CHECK_FILE_DIRS -name "*.cc" -o -name "*.hh" | grep -iv urdf`
if [ $xmlout -eq 1 ]; then
  (echo $CPPLINT_FILES | xargs python tools/cpplint.py 2>&1) \
    | python tools/cpplint_to_cppcheckxml.py 2> $xmldir/cpplint.xml
else
  echo $CPPLINT_FILES | xargs python tools/cpplint.py 2>&1
fi
