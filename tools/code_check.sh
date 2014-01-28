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

# Use a suppression file for unused function checking
SUPPRESS=/tmp/sdf_cpp_check.suppress
echo "*:src/SDF.cc" > $SUPPRESS
echo "*:src/Assert.cc" >> $SUPPRESS
echo "*:src/Console.cc" >> $SUPPRESS
echo "*:src/parser.cc" >> $SUPPRESS
echo "*:src/parser_urdf.cc" >> $SUPPRESS

CHECK_FILE_DIRS="./src ./include ./test/performance ./test/integration"

#cppcheck
CPPCHECK_BASE="cppcheck -q"
CPPCHECK_BASE2="cppcheck -q --suppressions-list=$SUPPRESS"
CPPCHECK_FILES=`find $CHECK_FILE_DIRS -name "*.cc"`
CPPCHECK_INCLUDES="-I include -I . -I src/urdf -I $builddir -I $builddir/include"
CPPCHECK_COMMAND1="-j 4 --enable=style,performance,portability,information $CPPCHECK_FILES"
# Unused function checking must happen in one job
CPPCHECK_COMMAND2="--enable=unusedFunction $CPPCHECK_FILES"
# -j 4 was used previously in CPPCHECK_COMMAND3 but it will generate a false
# warning as described in bug: 
# http://sourceforge.net/apps/trac/cppcheck/ticket/4946
CPPCHECK_COMMAND3="-j 1 --enable=missingInclude --suppress=missingIncludeSystem $CPPCHECK_FILES $CPPCHECK_INCLUDES --check-config"
if [ $xmlout -eq 1 ]; then
  # Performance, style, portability, and information
  ($CPPCHECK_BASE --xml $CPPCHECK_COMMAND1) 2> $xmldir/cppcheck.xml

  # Unused function checking
  ($CPPCHECK_BASE2 --xml $CPPCHECK_COMMAND2) 2> $xmldir/cppcheck-unused-functions.xml

  # Check the configuration
  ($CPPCHECK_BASE --xml $CPPCHECK_COMMAND3) 2> $xmldir/cppcheck-configuration.xml
else
  # Performance, style, portability, and information
  $CPPCHECK_BASE $CPPCHECK_COMMAND1 2>&1

  # Unused function checking
  $CPPCHECK_BASE2 $CPPCHECK_COMMAND2 2>&1

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
