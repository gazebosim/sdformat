@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%

cmake -G "NMake Makefiles"^
  -DBOOST_ROOT:STRING="%cd%\..\..\boost_1_56_0"^
  -DBOOST_LIBRARYDIR:STRING="%cd%\..\..\boost_1_56_0\lib64-msvc-12.0"^
  -DCMAKE_INSTALL_PREFIX="install/%build_type%"^
  -DIGNITION-MATH_INCLUDE_DIRS:STRING="%cd%\..\..\ign-math\build\install\%build_type%\include\ignition\math2"^
  -DIGNITION-MATH_LIBRARY_DIRS:STRING="%cd%\..\..\ign-math\build\install\%build_type%\lib"^
  -DIGNITION-MATH_LIBRARIES="ignition-math2"^
  -DCMAKE_BUILD_TYPE="%build_type%" ..
