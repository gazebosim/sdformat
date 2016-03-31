# Installation on Windows

This documentation describes how to set up a workspace for trying to compile sdformat on Windows.

## Supported compilers

At this moment, compilation has been tested on Windows 7 and 8.1 and is supported 
when using Visual Studio 2013. Patches for other versions are welcome.

## Installation

Totally experimental, using pre-compiled binaries in a local workspace.  To
make things easier, use a MinGW shell for your editing work, and only use the
Windows `cmd` for configuring and building.

1. Make a directory to work in, e.g.:

        mkdir sdformat-ws
        cd sdformat-ws

1. Download boost into that directory:

    - [boost 1.56.0](http://packages.osrfoundation.org/win32/deps/boost_1_56_0.zip)

1. Unzip it in sdformat-ws.

1. Clone sdformat

        hg clone https://bitbucket.org/osrf/sdformat

1. Load your compiler setup, e.g. (note that we are asking for the 64-bit toolchain here):

        "C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat" x86_amd64

1. Configure and build sdformat:

        cd sdformat
        mkdir build
        cd build
        ..\configure
        nmake
        nmake install

    You should now have an installation of sdformat in sdformat-ws/sdformat/build/install/Release.

    Once this all works (which it does with tender love and care): you should now have an installation of sdformat in sdformat-ws/sdformat/build/install/Release.
