# Converting between SDF and USD

This example shows how a world in a SDF file can be converted to [USD](https://graphics.pixar.com/usd/release/index.html).

## Requirements

You will need all of the dependencies for sdformat, along with the following additional dependencies:
* USD: [installation instructions](https://github.com/PixarAnimationStudios/USD/blob/release/README.md#getting-and-building-the-code)
* [ignition-common5](https://github.com/ignitionrobotics/ign-common)
* [ignition-utils1 (including the CLI component)](https://github.com/ignitionrobotics/ign-utils)

## Setup

Build sdformat. The steps below follow a traditional cmake build, but sdformat
can also be built with [colcon](https://colcon.readthedocs.io/en/released/index.html):
```bash
git clone https://github.com/ignitionrobotics/sdformat.git
cd sdformat
mkdir build
cd build
cmake ..
make
```

You should now have an executable named `sdf2usd` in the `sdformat/build/bin` directory.
This executable can be used to convert a SDF world file to a USD file.
To see how the executable works, run the following command from the `sdformat/build/bin` directory:
```bash
./sdf2usd -h
```

To convert [shapes_world.sdf](https://github.com/ignitionrobotics/sdformat/blob/sdf12/test/sdf/shapes_world.sdf) to its USD representation as a file called `shapes.usd`, run the following commands:

```bash
wget https://raw.githubusercontent.com/ignitionrobotics/sdformat/sdf12/test/sdf/shapes_world.sdf
./sdf2usd shapes_world.sdf shapes.usd
```

You can now view the contents of the generated USD file with `usdcat` (this should have been installed when setting up the USD dependency):
```bash
usdcat shapes.usd
```

To see the visual representation of the USD world, run `usdview` (this should have also been installed when setting up the USD dependency):
```bash
usdview shapes.usd
```

### Note about building with colcon
You may need to add the USD library path to your `LD_LIBRARY_PATH` environment variable after sourcing the colcon workspace.
If the USD library path is not a part of `LD_LIBRARY_PATH`, you will probably see the following error when running the `sdf2usd` executable:
```bash
sdf2usd: error while loading shared libraries: libusd_usd.so: cannot open shared object file: No such file or directory
```
The typical USD library path is `<usd_installation_path>/lib`.
So, if you installed USD at `/usr/local/USD`, the following command on Linux properly updates the `LD_LIBRARY_PATH` environment variable:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/USD/lib
```

Another thing to note if building with colcon is that after sourcing the workspace with sdformat,
the `sdf2usd` executable can be run without having to go to the `sdformat/build/bin` directory.
So, instead of going to that directory and running `./sdf2usd ...`, you should be able to run `sdf2usd ...` from anywhere.
