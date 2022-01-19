# Converting between SDF and USD

This example shows how a world in a SDF file can be converted to [USD](https://graphics.pixar.com/usd/release/index.html).

## Requirements

You will need all of the dependencies for sdformat, along with the following additional dependencies:
* USD: [installation instructions](https://github.com/PixarAnimationStudios/USD/blob/release/README.md#getting-and-building-the-code)
* [ignition-common4](https://github.com/ignitionrobotics/ign-common)

## Setup

Build sdformat, and then run the following commands to build the example (run these commands from this example directory):
```bash
mkdir build
cd build
cmake ..
make
```

You should now have an executable named `sdf2usd`, which can be used to convert a SDF world file to a USD file.
The following command converts the example `shapes.sdf` file to its USD representation, stored in a file called `shapes.usd` (run this command from the `build` directory):
```bash
./sdf2usd ../shapes.sdf shapes.usd
```

You can now view the contents of the generated USD file with `usdview` (this should have been installed when setting up the USD dependency):
```bash
usdview shapes.usd
```
