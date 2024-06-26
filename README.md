# VVC Extension: Geometry-corrected geodesic motion model with per-frame camera motion coding

VVC Reference Software VTM-17.2 extension enabling geometry-corrected geodesic motion modeling with per-frame camera motion coding.

* A. Regensky and A. Kaup, "Geometry-Corrected Geodesic Motion Modeling with Per-Frame Camera Motion for 360-Degree Video Compression," in Proc. *IEEE Int. Conf. Acoustics, Speech, Signal Process.*, pp. 3215-3219, doi: [10.1109/ICASSP48485.2024.10446915](https://doi.org/10.1109/ICASSP48485.2024.10446915).

## Build

For building, `cmake` is required. On Linux, build using
```shell
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DEXTENSION_360_VIDEO=1
make -j
```

*Note:* For some `cmake` versions, an issue in internal dependency resolution for multi-thread compilation requires `make -j` to be called twice.
Do not cancel the first run even after errors are reported.
To avoid such problems, single-thread compilation is another possible option: `make -j 1`.

## Usage

In addition to the standard parameters for VTM-17.2 (EncoderApp) with 360lib integration, the following arguments are available:
* `--GED=0/1`: Activate geodesic motion model
* `--GEDFlavor=`: Set type of geodesic motion model (`vishwanath_original`, `vishwanath_modulated` - GED+orig, `regensky_geo_global` - GED+gcg, `regensky_geo_block` - GED+gcl)
* `--Projection=`: Set projection format of 360-degree video, set to `0` for equirectangular projection (ERP)
* `--Epipole=curPOC,-1,x,y,z`: Set epipole for geodesic motion model for frame curPOC as (x, y, z), second value "-1" is reserved for future use, set curPOC as -1 for global epipole.

Basic encoder example:
```shell
bin/EncoderAppStatic
-c cfg/encoder_randomaccess_vtm.cfg
-c cfg-360Lib/encoder_360_ERP.cfg
--SphFile=cfg-360Lib/360Lib/sphere_655362.txt
-i Broadway_6144x3072_60fps_8bit_420_erp.yuv
-b Broadway_2048x1024_60fps_8bit_420_erp.enc
-wdt 6144
-hgt 3072
--InputBitDepth=8
--InputChromaFormat=420
--CodingFaceWidth=2048
--CodingFaceHeight=1024
--WrapAroundOffset=2048
-fr 60
-f 32
-fs 0
-q 37
--GED=1
--GEDFlavor=regensky_geo_global
--Projection=0
--Epipole=-1,-1,1.0,0.0,0.0
```

Basic decoder example:
```shell
DecoderAppStatic
-b Broadway_2048x1024_60fps_8bit_420_erp.enc
-o Broadway_2048x1024_60fps_8bit_420_erp.enc.yuv
-d 8
```


## License

BSD 3-Clause License, see `COPYING`.

## Citation

If you use this software in your work, please cite:

```
@inproceedings{Regensky23_MultiModel360,
    title={Multi-Model Motion Prediction for 360-Degree Video Compression}, 
    author={Andy Regensky and Andr\'{e} Kaup},
    booktitle={Proceedings of the IEEE International Conference on Acoustics, Speech and Signal Processing},
    year={2024},
    month = apr,
    pages={3215-3219},
    doi={10.1109/ICASSP48485.2024.104469157}
}
```

# Original VVC VTM README:
VTM reference software for VVC
==============================

This software package is the reference software for Rec. ITU-T H.266 | ISO/IEC 23090-3 Versatile Video Coding (VVC). The reference software includes both encoder and decoder functionality.

Reference software is useful in aiding users of a video coding standard to establish and test conformance and interoperability, and to educate users and demonstrate the capabilities of the standard. For these purposes, this software is provided as an aid for the study and implementation of Versatile Video Coding.

The software has been jointly developed by the ITU-T Video Coding Experts Group (VCEG, Question 6 of ITU-T Study Group 16) and the ISO/IEC Moving Picture Experts Group (MPEG Joint Video Coding Team(s) with ITU-T SG 16, Working Group 5 of Subcommittee 29 of ISO/IEC Joint Technical Committee 1).

A software manual, which contains usage instructions, can be found in the "doc" subdirectory of this software package.

The source code is stored in a Git repository. The most recent version can be retrieved using the following commands:
```bash
git clone https://vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM.git
cd VVCSoftware_VTM
```

Build instructions
==================

The CMake tool is used to create platform-specific build files. 

Although CMake may be able to generate 32-bit binaries, **it is generally suggested to build 64-bit binaries**. 32-bit binaries are not able to access more than 2GB of RAM, which will not be sufficient for coding larger image formats. Building in 32-bit environments is not tested and will not be supported.


Build instructions for plain CMake (suggested)
----------------------------------------------

**Note:** A working CMake installation is required for building the software.

CMake generates configuration files for the compiler environment/development environment on each platform. 
The following is a list of examples for Windows (MS Visual Studio), macOS (Xcode) and Linux (make).

Open a command prompt on your system and change into the root directory of this project.

Create a build directory in the root directory:
```bash
mkdir build 
```

Use one of the following CMake commands, based on your platform. Feel free to change the commands to satisfy
your needs.

**Windows Visual Studio 2015/17/19 64 Bit:**

Use the proper generator string for generating Visual Studio files, e.g. for VS 2015:

```bash
cd build
cmake .. -G "Visual Studio 14 2015 Win64"
```

Then open the generated solution file in MS Visual Studio.

For VS 2017 use "Visual Studio 15 2017 Win64", for VS 2019 use "Visual Studio 16 2019".

Visual Studio 2019 also allows you to open the CMake directory directly. Choose "File->Open->CMake" for this option.

**macOS Xcode:**

For generating an Xcode workspace type:
```bash
cd build
cmake .. -G "Xcode"
```
Then open the generated work space in Xcode.

For generating Makefiles with optional non-default compilers, use the following commands:

```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-9 -DCMAKE_CXX_COMPILER=g++-9
```
In this example the brew installed GCC 9 is used for a release build.

**Linux**

For generating Linux Release Makefile:
```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
```
For generating Linux Debug Makefile:
```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
```

Then type
```bash
make -j
```

For more details, refer to the CMake documentation: https://cmake.org/cmake/help/latest/

Build instructions for make
---------------------------

**Note:** The build instructions in this section require the make tool and Python to be installed, which are
part of usual Linux and macOS environments. See below for installation instruction for Python and GnuWin32 
on Windows.

Open a command prompt on your system and change into the root directory of this project.

To use the default system compiler simply call:
```bash
make all
```


**MSYS2 and MinGW (Windows)**

**Note:** Build files for MSYS MinGW were added on request. The build platform is not regularily tested and can't be supported. 

Open an MSYS MinGW 64-Bit terminal and change into the root directory of this project.

Call:
```bash
make all toolset=gcc
```

The following tools need to be installed for MSYS2 and MinGW:

Download CMake: http://www.cmake.org/ and install it.

Python and GnuWin32 are not mandatory, but they simplify the build process for the user.

python:    https://www.python.org/downloads/release/python-371/

gnuwin32:  https://sourceforge.net/projects/getgnuwin32/files/getgnuwin32/0.6.30/GetGnuWin32-0.6.3.exe/download

To use MinGW, install MSYS2: http://repo.msys2.org/distrib/msys2-x86_64-latest.exe

Installation instructions: https://www.msys2.org/

Install the needed toolchains:
```bash
pacman -S --needed base-devel mingw-w64-i686-toolchain mingw-w64-x86_64-toolchain git subversion mingw-w64-i686-cmake mingw-w64-x86_64-cmake
```

