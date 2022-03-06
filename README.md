# SRM Vision Project Remastered

## Build project

### Requirements

To build this project, all of these libraries should be pre-installed:

1. G++ compiler 8.x with binary `g++-8` available.
2. [CUDA](https://developer.nvidia.com/cuda-toolkit) 11.x, [cuDNN](https://developer.nvidia.com/cudnn)
   and [TensorRT](https://developer.nvidia.com/tensorrt) 7.x.
3. [OpenCV](https://opencv.org/) 4.5.x.
4. [Ceres Solver](http://ceres-solver.org/) 2.x.
5. [FMT Library](https://github.com/fmtlib/fmt) 8.x.
6. [HikRobots MVS](https://www.hikrobotics.com/cn/machinevision/service/download?module=0)
   and [DaHeng imaging camera drivers](https://www.daheng-imaging.com/list-59-1.html).

### Build with CMake

Steps to build with CMake:

1. Open `CMakeLists.txt`, check paths after `# Custom library path.` can be reached, or correct them to fit your system
   environment.
2. Make a new directory naming `build` and go there:
   ```shell
   mkdir -p build && cd build
   ```
3. Predict cmake initialization command. To build a debug binary for performance testing and debugging,
   use `-DCMAKE_BUILD_TYPE=Debug` flag, otherwise use `-DCMAKE_BUILD_TYPE=Release` flag. Here is an example of building
   a release binary:
   ```shell
   cmake -DCMAKE_BUILD_TYPE=Release ..
   ```
   Do not miss the last `..`.
4. If no error occurs, run `make -jn` (replace `n` with your cpu core number like `make -j4`) to build executable file.
5. Predict `./SRM2021 --type=controller_type` to run, with `controller_type` replaced with a valid type in this list:
    - infantry
    - Hero
    - Sentry (lower gimbal)

## Benchmarks

Some math algorithms in this project support x86_64 SSE2 and ARMv8 NEON for hardware acceleration. To benchmark these
algorithms, build all targets with CMake and run executables in `benchmark` folder. Here are all benchmark items:

1. `rsqrt` includes hardware acceleration, "magic
   number" [0x5f375a86](https://en.wikipedia.org/wiki/Fast_inverse_square_root) and standard inverse square root
   algorithms.
2. `trigonometric` includes hardware acceleration and standard trigonometric functions.
3. `sse2` is an x86-only basic benchmark for float performance.

All of these accelerated method are based on [SSEMath](http://gruntthepeon.free.fr/ssemath/) (
under [Zlib license](https://opensource.org/licenses/Zlib)) and [SSE2NEON](https://github.com/DLTcollab/sse2neon) (
under [MIT license](https://opensource.org/licenses/MIT)) open source projects.

Note that built benchmark target on mismatching architectures will be empty to avoid errors when building.

## Documentation

This project uses [doxygen](https://www.doxygen.nl/index.html) for documentation. All docs in code follow
[doxygen documentation rules](https://www.doxygen.nl/manual/docblocks.html#cppblock) and can be identified by IDEs and
doxygen tools.

### Default PDF file

A default PDF doc file `refman.pdf` is placed at docs directory. Directly open it to read all docs of this project. Note
that this file is automatically generated by programs, so, duplicate and useless information may be included in it.

### Web pages and custom-builds

A default doxygen config file is provided in directory docs. Here are steps to build it:

1. Predict `doxywizard` and import this config file.
2. Select the root directory of project in "Step 1" of doxygen GUI window.
3. Custom your config, turn to "Predict" page and click "Predict doxygen" button.
4. All source files will be generated in a minute. Now HTML files are ready to use.
5. (Optional) Open terminal at `docs/latex` and run `make -jn` (replace `n` with your cpu core number) to build PDF
   file. You must install [TeXLive](https://www.tug.org/texlive/) (or something else to run `tex`) in your system first
   to use it.
