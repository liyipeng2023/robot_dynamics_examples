# Robot Dynamics Library (RDL)

RDL is an advanced, high-performance C++ library designed for rigid body kinematics and dynamics in robotics. Derived and extended from classical foundations, RDL faithfully implements the algorithms described in Featherstone's seminal works, supporting comprehensive joint types, loop closure constraints, and custom-tailored numerical solvers for inverse kinematics, kinematic parameters calibration, and dynamic parameters calibration.

---

## Key Features

* **Inverse and Forward Dynamics**: Implements the forward and inverse dynamics algorithms compatible with all joint types described in Featherstone's book.
* **Loop Closure Constraints**: Support for solving dynamics with kinematic loop constraints.
* **Inverse Kinematics**: Includes numerical solutions for inverse kinematics.
* **Parameters Calibration Suite**: Built-in support for Modified Denavit-Hartenberg (MDH) parameters calibration and rigorous dynamics parameters identification.
* **Multi-Platform Support**: Cross-platform support for Linux (x86_64, ARM64) and Windows (x86_64, ARM64) with automated package detection and runtime dependency deployment.
* **Frictionless Integration**: C-like interface is adopted and strict ABI stability is ensured. RDL can be deployed easily for real-time control of robots on edge hardware like Raspberry Pi, RK3588, etc. 

---

## What's next

* **Handbooks of SDK**: ...
* **More Examples**: ...
* **Contact Dynamics Solvers**: ...

## Data Visualization Tool (`plot_data_GUI.py`)

To assist with experiment evaluation, this repository includes an interactive GUI plotting utility (`plot_data_GUI.py`) created with Gemini. It simplifies rendering high-quality figures directly from exported CSV/TXT test logs.

### Key Highlights:
* **Effortless LaTeX & Labeling**: Full LaTeX mathematical syntax support (`\theta`, `\dot{q}`, etc.) for custom legends and axis labels with live math font switching (`stix`, `cm`, etc.).
* **Fine-Grained Style Control**: Easily adjust line styles, markers, colors, line widths, and fonts per curve or category through an intuitive configurator.
* **Advanced Features**: Inset zoom window for detailed sub-plots and independent Y-axis limit/label mapping.
* **Publication-Ready Export**: One-click batch export to 300 DPI PNGs or lossless vector PDFs.

---

## Project Directory Structure

Ensure your project root is organized in the following structure to support cross-platform builds and third-party library loading:

```text
your_project/
├── CMakeLists.txt
├── plot_data_GUI.py            # Gemini-generated IEEE plotting tool
├── 3rdparties/
│   ├── rdl-linux-arm64/        # Linux ARM64 binaries and cmake configs
│   ├── rdl-linux-x86_64/       # Linux x86_64 binaries and cmake configs
│   ├── rdl-windows-arm64/      # Windows ARM64 binaries, headers, and DLLs
|   └── rdl-windows-x86_64/     # Windows x86_64 binaries, headers, and DLLs
├── examples/
│   ├── utils/
│   ├── test_3DOF_S_dynamics.cpp
│   ├── test_6DOF_S_dynamics.cpp
│   ├── test_dual_floating_robots
|   ├── test_floating_base.cpp
│   └── ...
└── ...
```

---

## Prerequisites

* **C++ Standard**: C++11 or higher
* **Build System**: CMake 3.10 or higher (CMake 3.21+ recommended)
* **Dependencies**: 
  * **Eigen3** (Linear algebra library required by examples, but it is optional for the use of RDL)
  * Pre-compiled **RDL** binaries matching your target platform (`rdl-linux-x86_64`, `rdl-linux-arm64`, or `rdl-windows-x86_64`, `rdl-windows-arm64`)
  * **Python 3.x** with `matplotlib`, `pandas`, `numpy`, and `tkinter` (optional, for running `plot_data_GUI.py`)

---

## Building and Configuration (`CMakeLists.txt`)

The project provides a clean and modern CMake script that automatically detects the target platform, configures the appropriate package paths (`rdl_DIR`), links against **Eigen3** and **RDL**, and automatically deploys runtime DLLs for Windows environments.

Here is the standard configuration pattern used in the root `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.10)
project(RDL_Examples)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Automatically detect OS and Architecture for RDL package path
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(RDL_PLATFORM_NAME "linux-arm64")
    else()
        set(RDL_PLATFORM_NAME "linux-x86_64")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(RDL_PLATFORM_NAME "windows-x86_64")
else()
    message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}")
endif()

# Point to the corresponding rdl directory based on the platform
set(rdl_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparties/rdl-${RDL_PLATFORM_NAME}/lib/cmake/rdl")

# 2. Find required packages
find_package(rdl REQUIRED)
find_package(Eigen3 REQUIRED)

# 3. Automatically glob and configure all test programs under 'examples/'
file(GLOB EXAMPLE_SOURCES "${CMAKE_SOURCE_DIR}/examples/*.cpp")

# 4. add test_floating_base
add_executable(test_floating_base ${CMAKE_SOURCE_DIR}/examples/test_floating_base.cpp)
target_include_directories(test_floating_base PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_floating_base PRIVATE rdl::rdl_core)
# 5. Ensure the rdl DLL is available next to the executable after build
if(WIN32)
    set(TEST_TARGETS test_floating_base)
    foreach(test_target ${TEST_TARGETS})
        add_custom_command(
                TARGET ${test_target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:${test_target}>
                $<TARGET_FILE_DIR:${test_target}>
                COMMAND_EXPAND_LISTS
                COMMENT "Copying runtime DLLs for ${test_target}..."
        )
    endforeach()
elseif(UNIX AND NOT APPLE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/rdl/rdl-${RDL_PLATFORM_NAME}/lib")
endif()
```

---

## Quick Start

1. Clone or copy your project files into the workspace, ensuring the `rdl/` folder contains the correct platform package directories.
   
2. Using 'add_executable' to add your files in CMakeLists.txt 
   
3. Create a build directory and run CMake:
   ```bash
   mkdir build && cd build
   cmake ..
   ```
4. Build the project:
   ```bash
   cmake --build . --config Release
   ```
5. Executables will be compiled into the build directory, with Windows DLLs automatically copied alongside them.

6. On macOS, please excute 'xattr -d com.apple.quarantine rdl/rdl-macos-arm64/lib/*.dylib' before running the excutables.