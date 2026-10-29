# Robot Dynamics Library (RDL)

RDL is an advanced, high-performance C++ library designed for rigid body kinematics and dynamics in robotics. Derived and extended from classical foundations, RDL faithfully implements the algorithms described in Featherstone's seminal works, supporting comprehensive joint types, loop closure constraints, and custom-tailored numerical solvers for kinematics and calibration.

---

## ✨ Key Features

* **Complete Featherstone Dynamics**: Implements robust forward and inverse dynamics algorithms compatible with all joint types described in Featherstone's literature.
* **Loop Closure Constraints**: Advanced mechanics support for solving dynamics systems with kinematic loop constraints.
* **Custom Solvers & Kinematics**: Includes the author's proprietary numerical solutions for inverse kinematics.
* **Parameter Calibration Suite**: Built-in support for Modified Denavit-Hartenberg (MDH) parameter calibration and rigorous dynamics parameter identification.
* **Multi-Platform & Modern CMake**: Seamless cross-platform support for Linux (x86_64, ARM64) and Windows (x86_64, ARM64) with automated package detection and runtime dependency deployment.

---

## 📊 Data Visualization Tool (`plot_data_GUI.py`)

To assist with experiment evaluation and paper writing, this repository includes an interactive GUI plotting utility (`plot_data_GUI.py`) created with Gemini. It simplifies rendering high-quality IEEE-compliant figures directly from exported CSV/TXT test logs.

### Key Highlights:
* **Effortless LaTeX & Labeling**: Full LaTeX mathematical syntax support (`\theta`, `\dot{q}`, etc.) for custom legends and axis labels with live math font switching (`stix`, `cm`, etc.).
* **Fine-Grained Style Control**: Easily adjust linestyles, markers, colors, line widths, and fonts per curve or category through an intuitive configurator.
* **Advanced Features**: Inset zoom window for detailed sub-plots and independent Y-axis limit/label mapping.
* **Publication-Ready Export**: One-click batch export to 300 DPI PNGs or lossless vector PDFs.

---

## 📂 Project Directory Structure

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
│   ├── test_dual_floating_robots.cpp
│   └── test_floating_base.cpp
└── ...
```

---

## ⚙️ Prerequisites

* **C++ Standard**: C++11 or higher
* **Build System**: CMake 3.10 or higher (CMake 3.21+ recommended)
* **Dependencies**: 
  * **Eigen3** (Linear algebra library required by RDL)
  * Pre-compiled **RDL** binaries matching your target platform (`rdl-linux-x86_64`, `rdl-linux-arm64`, or `rdl-windows-x86_64`, `rdl-windows-arm64`)
  * **Python 3.x** with `matplotlib`, `pandas`, `numpy`, and `tkinter` (optional, for running `plot_data_GUI.py`)

---

## 🔨 Building and Configuration (`CMakeLists.txt`)

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

foreach(src_file ${EXAMPLE_SOURCES})
    # Extract filename without extension as target name (e.g., test_dual_floating_robots)
    get_filename_component(target_name ${src_file} NAME_WE)

    add_executable(${target_name} ${src_file})

    # Include directories
    target_include_directories(${target_name} PRIVATE
        ${EIGEN3_INCLUDE_DIR}
    )

    # Link RDL core library
    target_link_libraries(${target_name} PRIVATE rdl::rdl_core)

    # 4. Post-build step: Automatically copy DLLs to target directory on Windows
    if(WIN32)
        add_custom_command(
            TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparties/rdl-${RDL_PLATFORM_NAME}/bin"
                    "$<TARGET_FILE_DIR:${target_name}>"
            COMMENT "Copying RDL runtime DLLs for ${target_name}..."
        )
    endif()
endforeach()
```

---

## 🚀 Quick Start

1. Clone or copy your project files into the workspace, ensuring the `3rdparties/` folder contains the correct platform package directories.
2. Create a build directory and run CMake:
   ```bash
   mkdir build && cd build
   cmake ..
   ```
3. Build the project:
   ```bash
   cmake --build . --config Release
   ```
4. Executables will be compiled into the build directory, with Windows DLLs automatically copied alongside them.