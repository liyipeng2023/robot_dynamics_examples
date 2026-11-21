#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RDL Linux Control Software Project Generator
--------------------------------------------
Generates a standalone, modular Linux C++17 robot control project featuring:
  1. Pure POSIX Raw TCP / Raw UDP communication channels.
  2. Optional ROS communication bridge built over UDP/TCP (zero ROS C++ build dependency)
     plus a companion ROS1/ROS2 Python gateway script.
  3. Virtual actuator/hardware abstraction interfaces (IJointActuator / IRobotHardware)
     with MuJoCo lockstep simulation implemented as a derived driver class.
  4. Clean include/src layout, MuJoCo test executable, and plug-and-play CMakeLists.txt
     configured with:
       set(rdl_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rdl/${RDL_PLATFORM_FOLDER}/lib/cmake/rdl")
"""

import os
import re
import stat
import shutil
import argparse
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any


DEFAULT_DEMO_ROBOT_XML = """<mujoco model="demo_robot">
    <worldbody>
        <body name="pedestal" pos="0 0 0">
            <geom name="pedestal_geom" type="cylinder" fromto="0 0 0 0 0 0.5" size="0.07" rgba="0.35 0.35 0.4 1" mass="10"/>
            <body name="link1" pos="0 0 0.5">
                <joint name="joint1" type="hinge" axis="0 1 0" damping="0.0"/>
                <geom name="link1_geom" type="capsule" fromto="0 0 0 0.35 0 0" size="0.04" rgba="0.2 0.5 0.85 1" mass="1.5"/>
                <body name="link2" pos="0.35 0 0">
                    <joint name="joint2" type="hinge" axis="0 1 0" damping="0.0"/>
                    <geom name="link2_geom" type="capsule" fromto="0 0 0 0.3 0 0" size="0.032" rgba="0.9 0.45 0.2 1" mass="1.0"/>
                </body>
            </body>
        </body>
    </worldbody>
</mujoco>
"""


@dataclass
class ProjectGenConfig:
    """Configuration passed from CLI or MuJoCo GUI to generate the Linux C++ control project."""
    output_dir: str
    project_name: str = "rdl_robot_control"
    model_file: Optional[str] = None
    is_floating_base: bool = False
    nq: int = 2
    nv: int = 2
    joint_names: List[str] = field(default_factory=list)
    rotor_inertia: List[float] = field(default_factory=list)
    sim_timestep: float = 0.001

    # MuJoCo ZMQ endpoints (from C++ controller's perspective: SUB connects to sim PUB, PUB connects to sim SUB)
    mujoco_sub_endpoint: str = "tcp://127.0.0.1:5555"
    mujoco_pub_endpoint: str = "tcp://127.0.0.1:5556"

    # RDL platform folder name under <project_root>/rdl/${RDL_PLATFORM_FOLDER}/lib/cmake/rdl
    rdl_platform_folder: str = "linux-x86_64"

    # Optional ROS communication over UDP/TCP
    enable_ros_comm: bool = False
    ros_transport: str = "udp"          # "udp" or "tcp"
    ros_remote_ip: str = "127.0.0.1"
    ros_state_port: int = 9001          # Controller -> ROS gateway
    ros_cmd_port: int = 9002            # ROS gateway -> Controller

    # Default controller gains & test trajectory parameters
    kp: float = 400.0
    ki: float = 20.0
    kd: float = 40.0
    traj_amp: float = 0.40
    traj_freq: float = 0.25


def sanitize_cmake_name(name: str) -> str:
    """Sanitizes a project name into a valid CMake / C++ identifier."""
    cleaned = re.sub(r"[^a-zA-Z0-9_]", "_", name.strip())
    if not cleaned or cleaned[0].isdigit():
        cleaned = "rdl_" + cleaned
    return cleaned


def convert_bind_to_connect_endpoint(endpoint: str, default_host: str = "127.0.0.1") -> str:
    """Converts MuJoCo bind endpoints like 'tcp://*:5555' to client connect endpoints 'tcp://127.0.0.1:5555'."""
    ep = (endpoint or "").strip()
    if not ep:
        return f"tcp://{default_host}:5555"
    return ep.replace("://*:", f"://{default_host}:").replace("://0.0.0.0:", f"://{default_host}:")


def render_template(template: str, tokens: Dict[str, str]) -> str:
    """Replaces @@TOKEN@@ placeholders without interfering with C++ braces or CMake ${VAR}."""
    out = template
    for key, val in tokens.items():
        out = out.replace(f"@@{key}@@", str(val))
    return out


class ControlProjectGenerator:
    """Generates a complete Linux C++17 RDL control software project."""

    def __init__(self, config: ProjectGenConfig):
        self.cfg = config
        self.cfg.project_name = sanitize_cmake_name(config.project_name)
        self.cfg.ros_transport = config.ros_transport.lower().strip()
        if self.cfg.ros_transport not in ("udp", "tcp"):
            self.cfg.ros_transport = "udp"
        self.generated_files: List[str] = []

    def generate(self) -> Dict[str, Any]:
        out_root = os.path.abspath(self.cfg.output_dir)
        os.makedirs(out_root, exist_ok=True)

        # 1. Create standard directory tree
        subdirs = [
            "config",
            "models",
            "rdl",
            "include/common",
            "include/comm",
            "include/hal",
            "include/controller",
            "src/comm",
            "src/hal",
            "src/controller",
        ]
        if self.cfg.enable_ros_comm:
            subdirs.append("scripts")

        for sd in subdirs:
            os.makedirs(os.path.join(out_root, sd), exist_ok=True)

        # 2. Copy or generate robot model file & rotor_inertia.txt
        model_rel_path = self._prepare_model_file(out_root)
        self._write_rotor_inertia_txt(out_root)

        # 3. Prepare template replacement tokens
        tokens = self._build_tokens(model_rel_path)

        # 4. Generate all C++ headers, sources, CMakeLists, and helper files
        files_to_write: Dict[str, str] = {
            "CMakeLists.txt": self._tmpl_cmakelists(),
            "README.md": self._tmpl_readme(),
            "build.sh": self._tmpl_build_sh(),
            "rdl/README_PUT_RDL_HERE.txt": self._tmpl_rdl_placeholder(),
            "include/common/robot_msg.h": self._tmpl_robot_msg_h(),
            "include/common/robot_types.h": self._tmpl_robot_types_h(),
            "include/comm/raw_udp_channel.h": self._tmpl_raw_udp_h(),
            "src/comm/raw_udp_channel.cpp": self._tmpl_raw_udp_cpp(),
            "include/comm/raw_tcp_channel.h": self._tmpl_raw_tcp_h(),
            "src/comm/raw_tcp_channel.cpp": self._tmpl_raw_tcp_cpp(),
            "include/hal/joint_interface.h": self._tmpl_joint_interface_h(),
            "include/hal/mujoco_joint_driver.h": self._tmpl_mujoco_driver_h(),
            "src/hal/mujoco_joint_driver.cpp": self._tmpl_mujoco_driver_cpp(),
            "include/hal/hardware_joint_stub.h": self._tmpl_hardware_stub_h(),
            "src/hal/hardware_joint_stub.cpp": self._tmpl_hardware_stub_cpp(),
            "include/controller/rdl_controller.h": self._tmpl_rdl_controller_h(),
            "src/controller/rdl_controller.cpp": self._tmpl_rdl_controller_cpp(),
            "src/main_mujoco_test.cpp": self._tmpl_main_mujoco_test_cpp(),
        }

        if self.cfg.enable_ros_comm:
            files_to_write["include/comm/ros_bridge.h"] = self._tmpl_ros_bridge_h()
            files_to_write["src/comm/ros_bridge.cpp"] = self._tmpl_ros_bridge_cpp()
            files_to_write["scripts/ros_bridge_gateway.py"] = self._tmpl_ros_gateway_py()

        for rel_path, tmpl in files_to_write.items():
            full_path = os.path.join(out_root, rel_path)
            content = render_template(tmpl, tokens)
            with open(full_path, "w", encoding="utf-8", newline="\n") as f:
                f.write(content)
            self.generated_files.append(full_path)

            if rel_path.endswith(".sh") or rel_path.endswith(".py"):
                try:
                    cur_mode = os.stat(full_path).st_mode
                    os.chmod(full_path, cur_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
                except OSError:
                    pass

        return {
            "output_dir": out_root,
            "project_name": self.cfg.project_name,
            "model_rel_path": model_rel_path,
            "enable_ros_comm": self.cfg.enable_ros_comm,
            "ros_transport": self.cfg.ros_transport,
            "generated_files": self.generated_files,
        }

    def _prepare_model_file(self, out_root: str) -> str:
        models_dir = os.path.join(out_root, "models")
        if self.cfg.model_file and os.path.isfile(self.cfg.model_file):
            fname = os.path.basename(self.cfg.model_file)
            dst = os.path.join(models_dir, fname)
            if os.path.abspath(self.cfg.model_file) != os.path.abspath(dst):
                shutil.copy2(self.cfg.model_file, dst)
            self.generated_files.append(dst)
            return f"models/{fname}"

        dst = os.path.join(models_dir, "demo_robot.xml")
        with open(dst, "w", encoding="utf-8", newline="\n") as f:
            f.write(DEFAULT_DEMO_ROBOT_XML)
        self.generated_files.append(dst)
        return "models/demo_robot.xml"

    def _write_rotor_inertia_txt(self, out_root: str) -> None:
        dst = os.path.join(out_root, "config", "rotor_inertia.txt")
        vals = list(self.cfg.rotor_inertia) if self.cfg.rotor_inertia else [0.0] * max(1, self.cfg.nv)
        with open(dst, "w", encoding="utf-8", newline="\n") as f:
            f.write("# Equivalent reflected motor rotor inertia (I_a = N^2 * I_m) per DOF\n")
            for v in vals:
                f.write(f"{float(v):.8f}\n")
        self.generated_files.append(dst)

    def _build_tokens(self, model_rel_path: str) -> Dict[str, str]:
        joint_names_cpp = ", ".join(f'"{n}"' for n in self.cfg.joint_names)
        joint_names_py = ", ".join(f'"{n}"' for n in self.cfg.joint_names)
        return {
            "PROJECT_NAME": self.cfg.project_name,
            "MODEL_REL_PATH": model_rel_path,
            "IS_FLOATING_BASE_INT": "1" if self.cfg.is_floating_base else "0",
            "IS_FLOATING_BASE_BOOL": "true" if self.cfg.is_floating_base else "false",
            "DEFAULT_NQ": str(self.cfg.nq),
            "DEFAULT_NV": str(self.cfg.nv),
            "SIM_TIMESTEP": f"{self.cfg.sim_timestep:.6f}",
            "MUJOCO_SUB_EP": convert_bind_to_connect_endpoint(self.cfg.mujoco_sub_endpoint),
            "MUJOCO_PUB_EP": convert_bind_to_connect_endpoint(self.cfg.mujoco_pub_endpoint),
            "RDL_PLATFORM_FOLDER": self.cfg.rdl_platform_folder,
            "ENABLE_ROS_CMAKE": "ON" if self.cfg.enable_ros_comm else "OFF",
            "ROS_TRANSPORT_UPPER": self.cfg.ros_transport.upper(),
            "ROS_TRANSPORT_LOWER": self.cfg.ros_transport.lower(),
            "ROS_REMOTE_IP": self.cfg.ros_remote_ip,
            "ROS_STATE_PORT": str(self.cfg.ros_state_port),
            "ROS_CMD_PORT": str(self.cfg.ros_cmd_port),
            "DEFAULT_KP": f"{self.cfg.kp:.4f}",
            "DEFAULT_KI": f"{self.cfg.ki:.4f}",
            "DEFAULT_KD": f"{self.cfg.kd:.4f}",
            "DEFAULT_TRAJ_AMP": f"{self.cfg.traj_amp:.4f}",
            "DEFAULT_TRAJ_FREQ": f"{self.cfg.traj_freq:.4f}",
            "JOINT_NAMES_CPP_LIST": joint_names_cpp,
            "JOINT_NAMES_PY_LIST": joint_names_py,
        }

    # =========================================================================
    # Template Definitions
    # =========================================================================

    @staticmethod
    def _tmpl_cmakelists() -> str:
        return r"""cmake_minimum_required(VERSION 3.16)
        project(@@PROJECT_NAME@@ LANGUAGES CXX)

        if(NOT UNIX OR APPLE)
            message(WARNING "This project is specifically configured for Linux platforms.")
        endif()

        set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        set(CMAKE_CXX_EXTENSIONS OFF)

        if(NOT CMAKE_BUILD_TYPE)
            set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
        endif()

        option(ENABLE_ROS_BRIDGE "Build ROS communication bridge over UDP/TCP" @@ENABLE_ROS_CMAKE@@)

        # -----------------------------------------------------------------------------
        # 1. RDL Dependency Path (Drop-in folder under ${CMAKE_CURRENT_SOURCE_DIR}/rdl/)
        # -----------------------------------------------------------------------------
        set(RDL_PLATFORM_FOLDER "@@RDL_PLATFORM_FOLDER@@" CACHE STRING
            "Platform folder name inside rdl/ (e.g., linux-x86_64, linux-arm64)")

        # Auto-detect if the specified platform folder does not exist yet,
        # but the user copied exactly one platform directory containing lib/cmake/rdl into rdl/.
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rdl/${RDL_PLATFORM_FOLDER}/lib/cmake/rdl")
            file(GLOB _RDL_CMAKE_CANDIDATES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/rdl"
                "${CMAKE_CURRENT_SOURCE_DIR}/rdl/*/lib/cmake/rdl")
            list(LENGTH _RDL_CMAKE_CANDIDATES _RDL_CAND_COUNT)
            if(_RDL_CAND_COUNT EQUAL 1)
                list(GET _RDL_CMAKE_CANDIDATES 0 _CAND_REL)
                get_filename_component(_CAND_DIR "${_CAND_REL}" DIRECTORY) # <plat>/lib/cmake
                get_filename_component(_CAND_DIR "${_CAND_DIR}" DIRECTORY) # <plat>/lib
                get_filename_component(_DETECTED_PLAT "${_CAND_DIR}" DIRECTORY)
                message(STATUS "[RDL] Auto-detected platform folder: ${_DETECTED_PLAT}")
                set(RDL_PLATFORM_FOLDER "${_DETECTED_PLAT}" CACHE STRING "" FORCE)
            endif()
        endif()

        set(rdl_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rdl/${RDL_PLATFORM_FOLDER}/lib/cmake/rdl")
        set(RDL_PLATFORM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/rdl/${RDL_PLATFORM_FOLDER}")
        message(STATUS "[RDL] Using rdl_DIR = ${rdl_DIR}")

        if(NOT EXISTS "${rdl_DIR}")
            message(FATAL_ERROR
                "\n[RDL Missing] Directory not found:\n  ${rdl_DIR}\n"
                "Please copy your Linux RDL SDK folder into:\n"
                "  ${CMAKE_CURRENT_SOURCE_DIR}/rdl/${RDL_PLATFORM_FOLDER}/\n"
                "Or pass -DRDL_PLATFORM_FOLDER=<your_folder_name> to cmake.\n")
        endif()

        find_package(rdl REQUIRED)

        # -----------------------------------------------------------------------------
        # 2. System Dependencies: Threads, Eigen3, libzmq (C API, no cppzmq required)
        # -----------------------------------------------------------------------------
        find_package(Threads REQUIRED)

        find_package(Eigen3 3.3 QUIET NO_MODULE)
        if(NOT Eigen3_FOUND)
            find_package(PkgConfig QUIET)
            if(PkgConfig_FOUND)
                pkg_check_modules(EIGEN3_PKG QUIET eigen3)
            endif()
            find_path(EIGEN3_FALLBACK_INCLUDE_DIR NAMES Eigen/Dense
                    PATHS ${EIGEN3_PKG_INCLUDE_DIRS} /usr/include/eigen3 /usr/local/include/eigen3)
            if(NOT EIGEN3_FALLBACK_INCLUDE_DIR)
                message(FATAL_ERROR "Eigen3 not found! Install via: sudo apt install libeigen3-dev")
            endif()
            add_library(Eigen3::Eigen INTERFACE IMPORTED)
            set_target_properties(Eigen3::Eigen PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_FALLBACK_INCLUDE_DIR}")
        endif()

        find_package(PkgConfig QUIET)
        if(PkgConfig_FOUND)
            pkg_check_modules(LIBZMQ QUIET libzmq)
        endif()
        if(NOT LIBZMQ_FOUND)
            find_path(LIBZMQ_INCLUDE_DIRS NAMES zmq.h PATHS /usr/include /usr/local/include)
            find_library(LIBZMQ_LIBRARIES NAMES zmq PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu)
            if(NOT LIBZMQ_INCLUDE_DIRS OR NOT LIBZMQ_LIBRARIES)
                message(FATAL_ERROR "libzmq not found! Install via: sudo apt install libzmq3-dev")
            endif()
        endif()

        # -----------------------------------------------------------------------------
        # 3. Core Control Library (comm + hal + controller)
        # -----------------------------------------------------------------------------
        set(CORE_SOURCES
            src/comm/raw_udp_channel.cpp
            src/comm/raw_tcp_channel.cpp
            src/hal/mujoco_joint_driver.cpp
            src/hal/hardware_joint_stub.cpp
            src/controller/rdl_controller.cpp
        )

        if(ENABLE_ROS_BRIDGE)
            list(APPEND CORE_SOURCES src/comm/ros_bridge.cpp)
        endif()

        add_library(${PROJECT_NAME}_core STATIC ${CORE_SOURCES})

        target_include_directories(${PROJECT_NAME}_core PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${LIBZMQ_INCLUDE_DIRS}
        )

        # Add RDL include directories cleanly (covers both standard include/ and utils/include/)
        if(EXISTS "${RDL_PLATFORM_ROOT}/include")
            target_include_directories(${PROJECT_NAME}_core PUBLIC "${RDL_PLATFORM_ROOT}/include")
        endif()
        if(EXISTS "${RDL_PLATFORM_ROOT}/include/utils")
            target_include_directories(${PROJECT_NAME}_core PUBLIC "${RDL_PLATFORM_ROOT}/include/utils")
        endif()

        # Resolve exported RDL CMake targets gracefully
        set(_RDL_LINK_TARGETS "")
        foreach(_cand rdl::rdl rdl::rdl_utils rdl::utils rdl)
            if(TARGET ${_cand})
                list(APPEND _RDL_LINK_TARGETS ${_cand})
            endif()
        endforeach()
        if(_RDL_LINK_TARGETS STREQUAL "" AND DEFINED rdl_LIBRARIES)
            set(_RDL_LINK_TARGETS ${rdl_LIBRARIES})
        endif()
        if(_RDL_LINK_TARGETS STREQUAL "" AND DEFINED RDL_LIBRARIES)
            set(_RDL_LINK_TARGETS ${RDL_LIBRARIES})
        endif()
        if(_RDL_LINK_TARGETS STREQUAL "")
            # Final fallback: link shared/static libs found inside rdl/<platform>/lib
            file(GLOB _RDL_RAW_LIBS "${RDL_PLATFORM_ROOT}/lib/librdl*.so*" "${RDL_PLATFORM_ROOT}/lib/librdl*.a")
            set(_RDL_LINK_TARGETS ${_RDL_RAW_LIBS})
        endif()
        message(STATUS "[RDL] Link targets: ${_RDL_LINK_TARGETS}")

        target_link_libraries(${PROJECT_NAME}_core PUBLIC
            ${_RDL_LINK_TARGETS}
            Eigen3::Eigen
            ${LIBZMQ_LIBRARIES}
            Threads::Threads
        )

        if(ENABLE_ROS_BRIDGE)
            target_compile_definitions(${PROJECT_NAME}_core PUBLIC ENABLE_ROS_BRIDGE=1)
        endif()

        target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wno-unused-parameter)

        # -----------------------------------------------------------------------------
        # 4. MuJoCo Test Executable
        # -----------------------------------------------------------------------------
        add_executable(mujoco_test_app src/main_mujoco_test.cpp)
        target_link_libraries(mujoco_test_app PRIVATE ${PROJECT_NAME}_core)

        # Embed RPATH so lib/ shared libraries under rdl/ are found automatically at runtime
        set_target_properties(mujoco_test_app PROPERTIES
            BUILD_RPATH "${RDL_PLATFORM_ROOT}/lib"
            INSTALL_RPATH "$ORIGIN/../rdl/${RDL_PLATFORM_FOLDER}/lib"
        )
        """

    @staticmethod
    def _tmpl_build_sh() -> str:
        return r"""#!/usr/bin/env bash
        set -euo pipefail

        SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        cd "${SCRIPT_DIR}"

        PLATFORM_ARG=""
        if [[ $# -ge 1 ]]; then
            PLATFORM_ARG="-DRDL_PLATFORM_FOLDER=$1"
        fi

        echo "=== Configuring @@PROJECT_NAME@@ ==="
        cmake -S . -B build -DCMAKE_BUILD_TYPE=Release ${PLATFORM_ARG}

        echo "=== Building @@PROJECT_NAME@@ ==="
        cmake --build build -j"$(nproc)"

        echo ""
        echo "=== Build Complete ==="
        echo "Run MuJoCo test controller:"
        echo "  ./build/mujoco_test_app"
        """

    @staticmethod
    def _tmpl_rdl_placeholder() -> str:
        return r"""Place your compiled Linux RDL SDK folder here.

        Expected directory layout:
        rdl/
        └── @@RDL_PLATFORM_FOLDER@@/
            ├── include/
            │   ├── graph.h
            │   ├── robot_dynamics.h
            │   └── robot_loader.h (or utils/robot_loader.h)
            └── lib/
                ├── cmake/
                │   └── rdl/
                │       └── rdlConfig.cmake
                └── librdl*.so / librdl*.a

        CMake automatically sets:
        set(rdl_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rdl/${RDL_PLATFORM_FOLDER}/lib/cmake/rdl")

        If your platform folder has a different name (for example linux-arm64), either:
        1. Copy only that single folder into rdl/ (CMake will auto-detect it), or
        2. Pass: cmake -S . -B build -DRDL_PLATFORM_FOLDER=<your_folder_name>
        """

    @staticmethod
    def _tmpl_readme() -> str:
        return r"""# @@PROJECT_NAME@@

        Auto-generated Linux C++17 RDL robot control software project.

        ## 1. Prerequisites (Ubuntu / Debian / Raspberry Pi OS)

        ```bash
        sudo apt update
        sudo apt install -y build-essential cmake pkg-config libeigen3-dev libzmq3-dev
        2. Copy RDL Library
        Copy your prebuilt Linux RDL library folder into rdl/:

        Plaintext
        rdl/@@RDL_PLATFORM_FOLDER@@/lib/cmake/rdl/rdlConfig.cmake
        3. Build
        Bash
        ./build.sh
        # Or specify custom platform folder name:
        ./build.sh linux-arm64
        4. Run with MuJoCo Simulator (robot_sim.py)
        Launch robot_sim.py and load your robot model.

        Run the generated controller directly from the project root:

        Bash
        ./build/mujoco_test_app
        Optional CLI overrides:

        Bash
        ./build/mujoco_test_app --model @@MODEL_REL_PATH@@ --floating @@IS_FLOATING_BASE_INT@@ \
            --kp @@DEFAULT_KP@@ --ki @@DEFAULT_KI@@ --kd @@DEFAULT_KD@@ \
            --amp @@DEFAULT_TRAJ_AMP@@ --freq @@DEFAULT_TRAJ_FREQ@@ \
            --inertia config/rotor_inertia.txt
        5. Communication & Hardware Abstraction
        Raw UDP / TCP: include/comm/raw_udp_channel.h, include/comm/raw_tcp_channel.h

        Virtual Joint / Hardware Interface: include/hal/joint_interface.h

        MujocoJointDriver (include/hal/mujoco_joint_driver.h) implements IRobotHardware via ZMQ lockstep.

        HardwareJointStub (include/hal/hardware_joint_stub.h) provides a clean starting point for real motors (EtherCAT/CAN).

        ROS Bridge (Optional, ENABLE_ROS_BRIDGE=@@ENABLE_ROS_CMAKE@@):

        Transport: @@ROS_TRANSPORT_UPPER@@ (State port @@ROS_STATE_PORT@@, Command port @@ROS_CMD_PORT@@).

        Run python3 scripts/ros_bridge_gateway.py in a ROS1 or ROS2 environment to expose /rdl/joint_states and /rdl/joint_commands.
        """

    @staticmethod
    def _tmpl_robot_msg_h() -> str:
        return r"""#pragma once

        #include 
        #include 
        #include

        namespace rdl_ctrl {

        constexpr uint32_t kMagicRdlm = 0x52444C4D; // "RDLM"

        #pragma pack(push, 1)
        struct SimStateHeader {
        uint32_t magic;      // 0x52444C4D ("RDLM")
        uint32_t nq;
        uint32_t nv;
        uint32_t ncon;
        double   timestamp;
        };

        struct ContactItem {
        int32_t geom1;
        int32_t geom2;
        double  dist;            // Penetration depth (m)
        double  pos[3];          // Contact world position
        double  normal[3];       // Contact normal vector
        double  force_local[6];  // Contact frame 6D wrench [fn, ft1, ft2, tx, ty, tz]
        double  force_world[3];  // World frame 3D contact force
        };

        struct RdlCmdHeader {
        uint32_t magic;      // 0x52444C4D
        uint32_t mode;       // 0: Torque control mode, 1: Direct state overwrite mode
        uint32_t nq;
        uint32_t nv;
        };
        #pragma pack(pop)

        inline bool parseSimStateZeroCopy(const void* raw_data,
        size_t raw_size,
        const SimStateHeader*& header_out,
        const double*& qpos_ptr,
        const double*& qvel_ptr,
        const double*& qfrc_constraint_ptr,
        const ContactItem*& contacts_ptr) {
        if (!raw_data || raw_size < sizeof(SimStateHeader)) {
        return false;
        }

        const auto* raw = static_cast<const uint8_t*>(raw_data);
        header_out = reinterpret_cast<const SimStateHeader*>(raw);
        if (header_out->magic != kMagicRdlm) {
            return false;
        }

        const size_t qpos_bytes = static_cast<size_t>(header_out->nq) * sizeof(double);
        const size_t qvel_bytes = static_cast<size_t>(header_out->nv) * sizeof(double);
        const size_t qfrc_bytes = static_cast<size_t>(header_out->nv) * sizeof(double);
        const size_t con_bytes  = static_cast<size_t>(header_out->ncon) * sizeof(ContactItem);
        const size_t required   = sizeof(SimStateHeader) + qpos_bytes + qvel_bytes + qfrc_bytes + con_bytes;

        if (raw_size < required) {
            return false;
        }

        size_t offset = sizeof(SimStateHeader);
        qpos_ptr = reinterpret_cast<const double*>(raw + offset);
        offset += qpos_bytes;

        qvel_ptr = reinterpret_cast<const double*>(raw + offset);
        offset += qvel_bytes;

        qfrc_constraint_ptr = reinterpret_cast<const double*>(raw + offset);
        offset += qfrc_bytes;

        contacts_ptr = reinterpret_cast<const ContactItem*>(raw + offset);
        return true;
        }

        } // namespace rdl_ctrl
        """

    @staticmethod
    def _tmpl_robot_types_h() -> str:
        return r"""#pragma once
        #include <Eigen/Dense>
        #include 
        #include 
        #include

        namespace rdl_ctrl {

        enum class ActuatorMode : uint8_t {
        DISABLED      = 0,
        TORQUE        = 1,
        POSITION      = 2,
        VELOCITY      = 3,
        MIT_IMPEDANCE = 4,
        DIRECT_STATE  = 5
        };

        struct ContactInfo {
        int32_t geom1 = -1;
        int32_t geom2 = -1;
        double dist = 0.0;
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        Eigen::Matrix<double, 6, 1> force_local = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Vector3d force_world = Eigen::Vector3d::Zero();
        };

        struct RobotState {
        double timestamp = 0.0;
        Eigen::VectorXd q;
        Eigen::VectorXd v;
        Eigen::VectorXd tau_meas;
        Eigen::VectorXd qfrc_constraint;
        std::vector contacts;
        bool valid = false;

        void resize(int nq, int nv) {
            q.setZero(nq);
            v.setZero(nv);
            tau_meas.setZero(nv);
            qfrc_constraint.setZero(nv);
            contacts.clear();
            valid = false;
        }
        };

        struct RobotCommand {
        ActuatorMode mode = ActuatorMode::TORQUE;
        Eigen::VectorXd q_des;
        Eigen::VectorXd v_des;
        Eigen::VectorXd tau_cmd;
        Eigen::VectorXd kp;
        Eigen::VectorXd kd;

        void resize(int nq, int nv) {
            q_des.setZero(nq);
            v_des.setZero(nv);
            tau_cmd.setZero(nv);
            kp.setZero(nv);
            kd.setZero(nv);
        }
        };

        } // namespace rdl_ctrl
        """

    @staticmethod
    def _tmpl_raw_udp_h() -> str:
        return r"""#pragma once
        #include 
        #include 
        #include 
        #include <sys/types.h>
        #include <netinet/in.h>

        namespace rdl_ctrl::comm {

        class RawUdpChannel {
        public:
        RawUdpChannel() = default;
        ~RawUdpChannel();

        RawUdpChannel(const RawUdpChannel&) = delete;
        RawUdpChannel& operator=(const RawUdpChannel&) = delete;

        bool open(uint16_t local_port = 0, const std::string& bind_ip = "0.0.0.0", bool reuse_addr = true);
        bool setDefaultPeer(const std::string& remote_ip, uint16_t remote_port);

        ssize_t send(const void* data, size_t len) const;
        ssize_t sendTo(const void* data, size_t len, const std::string& remote_ip, uint16_t remote_port) const;
        ssize_t recv(void* buffer,
                    size_t max_len,
                    int timeout_ms = 0,
                    std::string* src_ip = nullptr,
                    uint16_t* src_port = nullptr) const;

        void close();
        bool isOpen() const { return fd_ >= 0; }
        int fd() const { return fd_; }
        private:
        int fd_ = -1;
        sockaddr_in default_peer_{};
        bool has_default_peer_ = false;
        };

        } // namespace rdl_ctrl::comm
        """

    @staticmethod
    def _tmpl_raw_udp_cpp() -> str:
        return r"""#include "comm/raw_udp_channel.h"
        #include <arpa/inet.h>
        #include 
        #include 
        #include <fcntl.h>
        #include <poll.h>
        #include <sys/socket.h>
        #include <unistd.h>

        namespace rdl_ctrl::comm {

        RawUdpChannel::~RawUdpChannel() {
        close();
        }

        bool RawUdpChannel::open(uint16_t local_port, const std::string& bind_ip, bool reuse_addr) {
        close();

        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            return false;
        }

        if (reuse_addr) {
            int opt = 1;
            ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        }

        int flags = ::fcntl(fd_, F_GETFL, 0);
        if (flags >= 0) {
            ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
        }

        if (local_port > 0 || bind_ip != "0.0.0.0") {
            sockaddr_in local_addr{};
            local_addr.sin_family = AF_INET;
            local_addr.sin_port = htons(local_port);
            if (::inet_pton(AF_INET, bind_ip.c_str(), &local_addr.sin_addr) <= 0) {
                close();
                return false;
            }
            if (::bind(fd_, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) < 0) {
                close();
                return false;
            }
        }

        return true;
        }

        bool RawUdpChannel::setDefaultPeer(const std::string& remote_ip, uint16_t remote_port) {
        std::memset(&default_peer_, 0, sizeof(default_peer_));
        default_peer_.sin_family = AF_INET;
        default_peer_.sin_port = htons(remote_port);
        if (::inet_pton(AF_INET, remote_ip.c_str(), &default_peer_.sin_addr) <= 0) {
        has_default_peer_ = false;
        return false;
        }
        has_default_peer_ = true;
        return true;
        }

        ssize_t RawUdpChannel::send(const void* data, size_t len) const {
        if (fd_ < 0 || !has_default_peer_ || !data || len == 0) {
        return -1;
        }
        return ::sendto(fd_,
        data,
        len,
        MSG_NOSIGNAL,
        reinterpret_cast<const sockaddr*>(&default_peer_),
        sizeof(default_peer_));
        }

        ssize_t RawUdpChannel::sendTo(const void* data, size_t len, const std::string& remote_ip, uint16_t remote_port) const {
        if (fd_ < 0 || !data || len == 0) {
        return -1;
        }
        sockaddr_in peer{};
        peer.sin_family = AF_INET;
        peer.sin_port = htons(remote_port);
        if (::inet_pton(AF_INET, remote_ip.c_str(), &peer.sin_addr) <= 0) {
        return -1;
        }
        return ::sendto(fd_, data, len, MSG_NOSIGNAL, reinterpret_cast<const sockaddr*>(&peer), sizeof(peer));
        }

        ssize_t RawUdpChannel::recv(void* buffer,
        size_t max_len,
        int timeout_ms,
        std::string* src_ip,
        uint16_t* src_port) const {
        if (fd_ < 0 || !buffer || max_len == 0) {
        return -1;
        }

        if (timeout_ms != 0) {
            pollfd pfd{};
            pfd.fd = fd_;
            pfd.events = POLLIN;
            int ret = ::poll(&pfd, 1, timeout_ms);
            if (ret <= 0 || !(pfd.revents & POLLIN)) {
                return 0;
            }
        }

        sockaddr_in sender{};
        socklen_t sender_len = sizeof(sender);
        ssize_t n = ::recvfrom(fd_, buffer, max_len, 0, reinterpret_cast<sockaddr*>(&sender), &sender_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;
            }
            return -1;
        }

        if (src_ip) {
            char ip_buf[INET_ADDRSTRLEN] = {0};
            if (::inet_ntop(AF_INET, &sender.sin_addr, ip_buf, sizeof(ip_buf))) {
                *src_ip = ip_buf;
            }
        }
        if (src_port) {
            *src_port = ntohs(sender.sin_port);
        }
        return n;
        }

        void RawUdpChannel::close() {
        if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        }
        }

        } // namespace rdl_ctrl::comm
        """

    @staticmethod
    def _tmpl_raw_tcp_h() -> str:
        return r"""#pragma once
        #include 
        #include 
        #include 
        #include <sys/types.h>

        namespace rdl_ctrl::comm {

        class RawTcpChannel {
        public:
        RawTcpChannel() = default;
        ~RawTcpChannel();

        RawTcpChannel(const RawTcpChannel&) = delete;
        RawTcpChannel& operator=(const RawTcpChannel&) = delete;

        // Client mode
        bool connectTo(const std::string& remote_ip, uint16_t remote_port, int timeout_ms = 1000);

        // Server mode
        bool listenOn(uint16_t local_port, const std::string& bind_ip = "0.0.0.0", int backlog = 4);
        bool acceptClient(int timeout_ms = 0, std::string* client_ip = nullptr);

        // Data transfer
        bool sendAll(const void* data, size_t len);
        ssize_t recv(void* buffer, size_t max_len, int timeout_ms = 0);
        bool recvExact(void* buffer, size_t len, int timeout_ms = 50);

        void closeClient();
        void closeAll();

        bool isListening() const { return listen_fd_ >= 0; }
        bool isConnected() const { return conn_fd_ >= 0; }
        private:
        static void configureSocketLowLatency(int fd);
        int listen_fd_ = -1;
        int conn_fd_ = -1;
        };

        } // namespace rdl_ctrl::comm
        """

    @staticmethod
    def _tmpl_raw_tcp_cpp() -> str:
        return r"""#include "comm/raw_tcp_channel.h"
        #include <arpa/inet.h>
        #include 
        #include 
        #include <fcntl.h>
        #include <netinet/in.h>
        #include <netinet/tcp.h>
        #include <poll.h>
        #include <sys/socket.h>
        #include <unistd.h>

        namespace rdl_ctrl::comm {

        RawTcpChannel::~RawTcpChannel() {
        closeAll();
        }

        void RawTcpChannel::configureSocketLowLatency(int fd) {
        int nodelay = 1;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        int keepalive = 1;
        ::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
        }

        bool RawTcpChannel::connectTo(const std::string& remote_ip, uint16_t remote_port, int timeout_ms) {
        closeClient();

        int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            return false;
        }

        int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags >= 0) {
            ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(remote_port);
        if (::inet_pton(AF_INET, remote_ip.c_str(), &addr.sin_addr) <= 0) {
            ::close(fd);
            return false;
        }

        int ret = ::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        if (ret < 0) {
            if (errno != EINPROGRESS) {
                ::close(fd);
                return false;
            }
            pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLOUT;
            int pret = ::poll(&pfd, 1, timeout_ms);
            if (pret <= 0) {
                ::close(fd);
                return false;
            }
            int so_err = 0;
            socklen_t err_len = sizeof(so_err);
            if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_err, &err_len) < 0 || so_err != 0) {
                ::close(fd);
                return false;
            }
        }

        configureSocketLowLatency(fd);
        conn_fd_ = fd;
        return true;
        }

        bool RawTcpChannel::listenOn(uint16_t local_port, const std::string& bind_ip, int backlog) {
        closeAll();

        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd_ < 0) {
            return false;
        }

        int opt = 1;
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        int flags = ::fcntl(listen_fd_, F_GETFL, 0);
        if (flags >= 0) {
            ::fcntl(listen_fd_, F_SETFL, flags | O_NONBLOCK);
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(local_port);
        if (::inet_pton(AF_INET, bind_ip.c_str(), &addr.sin_addr) <= 0) {
            closeAll();
            return false;
        }

        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            closeAll();
            return false;
        }
        if (::listen(listen_fd_, backlog) < 0) {
            closeAll();
            return false;
        }
        return true;
        }

        bool RawTcpChannel::acceptClient(int timeout_ms, std::string* client_ip) {
        if (listen_fd_ < 0) {
        return false;
        }

        pollfd pfd{};
        pfd.fd = listen_fd_;
        pfd.events = POLLIN;
        int pret = ::poll(&pfd, 1, timeout_ms);
        if (pret <= 0 || !(pfd.revents & POLLIN)) {
            return false;
        }

        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);
        int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&peer), &peer_len);
        if (fd < 0) {
            return false;
        }

        int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags >= 0) {
            ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        }
        configureSocketLowLatency(fd);

        closeClient();
        conn_fd_ = fd;

        if (client_ip) {
            char buf[INET_ADDRSTRLEN] = {0};
            if (::inet_ntop(AF_INET, &peer.sin_addr, buf, sizeof(buf))) {
                *client_ip = buf;
            }
        }
        return true;
        }

        bool RawTcpChannel::sendAll(const void* data, size_t len) {
        if (conn_fd_ < 0 || !data || len == 0) {
        return false;
        }

        const auto* ptr = static_cast<const uint8_t*>(data);
        size_t sent_total = 0;
        while (sent_total < len) {
            ssize_t n = ::send(conn_fd_, ptr + sent_total, len - sent_total, MSG_NOSIGNAL);
            if (n > 0) {
                sent_total += static_cast<size_t>(n);
            } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                pollfd pfd{};
                pfd.fd = conn_fd_;
                pfd.events = POLLOUT;
                if (::poll(&pfd, 1, 20) <= 0) {
                    return false;
                }
            } else {
                closeClient();
                return false;
            }
        }
        return true;
        }

        ssize_t RawTcpChannel::recv(void* buffer, size_t max_len, int timeout_ms) {
        if (conn_fd_ < 0 || !buffer || max_len == 0) {
        return -1;
        }

        if (timeout_ms != 0) {
            pollfd pfd{};
            pfd.fd = conn_fd_;
            pfd.events = POLLIN;
            int pret = ::poll(&pfd, 1, timeout_ms);
            if (pret <= 0 || !(pfd.revents & POLLIN)) {
                return 0;
            }
        }

        ssize_t n = ::recv(conn_fd_, buffer, max_len, 0);
        if (n == 0) {
            closeClient();
            return -1;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;
            }
            closeClient();
            return -1;
        }
        return n;
        }

        bool RawTcpChannel::recvExact(void* buffer, size_t len, int timeout_ms) {
        if (conn_fd_ < 0 || !buffer || len == 0) {
        return false;
        }
        auto* ptr = static_cast<uint8_t*>(buffer);
        size_t received = 0;
        while (received < len) {
        ssize_t n = recv(ptr + received, len - received, timeout_ms);
        if (n <= 0) {
        return false;
        }
        received += static_cast<size_t>(n);
        }
        return true;
        }

        void RawTcpChannel::closeClient() {
        if (conn_fd_ >= 0) {
        ::close(conn_fd_);
        conn_fd_ = -1;
        }
        }

        void RawTcpChannel::closeAll() {
        closeClient();
        if (listen_fd_ >= 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
        }
        }

        } // namespace rdl_ctrl::comm
        """

    @staticmethod
    def _tmpl_ros_bridge_h() -> str:
        return r"""#pragma once
        #include "comm/raw_tcp_channel.h"
        #include "comm/raw_udp_channel.h"
        #include "common/robot_types.h"

        #include 
        #include 
        #include

        namespace rdl_ctrl::comm {

        constexpr uint32_t kRosStateMagic = 0x524F5353; // "ROSS"
        constexpr uint32_t kRosCmdMagic   = 0x524F5343; // "ROSC"

        #pragma pack(push, 1)
        struct RosStatePacketHeader {
        uint32_t magic;
        uint32_t nq;
        uint32_t nv;
        double   timestamp;
        };

        struct RosCmdPacketHeader {
        uint32_t magic;
        uint32_t mode; // Maps to ActuatorMode
        uint32_t nq;
        uint32_t nv;
        };
        #pragma pack(pop)

        enum class RosTransportType {
        UDP,
        TCP
        };

        struct RosBridgeConfig {
        RosTransportType transport = RosTransportType::@@ROS_TRANSPORT_UPPER@@;
        std::string remote_ip = "@@ROS_REMOTE_IP@@";
        uint16_t state_port = @@ROS_STATE_PORT@@; // UDP: remote port; TCP: local listen port
        uint16_t cmd_port   = @@ROS_CMD_PORT@@;   // UDP: local bind port; TCP: reuses state TCP connection
        };

        /

        Lightweight ROS communication bridge implemented strictly on top of raw UDP or TCP.

        Keeps the real-time C++ control binary 100% free of ROS SDK compile/link dependencies.
        */
        class RosBridge {
        public:
        RosBridge() = default;
        ~RosBridge();

        bool init(const RosBridgeConfig& cfg);
        void shutdown();

        bool publishJointState(const RobotState& state, const Eigen::VectorXd& tau_cmd);
        bool pollExternalCommand(RobotCommand& cmd_out, int timeout_ms = 0);

        private:
        bool parseCmdBuffer(const uint8_t* data, size_t len, RobotCommand& cmd_out) const;

        RosBridgeConfig cfg_{};
        RawUdpChannel udp_tx_;
        RawUdpChannel udp_rx_;
        RawTcpChannel tcp_server_;
        std::vector<uint8_t> tx_buf_;
        std::vector<uint8_t> rx_buf_;
        bool initialized_ = false;
        };

        } // namespace rdl_ctrl::comm
        """

    @staticmethod
    def _tmpl_ros_bridge_cpp() -> str:
        return r"""#include "comm/ros_bridge.h"
        #include 
        #include

        namespace rdl_ctrl::comm {

        RosBridge::~RosBridge() {
        shutdown();
        }

        bool RosBridge::init(const RosBridgeConfig& cfg) {
        shutdown();
        cfg_ = cfg;
        rx_buf_.resize(65536);

        if (cfg_.transport == RosTransportType::UDP) {
            if (!udp_tx_.open(0) || !udp_tx_.setDefaultPeer(cfg_.remote_ip, cfg_.state_port)) {
                std::cerr << "[RosBridge] Failed to open UDP TX socket.\n";
                return false;
            }
            if (!udp_rx_.open(cfg_.cmd_port)) {
                std::cerr << "[RosBridge] Failed to bind UDP RX port " << cfg_.cmd_port << ".\n";
                return false;
            }
            std::cout << "[RosBridge] UDP ready -> State TX " << cfg_.remote_ip << ":" << cfg_.state_port
                    << " | Cmd RX 0.0.0.0:" << cfg_.cmd_port << "\n";
        } else {
            if (!tcp_server_.listenOn(cfg_.state_port)) {
                std::cerr << "[RosBridge] Failed to listen on TCP port " << cfg_.state_port << ".\n";
                return false;
            }
            std::cout << "[RosBridge] TCP server listening on port " << cfg_.state_port
                    << " for ROS gateway connection.\n";
        }

        initialized_ = true;
        return true;
        }

        void RosBridge::shutdown() {
        udp_tx_.close();
        udp_rx_.close();
        tcp_server_.closeAll();
        initialized_ = false;
        }

        bool RosBridge::publishJointState(const RobotState& state, const Eigen::VectorXd& tau_cmd) {
        if (!initialized_ || !state.valid) {
        return false;
        }

        const uint32_t nq = static_cast<uint32_t>(state.q.size());
        const uint32_t nv = static_cast<uint32_t>(state.v.size());
        const size_t payload_bytes = sizeof(RosStatePacketHeader)
                                + (static_cast<size_t>(nq) + 2 * static_cast<size_t>(nv)) * sizeof(double);

        const size_t prefix_bytes = (cfg_.transport == RosTransportType::TCP) ? sizeof(uint32_t) : 0;
        tx_buf_.resize(prefix_bytes + payload_bytes);

        uint8_t* ptr = tx_buf_.data();
        if (cfg_.transport == RosTransportType::TCP) {
            if (!tcp_server_.isConnected()) {
                tcp_server_.acceptClient(0);
                if (!tcp_server_.isConnected()) {
                    return false;
                }
            }
            uint32_t frame_len = static_cast<uint32_t>(payload_bytes);
            std::memcpy(ptr, &frame_len, sizeof(frame_len));
            ptr += sizeof(frame_len);
        }

        RosStatePacketHeader hdr{};
        hdr.magic = kRosStateMagic;
        hdr.nq = nq;
        hdr.nv = nv;
        hdr.timestamp = state.timestamp;
        std::memcpy(ptr, &hdr, sizeof(hdr));
        ptr += sizeof(hdr);

        if (nq > 0) {
            std::memcpy(ptr, state.q.data(), static_cast<size_t>(nq) * sizeof(double));
            ptr += static_cast<size_t>(nq) * sizeof(double);
        }
        if (nv > 0) {
            std::memcpy(ptr, state.v.data(), static_cast<size_t>(nv) * sizeof(double));
            ptr += static_cast<size_t>(nv) * sizeof(double);
            if (static_cast<uint32_t>(tau_cmd.size()) == nv) {
                std::memcpy(ptr, tau_cmd.data(), static_cast<size_t>(nv) * sizeof(double));
            } else {
                std::memset(ptr, 0, static_cast<size_t>(nv) * sizeof(double));
            }
        }

        if (cfg_.transport == RosTransportType::UDP) {
            return udp_tx_.send(tx_buf_.data(), tx_buf_.size()) == static_cast<ssize_t>(tx_buf_.size());
        }
        return tcp_server_.sendAll(tx_buf_.data(), tx_buf_.size());
        }

        bool RosBridge::pollExternalCommand(RobotCommand& cmd_out, int timeout_ms) {
        if (!initialized_) {
        return false;
        }

        if (cfg_.transport == RosTransportType::UDP) {
            ssize_t last_n = 0;
            ssize_t n = udp_rx_.recv(rx_buf_.data(), rx_buf_.size(), timeout_ms);
            while (n > 0) {
                last_n = n;
                n = udp_rx_.recv(rx_buf_.data(), rx_buf_.size(), 0);
            }
            if (last_n > 0) {
                return parseCmdBuffer(rx_buf_.data(), static_cast<size_t>(last_n), cmd_out);
            }
            return false;
        }

        // TCP framed receive: [uint32_t payload_len][RosCmdPacketHeader][q_des][v_des][tau_cmd]
        if (!tcp_server_.isConnected()) {
            tcp_server_.acceptClient(0);
            return false;
        }

        uint32_t frame_len = 0;
        ssize_t n = tcp_server_.recv(&frame_len, sizeof(frame_len), timeout_ms);
        if (n != static_cast<ssize_t>(sizeof(frame_len)) || frame_len == 0 || frame_len > rx_buf_.size()) {
            return false;
        }
        if (!tcp_server_.recvExact(rx_buf_.data(), frame_len, 10)) {
            return false;
        }
        return parseCmdBuffer(rx_buf_.data(), frame_len, cmd_out);
        }

        bool RosBridge::parseCmdBuffer(const uint8_t* data, size_t len, RobotCommand& cmd_out) const {
        if (!data || len < sizeof(RosCmdPacketHeader)) {
        return false;
        }
        RosCmdPacketHeader hdr{};
        std::memcpy(&hdr, data, sizeof(hdr));
        if (hdr.magic != kRosCmdMagic) {
        return false;
        }

        const size_t q_bytes = static_cast<size_t>(hdr.nq) * sizeof(double);
        const size_t v_bytes = static_cast<size_t>(hdr.nv) * sizeof(double);
        const size_t required = sizeof(RosCmdPacketHeader) + q_bytes + 2 * v_bytes;
        if (len < required) {
            return false;
        }

        cmd_out.resize(static_cast<int>(hdr.nq), static_cast<int>(hdr.nv));
        cmd_out.mode = static_cast<ActuatorMode>(hdr.mode);

        const uint8_t* ptr = data + sizeof(RosCmdPacketHeader);
        if (hdr.nq > 0) {
            std::memcpy(cmd_out.q_des.data(), ptr, q_bytes);
            ptr += q_bytes;
        }
        if (hdr.nv > 0) {
            std::memcpy(cmd_out.v_des.data(), ptr, v_bytes);
            ptr += v_bytes;
            std::memcpy(cmd_out.tau_cmd.data(), ptr, v_bytes);
        }
        return true;
        }

        } // namespace rdl_ctrl::comm
        """

    @staticmethod
    def _tmpl_ros_gateway_py() -> str:
        return r"""#!/usr/bin/env python3
        -- coding: utf-8 --

        import argparse
        import socket
        import struct
        import threading
        import time

        MAGIC_ROSS = 0x524F5353
        MAGIC_ROSC = 0x524F5343
        STATE_HDR_FMT = "<IIId"
        STATE_HDR_SIZE = struct.calcsize(STATE_HDR_FMT)
        CMD_HDR_FMT = "<IIII"

        DEFAULT_JOINT_NAMES = [@@JOINT_NAMES_PY_LIST@@]

        class TransportLink:
        def init(self, transport: str, host: str, state_port: int, cmd_port: int):
        self.transport = transport.lower()
        self.host = host
        self.state_port = state_port
        self.cmd_port = cmd_port
        self.sock_rx = None
        self.sock_tx = None
        self.tcp_sock = None
        self._lock = threading.Lock()
        self._setup()

        def _setup(self):
            if self.transport == "udp":
                self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.sock_rx.bind(("0.0.0.0", self.state_port))
                self.sock_rx.settimeout(0.2)
                self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            else:
                self._connect_tcp()

        def _connect_tcp(self):
            while True:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    s.connect((self.host, self.state_port))
                    s.settimeout(0.2)
                    self.tcp_sock = s
                    print(f"[ROS Gateway] Connected to C++ TCP server {self.host}:{self.state_port}")
                    return
                except OSError:
                    time.sleep(0.5)

        def _recv_exact(self, n: int) -> bytes:
            buf = bytearray()
            while len(buf) < n:
                chunk = self.tcp_sock.recv(n - len(buf))
                if not chunk:
                    raise ConnectionError("TCP peer disconnected")
                buf.extend(chunk)
            return bytes(buf)

        def recv_state(self):
            try:
                if self.transport == "udp":
                    data, _ = self.sock_rx.recvfrom(65536)
                else:
                    if self.tcp_sock is None:
                        self._connect_tcp()
                    hdr_len_raw = self._recv_exact(4)
                    (payload_len,) = struct.unpack("<I", hdr_len_raw)
                    data = self._recv_exact(payload_len)
            except socket.timeout:
                return None
            except OSError:
                if self.transport == "tcp":
                    self.tcp_sock = None
                    time.sleep(0.2)
                return None

            if len(data) < STATE_HDR_SIZE:
                return None
            magic, nq, nv, ts = struct.unpack_from(STATE_HDR_FMT, data, 0)
            if magic != MAGIC_ROSS:
                return None
            expected = STATE_HDR_SIZE + (nq + 2 * nv) * 8
            if len(data) < expected:
                return None

            off = STATE_HDR_SIZE
            q = list(struct.unpack_from(f"<{nq}d", data, off))
            off += nq * 8
            v = list(struct.unpack_from(f"<{nv}d", data, off))
            off += nv * 8
            tau = list(struct.unpack_from(f"<{nv}d", data, off))
            return ts, q, v, tau

        def send_cmd(self, mode: int, q_des, v_des, tau_cmd):
            nq = len(q_des)
            nv = len(v_des)
            payload = struct.pack(CMD_HDR_FMT, MAGIC_ROSC, mode, nq, nv)
            if nq > 0:
                payload += struct.pack(f"<{nq}d", *q_des)
            if nv > 0:
                payload += struct.pack(f"<{nv}d", *v_des)
                payload += struct.pack(f"<{nv}d", *tau_cmd)

            with self._lock:
                try:
                    if self.transport == "udp":
                        self.sock_tx.sendto(payload, (self.host, self.cmd_port))
                    elif self.tcp_sock is not None:
                        self.tcp_sock.sendall(struct.pack("<I", len(payload)) + payload)
                except OSError:
                    pass
        def run_ros2(link: TransportLink):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        rclpy.init()

        class RdlGatewayNode(Node):
            def __init__(self):
                super().__init__("rdl_udp_tcp_gateway")
                self.pub = self.create_publisher(JointState, "/rdl/joint_states", 10)
                self.sub = self.create_subscription(JointState, "/rdl/joint_commands", self.on_cmd, 10)
                self._running = True
                self._rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
                self._rx_thread.start()

            def on_cmd(self, msg: JointState):
                nq = len(msg.position)
                nv = len(msg.velocity) if len(msg.velocity) > 0 else nq
                tau = list(msg.effort) if len(msg.effort) == nv else [0.0] * nv
                v = list(msg.velocity) if len(msg.velocity) == nv else [0.0] * nv
                link.send_cmd(mode=2, q_des=list(msg.position), v_des=v, tau_cmd=tau)

            def rx_loop(self):
                while self._running and rclpy.ok():
                    item = link.recv_state()
                    if item is None:
                        continue
                    _, q, v, tau = item
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = DEFAULT_JOINT_NAMES if len(DEFAULT_JOINT_NAMES) == len(q) else [f"joint_{i}" for i in range(len(q))]
                    msg.position = q
                    msg.velocity = v
                    msg.effort = tau
                    self.pub.publish(msg)

        node = RdlGatewayNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node._running = False
            node.destroy_node()
            rclpy.shutdown()
        def run_ros1(link: TransportLink):
        import rospy
        from sensor_msgs.msg import JointState

        rospy.init_node("rdl_udp_tcp_gateway", anonymous=False)
        pub = rospy.Publisher("/rdl/joint_states", JointState, queue_size=10)

        def on_cmd(msg: JointState):
            nq = len(msg.position)
            nv = len(msg.velocity) if len(msg.velocity) > 0 else nq
            tau = list(msg.effort) if len(msg.effort) == nv else [0.0] * nv
            v = list(msg.velocity) if len(msg.velocity) == nv else [0.0] * nv
            link.send_cmd(mode=2, q_des=list(msg.position), v_des=v, tau_cmd=tau)

        rospy.Subscriber("/rdl/joint_commands", JointState, on_cmd, queue_size=10)

        while not rospy.is_shutdown():
            item = link.recv_state()
            if item is None:
                continue
            _, q, v, tau = item
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = DEFAULT_JOINT_NAMES if len(DEFAULT_JOINT_NAMES) == len(q) else [f"joint_{i}" for i in range(len(q))]
            msg.position = q
            msg.velocity = v
            msg.effort = tau
            pub.publish(msg)
        def main():
            parser = argparse.ArgumentParser(description="ROS1/ROS2 UDP-TCP Gateway for RDL Controller")
            parser.add_argument("--transport", choices=["udp", "tcp"], default="@@ROS_TRANSPORT_LOWER@@")
            parser.add_argument("--host", default="@@ROS_REMOTE_IP@@")
            parser.add_argument("--state-port", type=int, default=@@ROS_STATE_PORT@@)
            parser.add_argument("--cmd-port", type=int, default=@@ROS_CMD_PORT@@)
            args = parser.parse_args()

            link = TransportLink(args.transport, args.host, args.state_port, args.cmd_port)
            try:
                import rclpy  # noqa: F401
                print("[ROS Gateway] Detected ROS2 (rclpy).")
                run_ros2(link)
            except ImportError:
                try:
                    import rospy  # noqa: F401
                    print("[ROS Gateway] Detected ROS1 (rospy).")
                    run_ros1(link)
                except ImportError:
                    print("[ROS Gateway] Neither rclpy (ROS2) nor rospy (ROS1) is installed in this Python env.")
                    print("[ROS Gateway] Running in console monitor mode (Ctrl+C to exit)...")
                    try:
                        while True:
                            item = link.recv_state()
                            if item is not None:
                                ts, q, v, tau = item
                                print(f"[t={ts:.3f}s] q={q[:4]} v={v[:4]} tau={tau[:4]}")
                    except KeyboardInterrupt:
                        pass
            if name == "main":
                main()
        """

    @staticmethod
    def _tmpl_joint_interface_h() -> str:
        return r"""#pragma once
        #include "common/robot_types.h"

        #include 
        #include 
        #include

        namespace rdl_ctrl::hal {

        /

        Pure virtual interface representing a single physical or simulated joint/motor actuator.
        */
        class IJointActuator {
        public:
        virtual ~IJointActuator() = default;

        virtual int id() const = 0;
        virtual std::string name() const = 0;

        virtual bool enable() = 0;
        virtual bool disable() = 0;
        virtual bool clearFault() = 0;
        virtual bool isEnabled() const = 0;
        virtual uint32_t faultCode() const = 0;

        virtual double position() const = 0;
        virtual double velocity() const = 0;
        virtual double torque() const = 0;

        virtual void setTorqueCommand(double tau) = 0;
        virtual void setImpedanceCommand(double q_des, double v_des, double kp, double kd, double tau_ff) = 0;
        };

        /

        Pure virtual interface representing a complete robot actuator bus/backend.

        Both real hardware buses (EtherCAT/CAN) and simulators (MuJoCo) inherit from this class.
        */
        class IRobotHardware {
        public:
        virtual ~IRobotHardware() = default;

        virtual std::string name() const = 0;

        virtual bool init() = 0;
        virtual void shutdown() = 0;

        virtual bool enableAll() = 0;
        virtual bool disableAll() = 0;
        virtual bool clearFaults() = 0;

        virtual bool readState(RobotState& state, int timeout_ms = 50) = 0;
        virtual bool writeCommand(const RobotCommand& cmd) = 0;

        virtual int numPositionDofs() const = 0;
        virtual int numVelocityDofs() const = 0;
        virtual bool isFloatingBase() const = 0;
        virtual bool isConnected() const = 0;

        virtual size_t jointCount() const = 0;
        virtual IJointActuator* getJoint(size_t index) = 0;
        };

        } // namespace rdl_ctrl::hal
        """

    @staticmethod
    def _tmpl_mujoco_driver_h() -> str:
        return r"""#pragma once
        #include "hal/joint_interface.h"

        #include 
        #include 
        #include 
        #include

        namespace rdl_ctrl::hal {

        class MujocoJointActuator final : public IJointActuator {
        public:
        MujocoJointActuator(int id, std::string name) : id_(id), name_(std::move(name)) {}

        int id() const override { return id_; }
        std::string name() const override { return name_; }

        bool enable() override { enabled_ = true; return true; }
        bool disable() override { enabled_ = false; tau_cmd_ = 0.0; return true; }
        bool clearFault() override { fault_code_ = 0; return true; }
        bool isEnabled() const override { return enabled_; }
        uint32_t faultCode() const override { return fault_code_; }

        double position() const override { return q_; }
        double velocity() const override { return v_; }
        double torque() const override { return tau_meas_; }

        void setTorqueCommand(double tau) override { tau_cmd_ = enabled_ ? tau : 0.0; }
        void setImpedanceCommand(double q_des, double v_des, double kp, double kd, double tau_ff) override {
            tau_cmd_ = enabled_ ? (tau_ff + kp * (q_des - q_) + kd * (v_des - v_)) : 0.0;
        }

        double torqueCommand() const { return tau_cmd_; }
        void updateFeedback(double q, double v, double tau_meas) {
            q_ = q;
            v_ = v;
            tau_meas_ = tau_meas;
        }
        private:
        int id_ = 0;
        std::string name_;
        bool enabled_ = true;
        uint32_t fault_code_ = 0;
        double q_ = 0.0;
        double v_ = 0.0;
        double tau_meas_ = 0.0;
        double tau_cmd_ = 0.0;
        };

        /

        MuJoCo simulator treated as a derived implementation of IRobotHardware.

        Communicates with robot_sim.py over ZeroMQ (pure libzmq C API) in causal lockstep.
        */
        class MujocoJointDriver final : public IRobotHardware {
        public:
        MujocoJointDriver(std::string sub_endpoint = "@@MUJOCO_SUB_EP@@",
        std::string pub_endpoint = "@@MUJOCO_PUB_EP@@",
        int expected_nq = @@DEFAULT_NQ@@,
        int expected_nv = @@DEFAULT_NV@@,
        bool floating_base = @@IS_FLOATING_BASE_BOOL@@);
        ~MujocoJointDriver() override;

        std::string name() const override { return "MuJoCo-ZMQ-Lockstep"; }

        bool init() override;
        void shutdown() override;

        bool enableAll() override;
        bool disableAll() override;
        bool clearFaults() override;

        bool readState(RobotState& state, int timeout_ms = 50) override;
        bool writeCommand(const RobotCommand& cmd) override;

        int numPositionDofs() const override { return nq_; }
        int numVelocityDofs() const override { return nv_; }
        bool isFloatingBase() const override { return is_floating_base_; }
        bool isConnected() const override { return connected_; }

        size_t jointCount() const override { return joints_.size(); }
        IJointActuator* getJoint(size_t index) override {
        return index < joints_.size() ? joints_[index].get() : nullptr;
        }

        private:
        void syncJointObjects(int nq, int nv);

        std::string sub_endpoint_;
        std::string pub_endpoint_;
        int nq_ = 0;
        int nv_ = 0;
        bool is_floating_base_ = false;
        bool connected_ = false;
        bool enabled_ = true;

        void* zmq_ctx_ = nullptr;
        void* sub_sock_ = nullptr;
        void* pub_sock_ = nullptr;

        std::vector<uint8_t> recv_buf_;
        std::vector<uint8_t> send_buf_;
        std::vector<std::unique_ptr<MujocoJointActuator>> joints_;
        };

        } // namespace rdl_ctrl::hal
        """

    @staticmethod
    def _tmpl_mujoco_driver_cpp() -> str:
        return r"""#include "hal/mujoco_joint_driver.h"
        #include "common/robot_msg.h"

        #include <zmq.h>
        #include 
        #include

        namespace rdl_ctrl::hal {

        MujocoJointDriver::MujocoJointDriver(std::string sub_endpoint,
        std::string pub_endpoint,
        int expected_nq,
        int expected_nv,
        bool floating_base)
        : sub_endpoint_(std::move(sub_endpoint)),
        pub_endpoint_(std::move(pub_endpoint)),
        nq_(expected_nq),
        nv_(expected_nv),
        is_floating_base_(floating_base) {
        recv_buf_.resize(256 * 1024);
        syncJointObjects(nq_, nv_);
        }

        MujocoJointDriver::~MujocoJointDriver() {
        shutdown();
        }

        void MujocoJointDriver::syncJointObjects(int nq, int nv) {
        nq_ = nq;
        nv_ = nv;
        is_floating_base_ = (nq == nv + 1);
        if (static_cast(joints_.size()) != nv_) {
        joints_.clear();
        joints_.reserve(static_cast<size_t>(nv_));
        for (int i = 0; i < nv_; ++i) {
        joints_.push_back(std::make_unique(i, "mj_dof_" + std::to_string(i)));
        }
        }
        }

        bool MujocoJointDriver::init() {
        shutdown();

        zmq_ctx_ = zmq_ctx_new();
        if (!zmq_ctx_) {
            std::cerr << "[MujocoJointDriver] Failed to create ZMQ context.\n";
            return false;
        }

        sub_sock_ = zmq_socket(zmq_ctx_, ZMQ_SUB);
        pub_sock_ = zmq_socket(zmq_ctx_, ZMQ_PUB);
        if (!sub_sock_ || !pub_sock_) {
            shutdown();
            return false;
        }

        int hwm = 16;
        int linger = 0;
        zmq_setsockopt(sub_sock_, ZMQ_RCVHWM, &hwm, sizeof(hwm));
        zmq_setsockopt(sub_sock_, ZMQ_LINGER, &linger, sizeof(linger));
        zmq_setsockopt(sub_sock_, ZMQ_SUBSCRIBE, "", 0);

        zmq_setsockopt(pub_sock_, ZMQ_SNDHWM, &hwm, sizeof(hwm));
        zmq_setsockopt(pub_sock_, ZMQ_LINGER, &linger, sizeof(linger));

        if (zmq_connect(sub_sock_, sub_endpoint_.c_str()) != 0) {
            std::cerr << "[MujocoJointDriver] Failed to connect SUB to " << sub_endpoint_ << "\n";
            shutdown();
            return false;
        }
        if (zmq_connect(pub_sock_, pub_endpoint_.c_str()) != 0) {
            std::cerr << "[MujocoJointDriver] Failed to connect PUB to " << pub_endpoint_ << "\n";
            shutdown();
            return false;
        }

        std::cout << "[MujocoJointDriver] Connected -> SUB: " << sub_endpoint_
                << " | PUB: " << pub_endpoint_ << "\n";
        return true;
        }

        void MujocoJointDriver::shutdown() {
        connected_ = false;
        if (sub_sock_) {
        zmq_close(sub_sock_);
        sub_sock_ = nullptr;
        }
        if (pub_sock_) {
        zmq_close(pub_sock_);
        pub_sock_ = nullptr;
        }
        if (zmq_ctx_) {
        zmq_ctx_term(zmq_ctx_);
        zmq_ctx_ = nullptr;
        }
        }

        bool MujocoJointDriver::enableAll() {
        enabled_ = true;
        for (auto& j : joints_) {
        j->enable();
        }
        return true;
        }

        bool MujocoJointDriver::disableAll() {
        enabled_ = false;
        for (auto& j : joints_) {
        j->disable();
        }
        return true;
        }

        bool MujocoJointDriver::clearFaults() {
        for (auto& j : joints_) {
        j->clearFault();
        }
        return true;
        }

        bool MujocoJointDriver::readState(RobotState& state, int timeout_ms) {
        if (!sub_sock_) {
        return false;
        }

        zmq_pollitem_t items[1] = {{sub_sock_, 0, ZMQ_POLLIN, 0}};
        int rc = zmq_poll(items, 1, timeout_ms);
        if (rc <= 0 || !(items[0].revents & ZMQ_POLLIN)) {
            return false;
        }

        // Drain queue so the controller always processes the freshest simulation frame
        int latest_bytes = -1;
        std::vector<uint8_t> tmp_buf(recv_buf_.size());
        while (true) {
            int n = zmq_recv(sub_sock_, tmp_buf.data(), tmp_buf.size(), ZMQ_DONTWAIT);
            if (n > 0) {
                if (static_cast<size_t>(n) > recv_buf_.size()) {
                    // Message was truncated; enlarge buffer for subsequent packets
                    recv_buf_.resize(static_cast<size_t>(n) * 2);
                    tmp_buf.resize(recv_buf_.size());
                    continue;
                }
                std::memcpy(recv_buf_.data(), tmp_buf.data(), static_cast<size_t>(n));
                latest_bytes = n;
            } else {
                break;
            }
        }

        if (latest_bytes <= 0) {
            return false;
        }

        const SimStateHeader* header = nullptr;
        const double* qpos_ptr = nullptr;
        const double* qvel_ptr = nullptr;
        const double* qfrc_ptr = nullptr;
        const ContactItem* contacts_ptr = nullptr;

        if (!parseSimStateZeroCopy(recv_buf_.data(),
                                static_cast<size_t>(latest_bytes),
                                header,
                                qpos_ptr,
                                qvel_ptr,
                                qfrc_ptr,
                                contacts_ptr)) {
            return false;
        }

        const int nq = static_cast<int>(header->nq);
        const int nv = static_cast<int>(header->nv);
        if (nq != nq_ || nv != nv_) {
            syncJointObjects(nq, nv);
        }

        state.timestamp = header->timestamp;
        if (state.q.size() != nq) state.q.resize(nq);
        if (state.v.size() != nv) state.v.resize(nv);
        if (state.tau_meas.size() != nv) state.tau_meas.setZero(nv);
        if (state.qfrc_constraint.size() != nv) state.qfrc_constraint.resize(nv);

        if (nq > 0) {
            std::memcpy(state.q.data(), qpos_ptr, static_cast<size_t>(nq) * sizeof(double));
        }
        if (nv > 0) {
            std::memcpy(state.v.data(), qvel_ptr, static_cast<size_t>(nv) * sizeof(double));
            std::memcpy(state.qfrc_constraint.data(), qfrc_ptr, static_cast<size_t>(nv) * sizeof(double));
        }

        state.contacts.clear();
        state.contacts.reserve(header->ncon);
        for (uint32_t i = 0; i < header->ncon; ++i) {
            ContactInfo ci;
            ci.geom1 = contacts_ptr[i].geom1;
            ci.geom2 = contacts_ptr[i].geom2;
            ci.dist = contacts_ptr[i].dist;
            ci.pos = Eigen::Map<const Eigen::Vector3d>(contacts_ptr[i].pos);
            ci.normal = Eigen::Map<const Eigen::Vector3d>(contacts_ptr[i].normal);
            ci.force_local = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(contacts_ptr[i].force_local);
            ci.force_world = Eigen::Map<const Eigen::Vector3d>(contacts_ptr[i].force_world);
            state.contacts.push_back(ci);
        }

        // Update per-joint actuator mirrors
        for (int i = 0; i < nv; ++i) {
            int q_idx = is_floating_base_ ? (i + 1) : i;
            double q_val = (q_idx >= 0 && q_idx < nq) ? state.q(q_idx) : 0.0;
            joints_[static_cast<size_t>(i)]->updateFeedback(q_val, state.v(i), joints_[static_cast<size_t>(i)]->torqueCommand());
        }

        state.valid = true;
        connected_ = true;
        return true;
        }

        bool MujocoJointDriver::writeCommand(const RobotCommand& cmd) {
        if (!pub_sock_ || nv_ <= 0) {
        return false;
        }

        if (cmd.mode == ActuatorMode::DIRECT_STATE && cmd.q_des.size() >= nq_ && cmd.v_des.size() >= nv_) {
            const size_t q_bytes = static_cast<size_t>(nq_) * sizeof(double);
            const size_t v_bytes = static_cast<size_t>(nv_) * sizeof(double);
            const size_t packet_size = sizeof(RdlCmdHeader) + q_bytes + v_bytes;
            send_buf_.resize(packet_size);

            auto* hdr = reinterpret_cast<RdlCmdHeader*>(send_buf_.data());
            hdr->magic = kMagicRdlm;
            hdr->mode = 1;
            hdr->nq = static_cast<uint32_t>(nq_);
            hdr->nv = static_cast<uint32_t>(nv_);

            std::memcpy(send_buf_.data() + sizeof(RdlCmdHeader), cmd.q_des.data(), q_bytes);
            std::memcpy(send_buf_.data() + sizeof(RdlCmdHeader) + q_bytes, cmd.v_des.data(), v_bytes);
            return zmq_send(pub_sock_, send_buf_.data(), send_buf_.size(), ZMQ_DONTWAIT) >= 0;
        }

        // Torque mode (mode = 0)
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(nv_);
        if (enabled_ && cmd.mode != ActuatorMode::DISABLED && cmd.tau_cmd.size() >= nv_) {
            tau = cmd.tau_cmd.head(nv_);
        }
        for (int i = 0; i < nv_; ++i) {
            joints_[static_cast<size_t>(i)]->setTorqueCommand(tau(i));
        }

        const size_t tau_bytes = static_cast<size_t>(nv_) * sizeof(double);
        const size_t packet_size = sizeof(RdlCmdHeader) + tau_bytes;
        send_buf_.resize(packet_size);

        auto* hdr = reinterpret_cast<RdlCmdHeader*>(send_buf_.data());
        hdr->magic = kMagicRdlm;
        hdr->mode = 0;
        hdr->nq = static_cast<uint32_t>(nq_);
        hdr->nv = static_cast<uint32_t>(nv_);

        std::memcpy(send_buf_.data() + sizeof(RdlCmdHeader), tau.data(), tau_bytes);
        return zmq_send(pub_sock_, send_buf_.data(), send_buf_.size(), ZMQ_DONTWAIT) >= 0;
        }

        } // namespace rdl_ctrl::hal
        """

    @staticmethod
    def _tmpl_hardware_stub_h() -> str:
        return r"""#pragma once
        #include "hal/joint_interface.h"

        #include 
        #include 
        #include

        namespace rdl_ctrl::hal {

        /

        Template stub demonstrating how to implement real motor hardware

        (e.g., EtherCAT/SOEM, CANopen, or UDP/TCP servo bus) under the same IRobotHardware interface.
        */
        class HardwareJointStub final : public IRobotHardware {
        public:
        explicit HardwareJointStub(int dof = @@DEFAULT_NV@@);
        ~HardwareJointStub() override = default;

        std::string name() const override { return "RealHardware-Stub"; }

        bool init() override;
        void shutdown() override;

        bool enableAll() override;
        bool disableAll() override;
        bool clearFaults() override;

        bool readState(RobotState& state, int timeout_ms = 50) override;
        bool writeCommand(const RobotCommand& cmd) override;

        int numPositionDofs() const override { return dof_; }
        int numVelocityDofs() const override { return dof_; }
        bool isFloatingBase() const override { return false; }
        bool isConnected() const override { return initialized_; }

        size_t jointCount() const override { return 0; }
        IJointActuator* getJoint(size_t) override { return nullptr; }

        private:
        int dof_ = 0;
        bool initialized_ = false;
        };

        } // namespace rdl_ctrl::hal
        """

    @staticmethod
    def _tmpl_hardware_stub_cpp() -> str:
        return r"""#include "hal/hardware_joint_stub.h"
        #include

        namespace rdl_ctrl::hal {

        HardwareJointStub::HardwareJointStub(int dof) : dof_(dof) {}

        bool HardwareJointStub::init() {
        std::cout << "[HardwareJointStub] Init placeholder for " << dof_ << " physical joints.\n";
        initialized_ = true;
        return true;
        }

        void HardwareJointStub::shutdown() {
        initialized_ = false;
        }

        bool HardwareJointStub::enableAll() {
        // TODO: Issue servo-on / CiA-402 enable state machine to real motors
        return initialized_;
        }

        bool HardwareJointStub::disableAll() {
        // TODO: Issue servo-off / zero torque to real motors
        return true;
        }

        bool HardwareJointStub::clearFaults() {
        // TODO: Issue fault-reset controlword to real motors
        return true;
        }

        bool HardwareJointStub::readState(RobotState& state, int /timeout_ms/) {
        if (!initialized_) return false;
        if (state.q.size() != dof_) state.resize(dof_, dof_);
        // TODO: Populate state.q, state.v, state.tau_meas from fieldbus RxPDO
        state.valid = true;
        return true;
        }

        bool HardwareJointStub::writeCommand(const RobotCommand& /cmd/) {
        if (!initialized_) return false;
        // TODO: Write cmd.tau_cmd / cmd.q_des to fieldbus TxPDO
        return true;
        }

        } // namespace rdl_ctrl::hal
        """

    @staticmethod
    def _tmpl_rdl_controller_h() -> str:
        return r"""#pragma once
        #include "common/robot_types.h"
        #include "hal/joint_interface.h"

        #include <Eigen/Dense>
        #include 
        #include 
        #include#include 
        #include

        #include <graph.h>
        #include <robot_dynamics.h>

        namespace rdl_ctrl {

        struct ControllerParams {
        std::string model_file = "@@MODEL_REL_PATH@@";
        std::string rotor_inertia_txt = "config/rotor_inertia.txt";
        bool is_floating_base = @@IS_FLOATING_BASE_BOOL@@;
        double kp = @@DEFAULT_KP@@;
        double ki = @@DEFAULT_KI@@;
        double kd = @@DEFAULT_KD@@;
        double traj_amp = @@DEFAULT_TRAJ_AMP@@;
        double traj_freq = @@DEFAULT_TRAJ_FREQ@@;
        double integral_limit = 2.0;
        bool use_mass_matrix_ctc = false; // false: tau_grav + tau_pid; true: tau_grav + M * (a_des + tau_pid)
        };

        class RdlController {
        public:
        explicit RdlController(const ControllerParams& params);

        bool init();

        /**
        * Executes one control step given the latest RobotState from any IRobotHardware implementation.
        * Optionally accepts an external reference command (e.g., from ROS / UDP / TCP).
        */
        bool computeCommand(const RobotState& state,
                            RobotCommand& cmd_out,
                            const RobotCommand* external_ref = nullptr);

        int nq() const { return dof_q_; }
        int nv() const { return dof_dyn_; }
        bool isFloatingBase() const { return params_.is_floating_base; }

        const Eigen::VectorXd& desiredPosition() const { return q_des_; }
        const Eigen::VectorXd& desiredVelocity() const { return v_des_; }
        const Eigen::VectorXd& lastTorqueCommand() const { return tau_cmd_; }
        private:
        bool loadModel(bool floating_base);
        Eigen::VectorXd loadRotorInertiaFromTxt(const std::string& txt_path, int dof_dyn) const;
        void resizeBuffers(int nq, int nv);

        ControllerParams params_;
        std::map<std::string, int> joint_map_;
        std::shared_ptr<RDL::RobotDynamics> rdl_dyn_;

        int dof_q_ = 0;
        int dof_dyn_ = 0;

        Eigen::VectorXd rotor_inertia_;
        Eigen::VectorXd q_init_;
        Eigen::VectorXd q_des_;
        Eigen::VectorXd v_des_;
        Eigen::VectorXd a_des_;
        Eigen::VectorXd tau_pid_;
        Eigen::MatrixXd M_;
        Eigen::VectorXd tau_grav_;
        Eigen::VectorXd tau_cmd_;
        Eigen::VectorXd integral_err_;

        bool connected_and_latched_ = false;
        double last_timestamp_ = -1.0;
        double nominal_dt_ = @@SIM_TIMESTEP@@;
        uint64_t step_counter_ = 0;
        std::chrono::steady_clock::time_point wall_log_t0_;
        };

        } // namespace rdl_ctrl
        """

    @staticmethod
    def _tmpl_rdl_controller_cpp() -> str:
        return r"""#include "controller/rdl_controller.h"
        #if __has_include(<robot_loader.h>)
        #include <robot_loader.h>
        #elif __has_include(<utils/robot_loader.h>)
        #include <utils/robot_loader.h>
        #elif __has_include(<rdl/robot_loader.h>)
        #include <rdl/robot_loader.h>
        #else
        #error "robot_loader.h not found! Check your rdl//include directory."
        #endif

        #include 
        #include 
        #include 
        #include 
        #include 
        #include

        namespace rdl_ctrl {

        namespace {
        constexpr double kPi = 3.14159265358979323846;
        }

        RdlController::RdlController(const ControllerParams& params)
        : params_(params), wall_log_t0_(std::chrono::steady_clock::now()) {}

        bool RdlController::init() {
        if (!loadModel(params_.is_floating_base)) {
        return false;
        }

        std::cout << "========================================================\n";
        std::cout << "[RDL Controller] Model: " << params_.model_file << "\n";
        std::cout << "[RDL Controller] Floating Base: " << (params_.is_floating_base ? "Yes" : "No")
                << " | nq: " << dof_q_ << " | nv: " << dof_dyn_ << "\n";
        std::cout << "[RDL Controller] Gains -> Kp: " << params_.kp
                << ", Ki: " << params_.ki << ", Kd: " << params_.kd << "\n";
        std::cout << "[RDL Controller] Trajectory -> Amp: " << params_.traj_amp
                << " rad, Freq: " << params_.traj_freq << " Hz\n";
        std::cout << "[RDL Controller] Inertia TXT -> " << params_.rotor_inertia_txt << "\n";
        std::cout << "========================================================\n";
        return true;
        }

        bool RdlController::loadModel(bool floating_base) {
        auto file_type = (params_.model_file.find(".xml") != std::string::npos)
        ? RDL::Utils::RobotFileType::MJCF
        : RDL::Utils::RobotFileType::URDF;

        joint_map_.clear();
        auto loaded = RDL::Utils::load_RobotDynamics(params_.model_file, joint_map_, file_type, floating_base);
        if (!loaded) {
            std::cerr << "[RDL Error] Failed to load model file: " << params_.model_file << "\n";
            return false;
        }

        rdl_dyn_ = loaded;
        params_.is_floating_base = floating_base;
        dof_q_ = rdl_dyn_->dim_position();
        dof_dyn_ = rdl_dyn_->DOF();
        resizeBuffers(dof_q_, dof_dyn_);
        rotor_inertia_ = loadRotorInertiaFromTxt(params_.rotor_inertia_txt, dof_dyn_);

        for (const auto& kv : joint_map_) {
            std::cout << "  [Joint] " << kv.second << " : " << kv.first << "\n";
        }
        return true;
        }

        void RdlController::resizeBuffers(int nq, int nv) {
        q_init_.setZero(nq);
        q_des_.setZero(nq);
        v_des_.setZero(nv);
        a_des_.setZero(nv);
        tau_pid_.setZero(nv);
        M_.setZero(nv, nv);
        tau_grav_.setZero(nv);
        tau_cmd_.setZero(nv);
        integral_err_.setZero(nv);
        }

        Eigen::VectorXd RdlController::loadRotorInertiaFromTxt(const std::string& txt_path, int dof_dyn) const {
        Eigen::VectorXd inertia_vec = Eigen::VectorXd::Zero(dof_dyn);
        std::ifstream infile(txt_path);
        if (!infile.is_open()) {
        std::cout << "[RDL Inertia] File '" << txt_path
        << "' not found. Using default rotor inertia = 0.0 for all "
        << dof_dyn << " DOFs.\n";
        return inertia_vec;
        }

        int idx = 0;
        std::string line;
        while (std::getline(infile, line) && idx < dof_dyn) {
            size_t comment_pos = line.find('#');
            if (comment_pos != std::string::npos) {
                line = line.substr(0, comment_pos);
            }
            std::replace(line.begin(), line.end(), ',', ' ');
            std::stringstream ss(line);
            double val = 0.0;
            while (ss >> val && idx < dof_dyn) {
                inertia_vec(idx++) = val;
            }
        }

        std::cout << "[RDL Inertia] Loaded " << idx << "/" << dof_dyn
                << " values from '" << txt_path << "': "
                << inertia_vec.transpose() << "\n";
        return inertia_vec;
        }

        bool RdlController::computeCommand(const RobotState& state,
        RobotCommand& cmd_out,
        const RobotCommand* external_ref) {
        if (!rdl_dyn_ || !state.valid) {
        return false;
        }

        // Auto-adapt if hardware / simulator dimensions differ (e.g., floating base mismatch)
        if (static_cast<int>(state.q.size()) != dof_q_ || static_cast<int>(state.v.size()) != dof_dyn_) {
            bool retry_fb = (state.q.size() == state.v.size() + 1);
            if (loadModel(retry_fb) &&
                dof_q_ == static_cast<int>(state.q.size()) &&
                dof_dyn_ == static_cast<int>(state.v.size())) {
                connected_and_latched_ = false;
                last_timestamp_ = -1.0;
                std::cout << "[RDL Controller] Auto-matched model dimensions (Floating Base: "
                        << (params_.is_floating_base ? "Yes" : "No")
                        << ", nq=" << dof_q_ << ", nv=" << dof_dyn_ << ")\n";
            } else {
                return false;
            }
        }

        const Eigen::VectorXd& q = state.q;
        const Eigen::VectorXd& v = state.v;
        const double t = state.timestamp;

        // Initial handshake or simulation reset (t == 0.0)
        if (!connected_and_latched_ || t == 0.0) {
            if (!connected_and_latched_ || last_timestamp_ > 0.0) {
                rotor_inertia_ = loadRotorInertiaFromTxt(params_.rotor_inertia_txt, dof_dyn_);
                std::cout << "[RDL Controller] Handshake complete! Latched q_init: "
                        << q.transpose() << "\n";
                wall_log_t0_ = std::chrono::steady_clock::now();
            }
            q_init_ = q;
            q_des_ = q_init_;
            v_des_.setZero();
            a_des_.setZero();
            integral_err_.setZero();
            connected_and_latched_ = true;
            last_timestamp_ = 0.0;
            step_counter_ = 0;
        } else if (std::abs(t - last_timestamp_) < 1e-12) {
            // Duplicate timestamp packet: resend previous torque without double-integrating
            cmd_out.mode = ActuatorMode::TORQUE;
            cmd_out.tau_cmd = tau_cmd_;
            cmd_out.q_des = q_des_;
            cmd_out.v_des = v_des_;
            return true;
        }

        double dt = t - last_timestamp_;
        if (dt > 1e-6 && dt <= 0.05) {
            nominal_dt_ = dt;
        } else {
            dt = nominal_dt_;
        }
        last_timestamp_ = t;

        // 1. Reference Trajectory (External ROS/UDP/TCP override or built-in smooth cosine)
        if (external_ref && external_ref->q_des.size() == dof_q_) {
            q_des_ = external_ref->q_des;
            v_des_ = (external_ref->v_des.size() == dof_dyn_) ? external_ref->v_des : Eigen::VectorXd::Zero(dof_dyn_);
            a_des_.setZero();
        } else {
            q_des_ = q_init_;
            v_des_.setZero();
            a_des_.setZero();

            if (t > 0.0) {
                const double ramp = std::min(1.0, t / 1.0);
                const int start_v_idx = params_.is_floating_base ? 6 : 0;

                for (int i = start_v_idx; i < dof_dyn_; ++i) {
                    const int q_idx = params_.is_floating_base ? (i + 1) : i;
                    const int jnt_order = i - start_v_idx;

                    const double sign = (jnt_order % 2 == 0) ? 1.0 : -1.0;
                    const double amp_i = sign * params_.traj_amp / (1.0 + 0.15 * jnt_order);
                    const double omega_i = 2.0 * kPi * params_.traj_freq * (1.0 + 0.08 * jnt_order);
                    const double phase = omega_i * t;

                    q_des_(q_idx) = q_init_(q_idx) + amp_i * (1.0 - std::cos(phase));
                    v_des_(i) = amp_i * omega_i * std::sin(phase);
                    a_des_(i) = ramp * amp_i * omega_i * omega_i * std::cos(phase);
                }
            }
        }

        // 2. RDL Dynamics: Gravity Torque and Mass Matrix M(q)
        tau_grav_.setZero();
        M_.setZero();
        rdl_dyn_->forward_kinematics(q.data(), q.size());
        rdl_dyn_->cal_gravitational_torque(tau_grav_.data(), dof_dyn_);
        rdl_dyn_->forward_dynamics_crba(M_.data(), dof_dyn_);

        for (int r = 0; r < dof_dyn_; ++r) {
            for (int c = r + 1; c < dof_dyn_; ++c) {
                if (std::abs(M_(r, c)) > 0.0 && std::abs(M_(c, r)) == 0.0) {
                    M_(c, r) = M_(r, c);
                } else if (std::abs(M_(c, r)) > 0.0 && std::abs(M_(r, c)) == 0.0) {
                    M_(r, c) = M_(c, r);
                }
            }
        }
        M_.diagonal() += rotor_inertia_;

        // 3. Feedback Control Law
        tau_pid_.setZero();
        const double effective_kd = std::max(params_.kd, 1.6 * std::sqrt(std::max(0.0, params_.kp)));
        const int start_v_idx = params_.is_floating_base ? 6 : 0;

        if (!params_.is_floating_base || (dof_q_ >= 7 && dof_dyn_ >= 6)) {
            for (int i = start_v_idx; i < dof_dyn_; ++i) {
                const int q_idx = params_.is_floating_base ? (i + 1) : i;
                const double pos_err = q_des_(q_idx) - q(q_idx);
                if (t > 0.0) {
                    integral_err_(i) = std::clamp(
                        integral_err_(i) + pos_err * dt, -params_.integral_limit, params_.integral_limit);
                }
                const double vel_err = v_des_(i) - v(i);
                tau_pid_(i) = params_.kp * pos_err + params_.ki * integral_err_(i) + effective_kd * vel_err;
            }
        }

        // 4. Torque Synthesis
        if (params_.use_mass_matrix_ctc) {
            tau_cmd_ = tau_grav_ + M_ * (a_des_ + tau_pid_);
        } else {
            tau_cmd_ = tau_grav_ + tau_pid_;
        }

        if (external_ref && external_ref->tau_cmd.size() == dof_dyn_) {
            tau_cmd_ += external_ref->tau_cmd;
        }

        cmd_out.mode = ActuatorMode::TORQUE;
        cmd_out.q_des = q_des_;
        cmd_out.v_des = v_des_;
        cmd_out.tau_cmd = tau_cmd_;

        if (t > 0.0 && ++step_counter_ % 1000 == 0) {
            auto now_wall = std::chrono::steady_clock::now();
            double wall_sec = std::chrono::duration<double>(now_wall - wall_log_t0_).count();
            wall_log_t0_ = now_wall;

            double max_pos_err = (q_des_ - q).cwiseAbs().maxCoeff();
            double max_vel_err = (v_des_ - v).cwiseAbs().maxCoeff();
            std::cout << std::fixed << std::setprecision(4)
                    << "[RDL Lockstep] Sim t: " << t << "s (Wall dt: " << wall_sec << "s)"
                    << " | Max |e_q|: " << max_pos_err << " rad"
                    << " | Max |e_v|: " << max_vel_err << " rad/s\n";
        }

        return true;
        }

        } // namespace rdl_ctrl
        """

    @staticmethod
    def _tmpl_main_mujoco_test_cpp() -> str:
        return r"""#include "comm/raw_tcp_channel.h"
        #include "comm/raw_udp_channel.h"
        #include "controller/rdl_controller.h"
        #include "hal/mujoco_joint_driver.h"

        #ifdef ENABLE_ROS_BRIDGE
        #include "comm/ros_bridge.h"
        #endif

        #include 
        #include 
        #include 
        #include

        namespace {
        volatile std::sig_atomic_t g_running = 1;

        void signalHandler(int) {
        g_running = 0;
        }

        void printUsage(const char* prog) {
        std::cout << "Usage: " << prog << " [options]\n"
        << "  --model        Robot URDF/MJCF path (default: @@MODEL_REL_PATH@@)\n"
        << "  --floating <0|1>     Floating base flag (default: @@IS_FLOATING_BASE_INT@@)\n"
        << "  --kp            Proportional gain (default: @@DEFAULT_KP@@)\n"
        << "  --ki            Integral gain (default: @@DEFAULT_KI@@)\n"
        << "  --kd            Derivative gain (default: @@DEFAULT_KD@@)\n"
        << "  --amp           Cosine trajectory amp rad (default: @@DEFAULT_TRAJ_AMP@@)\n"
        << "  --freq          Cosine trajectory freq Hz (default: @@DEFAULT_TRAJ_FREQ@@)\n"
        << "  --inertia      Rotor inertia TXT (default: config/rotor_inertia.txt)\n"
        << "  --sub      MuJoCo PUB endpoint to subscribe (default: @@MUJOCO_SUB_EP@@)\n"
        << "  --pub      MuJoCo SUB endpoint to publish (default: @@MUJOCO_PUB_EP@@)\n"
        << "  --ctc <0|1>          Multiply PID by M(q) (default: 0)\n"
        << "  --udp-log      Optional raw UDP telemetry destination port on 127.0.0.1 (0=off)\n"
        << "  --help               Show this help\n";
        }
        } // namespace

        int main(int argc, char argv) {
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        rdl_ctrl::ControllerParams params;
        std::string sub_ep = "@@MUJOCO_SUB_EP@@";
        std::string pub_ep = "@@MUJOCO_PUB_EP@@";
        uint16_t udp_log_port = 0;

        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            auto nextVal = [&]() -> std::string {
                return (i + 1 < argc) ? std::string(argv[++i]) : std::string();
            };
            if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return 0;
            } else if (arg == "--model") {
                params.model_file = nextVal();
            } else if (arg == "--floating") {
                params.is_floating_base = (std::stoi(nextVal()) != 0);
            } else if (arg == "--kp") {
                params.kp = std::stod(nextVal());
            } else if (arg == "--ki") {
                params.ki = std::stod(nextVal());
            } else if (arg == "--kd") {
                params.kd = std::stod(nextVal());
            } else if (arg == "--amp") {
                params.traj_amp = std::stod(nextVal());
            } else if (arg == "--freq") {
                params.traj_freq = std::stod(nextVal());
            } else if (arg == "--inertia") {
                params.rotor_inertia_txt = nextVal();
            } else if (arg == "--sub") {
                sub_ep = nextVal();
            } else if (arg == "--pub") {
                pub_ep = nextVal();
            } else if (arg == "--ctc") {
                params.use_mass_matrix_ctc = (std::stoi(nextVal()) != 0);
            } else if (arg == "--udp-log") {
                udp_log_port = static_cast<uint16_t>(std::stoi(nextVal()));
            }
        }

        // 1. Instantiate MuJoCo driver polymorphically through the IRobotHardware virtual interface
        std::unique_ptr<rdl_ctrl::hal::IRobotHardware> robot_hw =
            std::make_unique<rdl_ctrl::hal::MujocoJointDriver>(
                sub_ep, pub_ep, @@DEFAULT_NQ@@, @@DEFAULT_NV@@, params.is_floating_base);

        if (!robot_hw->init()) {
            std::cerr << "[Main] Failed to initialize hardware driver: " << robot_hw->name() << "\n";
            return -1;
        }
        robot_hw->enableAll();

        // 2. Initialize RDL dynamics controller
        rdl_ctrl::RdlController controller(params);
        if (!controller.init()) {
            std::cerr << "[Main] Failed to initialize RDL controller.\n";
            return -1;
        }

        // 3. Optional raw UDP telemetry sender (demonstrates pure Linux raw UDP channel)
        rdl_ctrl::comm::RawUdpChannel udp_telemetry;
        if (udp_log_port > 0) {
            if (udp_telemetry.open(0) && udp_telemetry.setDefaultPeer("127.0.0.1", udp_log_port)) {
                std::cout << "[Main] Raw UDP telemetry streaming enabled -> 127.0.0.1:" << udp_log_port << "\n";
            }
        }
        #ifdef ENABLE_ROS_BRIDGE
        // 4. Optional ROS bridge over UDP/TCP
        rdl_ctrl::comm::RosBridge ros_bridge;
        rdl_ctrl::comm::RosBridgeConfig ros_cfg;
        bool ros_ready = ros_bridge.init(ros_cfg);
        rdl_ctrl::RobotCommand external_ros_cmd;
        bool has_external_ros_cmd = false;
        #endif

        rdl_ctrl::RobotState state;
        rdl_ctrl::RobotCommand cmd;

        std::cout << "[Main] Waiting for MuJoCo simulation frames (" << robot_hw->name() << ")...\n";

        while (g_running) {
            if (!robot_hw->readState(state, 50)) {
                continue;
            }

            const rdl_ctrl::RobotCommand* ext_ref_ptr = nullptr;
        #ifdef ENABLE_ROS_BRIDGE
        if (ros_ready) {
        if (ros_bridge.pollExternalCommand(external_ros_cmd, 0)) {
        has_external_ros_cmd = true;
        }
        if (has_external_ros_cmd) {
        ext_ref_ptr = &external_ros_cmd;
        }
        }
        #endif

            if (controller.computeCommand(state, cmd, ext_ref_ptr)) {
                robot_hw->writeCommand(cmd);

                if (udp_telemetry.isOpen() && state.q.size() > 0) {
                    udp_telemetry.send(state.q.data(), static_cast<size_t>(state.q.size()) * sizeof(double));
                }
        #ifdef ENABLE_ROS_BRIDGE
        if (ros_ready) {
        ros_bridge.publishJointState(state, cmd.tau_cmd);
        }
        #endif
        }
        }

        std::cout << "[Main] Shutting down cleanly.\n";
        robot_hw->disableAll();
        robot_hw->shutdown();
        return 0;
        }
        """

def main():
    parser = argparse.ArgumentParser(description="Generate a standalone Linux C++17 RDL robot control project.")
    parser.add_argument("--out", required=True, help="Target output folder for the generated C++ project")
    parser.add_argument("--name", default="rdl_robot_control", help="CMake project name")
    parser.add_argument("--model", default=None, help="Path to robot URDF or MJCF (.xml) file")
    parser.add_argument("--floating", action="store_true", help="Robot has a floating base")
    parser.add_argument("--nq", type=int, default=2, help="Position DOF count (nq)")
    parser.add_argument("--nv", type=int, default=2, help="Velocity DOF count (nv)")
    parser.add_argument("--rdl-platform", default="linux-x86_64", help="Platform folder name under rdl/")
    parser.add_argument("--enable-ros", action="store_true", help="Enable ROS communication bridge over UDP/TCP")
    parser.add_argument("--ros-transport", choices=["udp", "tcp"], default="udp", help="Transport for ROS bridge")
    parser.add_argument("--ros-ip", default="127.0.0.1", help="Remote IP for ROS UDP/TCP bridge")
    parser.add_argument("--ros-state-port", type=int, default=9001, help="ROS state port")
    parser.add_argument("--ros-cmd-port", type=int, default=9002, help="ROS command port")
    parser.add_argument("--sub-ep", default="tcp://127.0.0.1:5555", help="MuJoCo state PUB endpoint")
    parser.add_argument("--pub-ep", default="tcp://127.0.0.1:5556", help="MuJoCo cmd SUB endpoint")
    args = parser.parse_args()

    cfg = ProjectGenConfig(
        output_dir=args.out,
        project_name=args.name,
        model_file=args.model,
        is_floating_base=args.floating,
        nq=args.nq,
        nv=args.nv,
        rdl_platform_folder=args.rdl_platform,
        enable_ros_comm=args.enable_ros,
        ros_transport=args.ros_transport,
        ros_remote_ip=args.ros_ip,
        ros_state_port=args.ros_state_port,
        ros_cmd_port=args.ros_cmd_port,
        mujoco_sub_endpoint=args.sub_ep,
        mujoco_pub_endpoint=args.pub_ep,
    )

    gen = ControlProjectGenerator(cfg)
    result = gen.generate()
    print(f"[OK] Generated Linux C++ project '{result['project_name']}' at:\n  {result['output_dir']}")
    print(f"     Model: {result['model_rel_path']}")
    print(f"     ROS Bridge: {'Enabled (' + result['ros_transport'].upper() + ')' if result['enable_ros_comm'] else 'Disabled'}")
    print(f"     Files created: {len(result['generated_files'])}")
    if name == "main":
        main()