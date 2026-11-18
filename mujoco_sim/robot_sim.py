import sys
import os
import time
import struct
import ctypes
import threading
import argparse
import xml.etree.ElementTree as ET
import numpy as np
import zmq
import mujoco
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFileDialog, QTextEdit, QGroupBox, QOpenGLWidget,
    QFormLayout, QLineEdit, QCheckBox, QDialog, QScrollArea, QDoubleSpinBox
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QSurfaceFormat
from OpenGL.GL import glClearColor, glClear, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT


# Reduce Python GIL thread switch interval from default 5ms to 0.5ms to prevent GUI/Physics contention
sys.setswitchinterval(0.0005)

# Enable 1ms high-resolution OS timer on Windows
if sys.platform == "win32":
    try:
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass


# Binary protocol constants matching robot_msg.h (#pragma pack(push, 1))
MAGIC_RDLM = 0x52444C4D
SIM_HEADER_FMT = "<IIIId"       # magic(u32), nq(u32), nv(u32), ncon(u32), timestamp(f64) -> 24 bytes
CONTACT_ITEM_FMT = "<ii16d"     # geom1(i32), geom2(i32), dist(f64), pos(3d), normal(3d), force_local(6d), force_world(3d) -> 136 bytes
RDL_CMD_HEADER_FMT = "<IIII"    # magic(u32), mode(u32), nq(u32), nv(u32) -> 16 bytes
RDL_CMD_HEADER_SIZE = struct.calcsize(RDL_CMD_HEADER_FMT)


class XMLModelMerger:
    """Handles merging environment and robot MJCF/URDF models, resolving relative STL/asset paths, and injecting a white skybox."""

    @staticmethod
    def DefaultEnvXML():
        return """
        <mujoco model="default_environment">
            <option timestep="0.001" integrator="Euler" gravity="0 0 -9.81"/>
            <visual>
                <headlight ambient="0.45 0.45 0.45" diffuse="0.65 0.65 0.65" specular="0.3 0.3 0.3"/>
                <scale contactwidth="0.04" contactheight="0.015" forcewidth="0.015"/>
                <map force="0.005"/>
                <rgba fog="1 1 1 1" contactpoint="0.95 0.55 0.25 0.40" contactforce="0.25 0.65 0.95 0.35"/>
                <quality shadowsize="0"/>
            </visual>
            <asset>
                <texture name="white_skybox" type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="64" height="64"/>
                <texture name="grid" type="2d" builtin="checker" width="256" height="256" rgb1=".88 .88 .88" rgb2=".96 .96 .96"/>
                <material name="grid_mat" texture="grid" texrepeat="10 10" texuniform="true" reflectance="0"/>
            </asset>
            <worldbody>
                <light pos="0 0 4" dir="0 0 -1" directional="true" castshadow="false"/>
                <geom name="ground_plane" type="plane" size="10 10 0.1" material="grid_mat" condim="3" friction="1 0.005 0.0001"/>
            </worldbody>
        </mujoco>
        """

    @staticmethod
    def DemoRobotXML():
        return """
        <mujoco model="demo_robot">
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

    @classmethod
    def resolve_relative_paths(cls, root: ET.Element, base_dir: str):
        """Recursively resolves relative STL, OBJ, texture, and include paths into absolute file paths."""
        if not base_dir:
            return

        for parent in root.iter():
            for child in list(parent):
                if child.tag == "include" and "file" in child.attrib:
                    inc_path = child.attrib["file"]
                    if not os.path.isabs(inc_path):
                        inc_path = os.path.normpath(os.path.join(base_dir, inc_path))
                    if os.path.exists(inc_path):
                        inc_tree = ET.parse(inc_path)
                        inc_root = inc_tree.getroot()
                        cls.resolve_relative_paths(inc_root, os.path.dirname(inc_path))
                        idx = list(parent).index(child)
                        parent.remove(child)
                        if inc_root.tag == "mujoco":
                            for sub in list(inc_root):
                                parent.insert(idx, sub)
                                idx += 1
                        else:
                            parent.insert(idx, inc_root)

        meshdir = ""
        texturedir = ""
        for compiler in root.findall("compiler"):
            if "meshdir" in compiler.attrib:
                meshdir = compiler.attrib.pop("meshdir")
            if "texturedir" in compiler.attrib:
                texturedir = compiler.attrib.pop("texturedir")

        abs_meshdir = os.path.normpath(os.path.join(base_dir, meshdir)) if not os.path.isabs(meshdir) else meshdir
        abs_texdir = os.path.normpath(os.path.join(base_dir, texturedir)) if not os.path.isabs(texturedir) else texturedir

        for asset in root.findall("asset"):
            for elem in asset:
                if "file" in elem.attrib:
                    fpath = elem.attrib["file"]
                    if not os.path.isabs(fpath):
                        target_dir = abs_texdir if elem.tag == "texture" else abs_meshdir
                        elem.attrib["file"] = os.path.normpath(os.path.join(target_dir, fpath))

    @classmethod
    def load_robot_xml_root(cls, robot_path: str) -> ET.Element:
        """Loads either an MJCF (.xml) or URDF (.urdf) file into an MJCF ElementTree root with resolved asset paths."""
        if not robot_path or not os.path.exists(robot_path):
            return ET.fromstring(cls.DemoRobotXML())

        abs_robot_path = os.path.abspath(robot_path)
        base_dir = os.path.dirname(abs_robot_path)
        tree = ET.parse(abs_robot_path)
        root = tree.getroot()

        if root.tag == "robot":
            mj_ext = root.find("mujoco")
            if mj_ext is None:
                mj_ext = ET.SubElement(root, "mujoco")
            mj_comp = mj_ext.find("compiler")
            if mj_comp is None:
                mj_comp = ET.SubElement(mj_ext, "compiler")
            mj_comp.set("strippath", "false")
            mj_comp.set("balanceinertia", "false")
            mj_comp.set("discardvisual", "false")

            for mesh in root.iter("mesh"):
                if "filename" in mesh.attrib:
                    fn = mesh.attrib["filename"].replace("package://", "")
                    if not os.path.isabs(fn):
                        abs_fn = os.path.normpath(os.path.join(base_dir, fn))
                    else:
                        abs_fn = fn
                    if abs_fn.lower().endswith(".dae") and not os.path.exists(abs_fn):
                        stl_candidate = abs_fn[:-4] + ".stl"
                        if os.path.exists(stl_candidate):
                            abs_fn = stl_candidate
                    mesh.attrib["filename"] = abs_fn

            urdf_str = ET.tostring(root, encoding="unicode")
            tmp_model = mujoco.MjModel.from_xml_string(urdf_str)
            tmp_mjcf_path = os.path.join(base_dir, "_converted_temp_mjcf.xml")
            mujoco.mj_saveLastXML(tmp_mjcf_path, tmp_model)
            mjcf_root = ET.parse(tmp_mjcf_path).getroot()
            try:
                os.remove(tmp_mjcf_path)
            except OSError:
                pass
            cls.resolve_relative_paths(mjcf_root, base_dir)
            return mjcf_root
        else:
            cls.resolve_relative_paths(root, base_dir)
            return root

    @classmethod
    def ensure_white_skybox(cls, root: ET.Element):
        """Ensures the scene uses a pure white skybox background texture."""
        asset = root.find("asset")
        if asset is None:
            asset = ET.SubElement(root, "asset")

        for tex in list(asset.findall("texture")):
            if tex.attrib.get("type") == "skybox":
                asset.remove(tex)

        ET.SubElement(asset, "texture", {
            "name": "white_skybox",
            "type": "skybox",
            "builtin": "flat",
            "rgb1": "1 1 1",
            "rgb2": "1 1 1",
            "width": "64",
            "height": "64"
        })

    @staticmethod
    def _has_free_joint(robot_root: ET.Element) -> bool:
        """Checks whether the robot XML defines a floating-base free joint."""
        if robot_root.find(".//freejoint") is not None:
            return True
        for jnt in robot_root.findall(".//joint"):
            if jnt.attrib.get("type") == "free":
                return True
        return False

    @classmethod
    def build_combined_xml(cls, env_path: str = None, robot_path: str = None, timestep: float = 0.001) -> str:
        if env_path and os.path.exists(env_path):
            env_root = ET.parse(env_path).getroot()
            cls.resolve_relative_paths(env_root, os.path.dirname(os.path.abspath(env_path)))
        else:
            env_root = ET.fromstring(cls.DefaultEnvXML())

        robot_root = cls.load_robot_xml_root(robot_path)
        has_free = cls._has_free_joint(robot_root)

        # Merge non-worldbody sections
        merge_tags = ["asset", "default", "actuator", "sensor", "contact", "equality", "tendon", "keyframe"]
        for tag in merge_tags:
            for r_section in robot_root.findall(tag):
                e_section = env_root.find(tag)
                if e_section is None:
                    env_root.append(r_section)
                else:
                    for child in list(r_section):
                        e_section.append(child)

        # Merge worldbody; wrap fixed-base robot elements inside 'robot_base_mount' so the base can be translated freely
        e_wb = env_root.find("worldbody")
        if e_wb is None:
            e_wb = ET.SubElement(env_root, "worldbody")

        for r_wb in robot_root.findall("worldbody"):
            if has_free:
                for child in list(r_wb):
                    e_wb.append(child)
            else:
                mount_body = ET.SubElement(e_wb, "body", {"name": "robot_base_mount", "pos": "0 0 0"})
                for child in list(r_wb):
                    if child.tag in ("light", "camera"):
                        e_wb.append(child)
                    else:
                        mount_body.append(child)

        r_comp = robot_root.find("compiler")
        if r_comp is not None:
            e_comp = env_root.find("compiler")
            if e_comp is None:
                env_root.insert(0, r_comp)
            else:
                e_comp.attrib.update(r_comp.attrib)

        opt = env_root.find("option")
        if opt is None:
            opt = ET.SubElement(env_root, "option")
        opt.set("timestep", f"{timestep:.6f}")
        opt.set("integrator", "Euler")

        cls.ensure_white_skybox(env_root)
        return ET.tostring(env_root, encoding="unicode")


class RobotBasePosDialog(QDialog):
    """Popup dialog allowing the user to set the 3D position (X, Y, Z) of the robot base."""

    def __init__(self, current_base_pos: np.ndarray, on_change_callback, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Set Robot Base Position")
        self.resize(360, 220)
        self.base_pos = np.copy(current_base_pos)
        self.on_change_callback = on_change_callback
        self.spinboxes = []

        main_layout = QVBoxLayout(self)
        info_lbl = QLabel(
            "Set robot base 3D world coordinates (m).\n"
            "You can also hold Ctrl + Left/Right Mouse Drag in the 3D view."
        )
        main_layout.addWidget(info_lbl)

        form = QFormLayout()
        for idx, axis_name in enumerate(["X (m)", "Y (m)", "Z (m)"]):
            sb = QDoubleSpinBox()
            sb.setRange(-50.0, 50.0)
            sb.setDecimals(4)
            sb.setSingleStep(0.05)
            sb.setValue(float(self.base_pos[idx]))
            sb.valueChanged.connect(self._on_spinbox_changed)
            self.spinboxes.append(sb)
            form.addRow(f"Base {axis_name}:", sb)
        main_layout.addLayout(form)

        btn_row = QHBoxLayout()
        btn_apply = QPushButton("Apply")
        btn_reset = QPushButton("Reset (0, 0, 0)")
        btn_close = QPushButton("Close")

        btn_apply.clicked.connect(self._on_spinbox_changed)
        btn_reset.clicked.connect(self._reset_origin)
        btn_close.clicked.connect(self.accept)

        btn_row.addWidget(btn_apply)
        btn_row.addWidget(btn_reset)
        btn_row.addWidget(btn_close)
        main_layout.addLayout(btn_row)

    def _on_spinbox_changed(self):
        new_pos = np.array([float(sb.value()) for sb in self.spinboxes], dtype=np.float64)
        self.base_pos = new_pos
        if self.on_change_callback:
            self.on_change_callback(new_pos)

    def _reset_origin(self):
        for sb in self.spinboxes:
            sb.blockSignals(True)
            sb.setValue(0.0)
            sb.blockSignals(False)
        self._on_spinbox_changed()


class InitialJointDialog(QDialog):
    """Popup dialog allowing the user to modify initial joint positions (qpos) and joint polarities (+1 / -1)."""

    def __init__(self, model: mujoco.MjModel, current_init_qpos: np.ndarray,
                 current_qvel_polarity: np.ndarray, on_apply_callback, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Joint Pose & Polarity")
        self.resize(520, 540)
        self.model = model
        self.init_qpos = np.copy(current_init_qpos)
        self.qvel_polarity = np.copy(current_qvel_polarity)
        self.on_apply_callback = on_apply_callback
        self.pos_spinboxes = []
        self.polarity_checkboxes = []

        main_layout = QVBoxLayout(self)
        info_lbl = QLabel(
            "Configure initial joint coordinates and RDL-to-MuJoCo joint polarity.\n"
            "Checking 'Invert (-1)' flips both (q, qvel) sent to RDL and (tau) received from RDL."
        )
        main_layout.addWidget(info_lbl)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        container = QWidget()
        form = QFormLayout(container)

        for j in range(int(self.model.njnt)):
            jnt_name = mujoco.mj_id2name(self.model, int(mujoco.mjtObj.mjOBJ_JOINT), j) or f"joint_{j}"
            jnt_type = int(self.model.jnt_type[j])
            qpos_adr = int(self.model.jnt_qposadr[j])
            dof_adr = int(self.model.jnt_dofadr[j])
            axis_vec = np.round(self.model.jnt_axis[j], 2).tolist()

            if jnt_type == int(mujoco.mjtJoint.mjJNT_FREE):
                labels = ["x (m)", "y (m)", "z (m)", "qw", "qx", "qy", "qz"]
                for k, suffix in enumerate(labels):
                    idx = qpos_adr + k
                    sb = self._make_spinbox(float(self.init_qpos[idx]))
                    self.pos_spinboxes.append((idx, sb, 1.0 if k == 3 else 0.0))
                    form.addRow(f"[q_{idx}] {jnt_name}.{suffix}:", sb)
            elif jnt_type == int(mujoco.mjtJoint.mjJNT_BALL):
                labels = ["qw", "qx", "qy", "qz"]
                for k, suffix in enumerate(labels):
                    idx = qpos_adr + k
                    sb = self._make_spinbox(float(self.init_qpos[idx]))
                    self.pos_spinboxes.append((idx, sb, 1.0 if k == 0 else 0.0))
                    form.addRow(f"[q_{idx}] {jnt_name}.{suffix}:", sb)
            else:
                unit = "rad" if jnt_type == int(mujoco.mjtJoint.mjJNT_HINGE) else "m"
                row_widget = QWidget()
                row_layout = QHBoxLayout(row_widget)
                row_layout.setContentsMargins(0, 0, 0, 0)

                sb = self._make_spinbox(float(self.init_qpos[qpos_adr]))
                chk_inv = QCheckBox("Invert (-1)")
                chk_inv.setChecked(bool(self.qvel_polarity[dof_adr] < 0.0))

                row_layout.addWidget(sb, stretch=2)
                row_layout.addWidget(chk_inv, stretch=1)

                self.pos_spinboxes.append((qpos_adr, sb, 0.0))
                self.polarity_checkboxes.append((qpos_adr, dof_adr, chk_inv))
                form.addRow(f"[{dof_adr}] {jnt_name} ({unit}, axis={axis_vec}):", row_widget)

        scroll.setWidget(container)
        main_layout.addWidget(scroll)

        btn_row = QHBoxLayout()
        btn_zero = QPushButton("Zero Pos")
        btn_clear_pol = QPushButton("Reset Polarity")
        btn_apply = QPushButton("Apply")
        btn_close = QPushButton("Close")

        btn_zero.clicked.connect(self._reset_to_defaults)
        btn_clear_pol.clicked.connect(self._reset_polarities)
        btn_apply.clicked.connect(self._apply_changes)
        btn_close.clicked.connect(self.accept)

        btn_row.addWidget(btn_apply)
        btn_row.addWidget(btn_zero)
        btn_row.addWidget(btn_clear_pol)
        btn_row.addWidget(btn_close)
        main_layout.addLayout(btn_row)

    @staticmethod
    def _make_spinbox(val: float) -> QDoubleSpinBox:
        sb = QDoubleSpinBox()
        sb.setRange(-100.0, 100.0)
        sb.setDecimals(4)
        sb.setSingleStep(0.05)
        sb.setValue(float(val))
        return sb

    def _reset_to_defaults(self):
        for _, sb, default_val in self.pos_spinboxes:
            sb.setValue(float(default_val))

    def _reset_polarities(self):
        for _, _, chk in self.polarity_checkboxes:
            chk.setChecked(False)

    def _apply_changes(self):
        new_qpos = np.copy(self.init_qpos)
        for idx, sb, _ in self.pos_spinboxes:
            new_qpos[idx] = float(sb.value())

        new_qpos_pol = np.ones(int(self.model.nq), dtype=np.float64)
        new_qvel_pol = np.ones(int(self.model.nv), dtype=np.float64)
        for q_adr, v_adr, chk in self.polarity_checkboxes:
            sign = -1.0 if bool(chk.isChecked()) else 1.0
            new_qpos_pol[q_adr] = sign
            new_qvel_pol[v_adr] = sign

        self.init_qpos = new_qpos
        self.qvel_polarity = new_qvel_pol
        if self.on_apply_callback:
            self.on_apply_callback(new_qpos, new_qpos_pol, new_qvel_pol)


class RotorInertiaDialog(QDialog):
    """Popup dialog to configure reflected motor inertia (armature) for each DOF without modifying URDF/MJCF."""

    def __init__(self, model: mujoco.MjModel, current_inertia: np.ndarray,
                 txt_file_path: str, on_apply_callback, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Equivalent Motor Inertia (Armature)")
        self.resize(500, 520)
        self.model = model
        self.inertia_vec = np.copy(current_inertia)
        self.txt_file_path = txt_file_path
        self.on_apply_callback = on_apply_callback
        self.spinboxes = []

        main_layout = QVBoxLayout(self)
        info_lbl = QLabel(
            "Set reflected rotor inertia (I_a = N^2 * I_m) in sequential DOF order (default: 0.0).\n"
            "Click 'Save' to update MuJoCo and write values to TXT for RDL."
        )
        main_layout.addWidget(info_lbl)

        file_row = QHBoxLayout()
        file_row.addWidget(QLabel("TXT File:"))
        self.edt_txt_path = QLineEdit(self.txt_file_path)
        btn_load_txt = QPushButton("Load TXT")
        btn_load_txt.clicked.connect(self._load_from_txt_dialog)
        file_row.addWidget(self.edt_txt_path, stretch=1)
        file_row.addWidget(btn_load_txt)
        main_layout.addLayout(file_row)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        container = QWidget()
        form = QFormLayout(container)

        dof_labels = self._build_dof_labels()
        for dof_idx in range(int(self.model.nv)):
            sb = QDoubleSpinBox()
            sb.setRange(0.0, 1000.0)
            sb.setDecimals(6)
            sb.setSingleStep(0.01)
            sb.setValue(float(self.inertia_vec[dof_idx]))
            self.spinboxes.append((dof_idx, sb))
            form.addRow(f"DOF [{dof_idx}] {dof_labels[dof_idx]}:", sb)

        scroll.setWidget(container)
        main_layout.addWidget(scroll)

        btn_row = QHBoxLayout()
        btn_zero = QPushButton("Zero All")
        btn_apply_only = QPushButton("Apply")
        btn_apply_only.setDefault(True)
        btn_apply_only.setAutoDefault(True)
        btn_apply_save = QPushButton("Save")
        btn_close = QPushButton("Close")

        btn_zero.clicked.connect(self._set_all_zero)
        btn_apply_only.clicked.connect(lambda: self._apply_changes(save_txt=False))
        btn_apply_save.clicked.connect(lambda: self._apply_changes(save_txt=True))
        btn_close.clicked.connect(self.accept)

        btn_row.addWidget(btn_apply_only)
        btn_row.addWidget(btn_apply_save)
        btn_row.addWidget(btn_zero)
        btn_row.addWidget(btn_close)
        main_layout.addLayout(btn_row)

    def _build_dof_labels(self):
        labels = [f"dof_{i}" for i in range(int(self.model.nv))]
        for j in range(int(self.model.njnt)):
            jnt_name = mujoco.mj_id2name(self.model, int(mujoco.mjtObj.mjOBJ_JOINT), j) or f"joint_{j}"
            jnt_type = int(self.model.jnt_type[j])
            dof_adr = int(self.model.jnt_dofadr[j])
            if jnt_type == int(mujoco.mjtJoint.mjJNT_FREE):
                suffixes = ["vx", "vy", "vz", "wx", "wy", "wz"]
                for k, s in enumerate(suffixes):
                    if dof_adr + k < len(labels):
                        labels[dof_adr + k] = f"{jnt_name}.{s} (free)"
            elif jnt_type == int(mujoco.mjtJoint.mjJNT_BALL):
                for k, s in enumerate(["wx", "wy", "wz"]):
                    if dof_adr + k < len(labels):
                        labels[dof_adr + k] = f"{jnt_name}.{s} (ball)"
            elif jnt_type == int(mujoco.mjtJoint.mjJNT_SLIDE):
                if dof_adr < len(labels):
                    labels[dof_adr] = f"{jnt_name} (kg)"
            else:
                if dof_adr < len(labels):
                    labels[dof_adr] = f"{jnt_name} (kg*m^2)"
        return labels

    def _set_all_zero(self):
        for _, sb in self.spinboxes:
            sb.setValue(0.0)

    def _load_from_txt_dialog(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select Motor Inertia TXT File", self.edt_txt_path.text(), "Text Files (*.txt *.dat *.*)"
        )
        if not path:
            return
        self.edt_txt_path.setText(path)
        try:
            with open(path, "r", encoding="utf-8") as f:
                tokens = f.read().replace(",", " ").split()
            vals = [float(tok) for tok in tokens]
            for dof_idx, sb in self.spinboxes:
                sb.setValue(vals[dof_idx] if dof_idx < len(vals) else 0.0)
        except Exception as e:
            print(f"[Motor Inertia TXT Load Warning] {e}")

    def _apply_changes(self, save_txt: bool):
        new_inertia = np.zeros(int(self.model.nv), dtype=np.float64)
        for dof_idx, sb in self.spinboxes:
            new_inertia[dof_idx] = float(sb.value())

        txt_path = self.edt_txt_path.text().strip() or "rotor_inertia.txt"
        if save_txt:
            try:
                with open(txt_path, "w", encoding="utf-8") as f:
                    for val in new_inertia:
                        f.write(f"{val:.8f}\n")
            except Exception as e:
                print(f"[Motor Inertia TXT Save Error] {e}")

        self.inertia_vec = new_inertia
        if self.on_apply_callback:
            self.on_apply_callback(new_inertia, txt_path)


class RDLZmqBinaryNode:
    """ZeroMQ server node in MuJoCo supporting strict causal lockstep ping-pong synchronization."""

    def __init__(self, pub_endpoint="tcp://*:5555", sub_endpoint="tcp://*:5556"):
        self.ctx = zmq.Context()
        self.pub_sock = None
        self.sub_sock = None
        self.poller = zmq.Poller()
        self.pub_endpoint = pub_endpoint
        self.sub_endpoint = sub_endpoint
        self.configure_sockets(pub_endpoint, sub_endpoint)

    def configure_sockets(self, pub_endpoint: str, sub_endpoint: str):
        if self.pub_sock is not None:
            self.pub_sock.close()
        if self.sub_sock is not None:
            self.poller.unregister(self.sub_sock)
            self.sub_sock.close()

        self.pub_endpoint = pub_endpoint
        self.sub_endpoint = sub_endpoint

        self.pub_sock = self.ctx.socket(zmq.PUB)
        self.pub_sock.setsockopt(zmq.SNDHWM, 16)
        self.pub_sock.setsockopt(zmq.LINGER, 0)
        try:
            if "*" in pub_endpoint or "0.0.0.0" in pub_endpoint:
                self.pub_sock.bind(pub_endpoint)
            else:
                self.pub_sock.connect(pub_endpoint)
        except Exception as e:
            print(f"[ZMQ PUB Warning] Failed to bind/connect {pub_endpoint}: {e}")

        self.sub_sock = self.ctx.socket(zmq.SUB)
        self.sub_sock.setsockopt(zmq.RCVHWM, 16)
        self.sub_sock.setsockopt(zmq.LINGER, 0)
        self.sub_sock.setsockopt(zmq.SUBSCRIBE, b"")
        try:
            if "*" in sub_endpoint or "0.0.0.0" in sub_endpoint:
                self.sub_sock.bind(sub_endpoint)
            else:
                self.sub_sock.connect(sub_endpoint)
        except Exception as e:
            print(f"[ZMQ SUB Warning] Failed to bind/connect {sub_endpoint}: {e}")

        self.poller.register(self.sub_sock, zmq.POLLIN)

    def flush_cmd_queue(self):
        if self.sub_sock is None:
            return
        while True:
            try:
                self.sub_sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except Exception:
                break

    def receive_rdl_cmd(self, timeout_ms: int = 0):
        """Polls up to timeout_ms and unpacks the latest RdlCmdHeader + payload."""
        if self.sub_sock is None:
            return None

        if timeout_ms > 0:
            events = dict(self.poller.poll(timeout_ms))
            if self.sub_sock not in events:
                return None

        latest_msg = None
        while True:
            try:
                latest_msg = self.sub_sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except Exception as e:
                print(f"[ZMQ SUB Error] {e}")
                break

        if latest_msg is None or len(latest_msg) < RDL_CMD_HEADER_SIZE:
            return None

        magic, mode, nq, nv = struct.unpack_from(RDL_CMD_HEADER_FMT, latest_msg, 0)
        if magic != MAGIC_RDLM:
            return None

        payload_bytes = latest_msg[RDL_CMD_HEADER_SIZE:]
        payload_doubles = np.frombuffer(payload_bytes, dtype="<f8")

        return {
            "mode": mode,
            "nq": nq,
            "nv": nv,
            "payload": payload_doubles
        }

    def send_sim_state(self, model: mujoco.MjModel, data: mujoco.MjData,
                       qpos_polarity: np.ndarray, qvel_polarity: np.ndarray,
                       contacts_bin: bytes, ncon: int, timestamp_override: float = None):
        """Applies polarity mapping (q_rdl = P * q_mj) and sends SimState binary packet."""
        ts = float(data.time) if timestamp_override is None else float(timestamp_override)
        header_bin = struct.pack(
            SIM_HEADER_FMT,
            MAGIC_RDLM,
            model.nq,
            model.nv,
            ncon,
            ts
        )
        qpos_mapped = np.asarray(data.qpos * qpos_polarity, dtype="<f8").tobytes()
        qvel_mapped = np.asarray(data.qvel * qvel_polarity, dtype="<f8").tobytes()
        qfrc_mapped = np.asarray(data.qfrc_constraint * qvel_polarity, dtype="<f8").tobytes()

        full_packet = b"".join([header_bin, qpos_mapped, qvel_mapped, qfrc_mapped, contacts_bin])
        try:
            self.pub_sock.send(full_packet, flags=zmq.NOBLOCK)
        except zmq.Again:
            pass


class MuJoCoViewport(QOpenGLWidget):
    """Thread-safe, VSync-unlocked MuJoCo OpenGL viewport supporting camera control and Ctrl+Mouse robot dragging."""

    def __init__(self, sim_lock: threading.Lock, on_drag_robot_callback=None, parent=None):
        super().__init__(parent)
        fmt = QSurfaceFormat()
        fmt.setSwapInterval(0)
        self.setFormat(fmt)

        self.sim_lock = sim_lock
        self.on_drag_robot_callback = on_drag_robot_callback
        self.model = None
        self.data = None

        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.pert = mujoco.MjvPerturb()
        self.scn = None
        self.ctx = None
        self._need_reinit_gl = False

        self.last_x = 0.0
        self.last_y = 0.0
        self.SetDefaultCamera()
        self.setFocusPolicy(Qt.StrongFocus)

    def SetDefaultCamera(self):
        mujoco.mjv_defaultCamera(self.cam)
        mujoco.mjv_defaultOption(self.opt)
        mujoco.mjv_defaultPerturb(self.pert)
        self.cam.type = int(mujoco.mjtCamera.mjCAMERA_FREE)
        self.cam.distance = 2.2
        self.cam.azimuth = 135.0
        self.cam.elevation = -20.0
        self.cam.lookat[:] = [0.0, 0.0, 0.45]

        # Enable subtle contact point and contact force display; disable convex hull overlay so robot mesh stands out
        self.opt.flags[int(mujoco.mjtVisFlag.mjVIS_CONTACTPOINT)] = 1
        self.opt.flags[int(mujoco.mjtVisFlag.mjVIS_CONTACTFORCE)] = 1
        self.opt.flags[int(mujoco.mjtVisFlag.mjVIS_CONVEXHULL)] = 0

    def bind_model(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        self._need_reinit_gl = True
        self.update()

    def _rebuild_gl_resources(self):
        if self.model is not None:
            self.scn = mujoco.MjvScene(self.model, maxgeom=2000)
            self.ctx = mujoco.MjrContext(self.model, int(mujoco.mjtFontScale.mjFONTSCALE_150))
            self._need_reinit_gl = False

    def initializeGL(self):
        glClearColor(1.0, 1.0, 1.0, 1.0)
        with self.sim_lock:
            self._rebuild_gl_resources()

    def paintGL(self):
        glClearColor(1.0, 1.0, 1.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        with self.sim_lock:
            if self.model is None or self.data is None:
                return
            if self._need_reinit_gl or self.scn is None or self.ctx is None:
                self._rebuild_gl_resources()
            mujoco.mjv_updateScene(
                self.model, self.data, self.opt, self.pert,
                self.cam, int(mujoco.mjtCatBit.mjCAT_ALL), self.scn
            )

        dpr = self.devicePixelRatio()
        w = max(1, int(self.width() * dpr))
        h = max(1, int(self.height() * dpr))
        viewport = mujoco.MjrRect(0, 0, w, h)
        mujoco.mjr_render(viewport, self.scn, self.ctx)

    def mousePressEvent(self, event):
        self.last_x = float(event.pos().x())
        self.last_y = float(event.pos().y())
        event.accept()

    def mouseMoveEvent(self, event):
        if self.model is None:
            return

        curr_x = float(event.pos().x())
        curr_y = float(event.pos().y())
        dx = curr_x - self.last_x
        dy = curr_y - self.last_y
        self.last_x = curr_x
        self.last_y = curr_y

        buttons = event.buttons()
        ctrl_pressed = bool(event.modifiers() & Qt.ControlModifier)

        az_rad = np.deg2rad(self.cam.azimuth)
        el_rad = np.deg2rad(self.cam.elevation)
        right = np.array([np.sin(az_rad), -np.cos(az_rad), 0.0])
        forward_horiz = np.array([np.cos(az_rad), np.sin(az_rad), 0.0])
        scale = self.cam.distance * 0.0018

        # Hold Ctrl + Mouse Drag to interactively drag the robot base position in 3D space
        if ctrl_pressed and self.on_drag_robot_callback is not None:
            if bool(buttons & Qt.LeftButton):
                # Ctrl + Left Drag: translate robot horizontally in XY plane
                delta_pos = (dx * right - dy * forward_horiz) * scale
                self.on_drag_robot_callback(delta_pos)
            elif bool(buttons & Qt.RightButton):
                # Ctrl + Right Drag: translate robot along vertical Z axis and horizontal right
                up_z = np.array([0.0, 0.0, 1.0])
                delta_pos = (dx * right - dy * up_z) * scale
                self.on_drag_robot_callback(delta_pos)
            self.update()
            event.accept()
            return

        # Standard camera manipulation when Ctrl is not held
        if bool(buttons & Qt.LeftButton):
            self.cam.azimuth = (self.cam.azimuth - dx * 0.4) % 360.0
            self.cam.elevation = float(np.clip(self.cam.elevation - dy * 0.4, -89.0, 89.0))
        elif bool(buttons & Qt.RightButton):
            forward = np.array([
                np.cos(el_rad) * np.cos(az_rad),
                np.cos(el_rad) * np.sin(az_rad),
                np.sin(el_rad)
            ])
            up = np.cross(right, forward)
            self.cam.lookat[:] += (-dx * right + dy * up) * scale
        elif bool(buttons & Qt.MiddleButton):
            self.cam.distance = float(max(0.05, self.cam.distance * (1.0 + dy * 0.01)))
        else:
            return

        self.update()
        event.accept()

    def wheelEvent(self, event):
        if self.model is None:
            return
        steps = event.angleDelta().y() / 120.0
        # Hold Ctrl + Mouse Wheel to adjust robot Z height directly
        if bool(event.modifiers() & Qt.ControlModifier) and self.on_drag_robot_callback is not None:
            self.on_drag_robot_callback(np.array([0.0, 0.0, 0.02 * steps]))
        else:
            self.cam.distance = float(max(0.05, self.cam.distance * (0.9 ** steps)))
        self.update()
        event.accept()


class RobotSimulationSystem(QMainWindow):
    """Main window: executes causal lockstep synchronization and supports interactive robot base positioning."""

    def __init__(self, robot_xml_path: str = None, env_xml_path: str = None, inertia_txt_path: str = "rotor_inertia.txt"):
        super().__init__()
        self.setWindowTitle("MuJoCo Real-Time Robot Simulator")
        self.resize(1380, 860)

        self.sim_lock = threading.Lock()
        self.env_xml_path = env_xml_path
        self.robot_xml_path = robot_xml_path
        self.inertia_txt_path = inertia_txt_path
        self.model = None
        self.data = None
        self.init_qpos = None
        self.qpos_polarity = None
        self.qvel_polarity = None
        self.rotor_inertia = None

        # Robot base position state
        self.robot_base_pos = np.zeros(3, dtype=np.float64)
        self.mount_body_id = -1
        self.free_jnt_qpos_adr = -1

        # Simulation timing parameters
        self.sim_timestep = 0.001
        self.sim_duration = 10.0
        self.sim_finished = False

        self.t0_mj_grav = None
        self.t0_rdl_tau = None
        self.polarity_warning_str = "Not checked yet (Waiting for RDL)"

        self.measured_phys_hz = 0.0
        self.measured_rtf = 0.0
        self.missed_sync_count = 0
        self.gui_tick_count = 0

        self.is_paused = False
        self.rdl_connected = False
        self.last_cmd_mode = "Waiting for RDL..."
        self._running = True
        self._need_net_reconfig = None

        self.rdl_net = RDLZmqBinaryNode(pub_endpoint="tcp://*:5555", sub_endpoint="tcp://*:5556")
        self._init_ui()
        self.reload_simulation()

        # Start dedicated lockstep physics thread
        self.physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self.physics_thread.start()

        # 30Hz GUI timer (33ms): renders 3D scene at 30Hz and refreshes text panel at 10Hz
        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self._gui_refresh_step)
        self.render_timer.start(33)

    def closeEvent(self, event):
        self._running = False
        if self.physics_thread.is_alive():
            self.physics_thread.join(timeout=0.5)
        super().closeEvent(event)

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        self.viewport = MuJoCoViewport(self.sim_lock, on_drag_robot_callback=self.translate_robot_base_by_delta, parent=self)
        main_layout.addWidget(self.viewport, stretch=3)

        panel = QVBoxLayout()
        main_layout.addLayout(panel, stretch=1)

        # 1. Model & Robot Setup Group
        model_group = QGroupBox("Model & Setup")
        model_layout = QVBoxLayout(model_group)

        env_label_str = os.path.basename(self.env_xml_path) if self.env_xml_path else "Default Ground"
        robot_label_str = os.path.basename(self.robot_xml_path) if self.robot_xml_path else "2-Link Demo"
        self.lbl_env = QLabel(f"Env: {env_label_str}")
        self.lbl_robot = QLabel(f"Robot: {robot_label_str}")
        lbl_drag_tip = QLabel("Tip: Hold Ctrl + Mouse Drag/Wheel to move robot")
        lbl_drag_tip.setStyleSheet("color: #555555; font-size: 11px;")

        env_btn_row = QHBoxLayout()
        btn_load_env = QPushButton("Load Env")
        btn_reset_env = QPushButton("Default Env")
        env_btn_row.addWidget(btn_load_env)
        env_btn_row.addWidget(btn_reset_env)

        robot_btn_row = QHBoxLayout()
        btn_load_robot = QPushButton("Load Robot")
        btn_base_pos = QPushButton("Robot Base Pos")
        btn_base_pos.setStyleSheet("font-weight: bold; padding: 4px;")
        robot_btn_row.addWidget(btn_load_robot)
        robot_btn_row.addWidget(btn_base_pos)

        btn_edit_qpos = QPushButton("Joint Pos. and Polarity")
        btn_edit_qpos.setStyleSheet("font-weight: bold; padding: 4px;")
        btn_edit_inertia = QPushButton("Motor Inertia")
        btn_edit_inertia.setStyleSheet("font-weight: bold; padding: 4px;")

        ctrl_btn_row = QHBoxLayout()
        btn_reset_sim = QPushButton("Reset Sim")
        self.btn_pause = QPushButton("Pause / Resume")
        ctrl_btn_row.addWidget(btn_reset_sim)
        ctrl_btn_row.addWidget(self.btn_pause)

        btn_load_env.clicked.connect(self.select_env_xml)
        btn_reset_env.clicked.connect(self.reset_default_env)
        btn_load_robot.clicked.connect(self.select_robot_xml)
        btn_base_pos.clicked.connect(self.open_robot_base_pos_dialog)
        btn_edit_qpos.clicked.connect(self.open_initial_joint_dialog)
        btn_edit_inertia.clicked.connect(self.open_rotor_inertia_dialog)
        btn_reset_sim.clicked.connect(self.reset_physics_state)
        self.btn_pause.clicked.connect(self.toggle_pause)

        model_layout.addWidget(self.lbl_env)
        model_layout.addLayout(env_btn_row)
        model_layout.addSpacing(4)
        model_layout.addWidget(self.lbl_robot)
        model_layout.addLayout(robot_btn_row)
        model_layout.addWidget(lbl_drag_tip)
        model_layout.addSpacing(4)
        model_layout.addWidget(btn_edit_qpos)
        model_layout.addWidget(btn_edit_inertia)
        model_layout.addLayout(ctrl_btn_row)
        panel.addWidget(model_group)

        # 2. Simulation Timing Group (Timestep & Duration)
        time_group = QGroupBox("Simulation Timing")
        time_form = QFormLayout(time_group)

        self.spn_timestep = QDoubleSpinBox()
        self.spn_timestep.setRange(0.0001, 0.0500)
        self.spn_timestep.setDecimals(4)
        self.spn_timestep.setSingleStep(0.0005)
        self.spn_timestep.setValue(self.sim_timestep)

        self.spn_duration = QDoubleSpinBox()
        self.spn_duration.setRange(0.10, 100000.0)
        self.spn_duration.setDecimals(2)
        self.spn_duration.setSingleStep(1.0)
        self.spn_duration.setValue(self.sim_duration)

        btn_apply_time = QPushButton("Apply Timing")
        btn_apply_time.clicked.connect(self.apply_timing_config)

        time_form.addRow("Timestep (s):", self.spn_timestep)
        time_form.addRow("Duration (s):", self.spn_duration)
        time_form.addRow(btn_apply_time)
        panel.addWidget(time_group)

        # 3. RDL ZeroMQ Binary Interface Group
        net_group = QGroupBox("ZMQ Endpoints")
        net_form = QFormLayout(net_group)
        self.edt_pub_ep = QLineEdit("tcp://*:5555")
        self.edt_sub_ep = QLineEdit("tcp://*:5556")
        btn_apply_net = QPushButton("Apply ZMQ")
        btn_apply_net.clicked.connect(self.apply_network_config)
        net_form.addRow("PUB Bind:", self.edt_pub_ep)
        net_form.addRow("SUB Bind:", self.edt_sub_ep)
        net_form.addRow(btn_apply_net)
        panel.addWidget(net_group)

        # 4. Real-time Dynamics, Polarity Diagnostics & Collision Status Monitor
        col_group = QGroupBox("Status & Collision Monitor")
        col_layout = QVBoxLayout(col_group)
        self.txt_collision = QTextEdit()
        self.txt_collision.setReadOnly(True)
        col_layout.addWidget(self.txt_collision)
        panel.addWidget(col_group, stretch=1)

    def _apply_subtle_contact_visuals_unlocked(self):
        """Weakens the visual prominence of contact points and force arrows so the robot stands out clearly."""
        if self.model is None:
            return
        self.model.vis.scale.contactwidth = 0.04
        self.model.vis.scale.contactheight = 0.015
        self.model.vis.scale.forcewidth = 0.015
        self.model.vis.map.force = 0.005
        self.model.vis.rgba.contactpoint[:] = [0.95, 0.55, 0.25, 0.40]
        self.model.vis.rgba.contactforce[:] = [0.25, 0.65, 0.95, 0.35]

    def _locate_robot_base_handle_unlocked(self):
        """Identifies whether the robot base is controlled via 'robot_base_mount' body or a floating-base free joint."""
        self.mount_body_id = -1
        self.free_jnt_qpos_adr = -1
        if self.model is None:
            return

        mount_id = mujoco.mj_name2id(self.model, int(mujoco.mjtObj.mjOBJ_BODY), "robot_base_mount")
        if mount_id >= 0:
            self.mount_body_id = int(mount_id)
            self.robot_base_pos = np.copy(self.model.body_pos[self.mount_body_id])
            return

        for j in range(int(self.model.njnt)):
            if int(self.model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE):
                self.free_jnt_qpos_adr = int(self.model.jnt_qposadr[j])
                self.robot_base_pos = np.copy(self.init_qpos[self.free_jnt_qpos_adr:self.free_jnt_qpos_adr + 3])
                return

    def _set_robot_base_pos_unlocked(self, new_pos: np.ndarray):
        """Updates the robot base 3D position in the model/state and refreshes forward kinematics."""
        self.robot_base_pos = np.copy(new_pos)
        if self.model is None or self.data is None:
            return

        if self.mount_body_id >= 0:
            self.model.body_pos[self.mount_body_id] = self.robot_base_pos
        elif self.free_jnt_qpos_adr >= 0:
            adr = self.free_jnt_qpos_adr
            self.init_qpos[adr:adr + 3] = self.robot_base_pos
            if not self.rdl_connected or self.sim_finished:
                self.data.qpos[adr:adr + 3] = self.robot_base_pos

        mujoco.mj_forward(self.model, self.data)

    def translate_robot_base_by_delta(self, delta_pos: np.ndarray):
        """Callback invoked by Ctrl + Mouse Drag/Wheel in the 3D viewport."""
        with self.sim_lock:
            if self.model is None:
                return
            self._set_robot_base_pos_unlocked(self.robot_base_pos + delta_pos)

    def open_robot_base_pos_dialog(self):
        if self.model is None:
            return
        dlg = RobotBasePosDialog(self.robot_base_pos, self.apply_new_robot_base_pos, self)
        dlg.exec_()

    def apply_new_robot_base_pos(self, new_pos: np.ndarray):
        with self.sim_lock:
            self._set_robot_base_pos_unlocked(new_pos)
        self.viewport.update()

    def apply_timing_config(self):
        with self.sim_lock:
            self.sim_timestep = float(self.spn_timestep.value())
            self.sim_duration = float(self.spn_duration.value())
            if self.model is not None and self.data is not None:
                self.model.opt.timestep = self.sim_timestep
                self._reset_physics_state_unlocked()
        self.viewport.update()

    def apply_network_config(self):
        pub_ep = self.edt_pub_ep.text().strip()
        sub_ep = self.edt_sub_ep.text().strip()
        self._need_net_reconfig = (pub_ep, sub_ep)

    def select_env_xml(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select Environment XML File", "", "XML Files (*.xml)")
        if path:
            self.env_xml_path = path
            self.lbl_env.setText(f"Env: {os.path.basename(path)}")
            self.reload_simulation()

    def reset_default_env(self):
        self.env_xml_path = None
        self.lbl_env.setText("Env: Default Ground")
        self.reload_simulation()

    def select_robot_xml(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select Robot XML or URDF File", "", "Robot Models (*.xml *.urdf)"
        )
        if path:
            self.robot_xml_path = path
            self.lbl_robot.setText(f"Robot: {os.path.basename(path)}")
            self.reload_simulation()

    def reload_simulation(self):
        with self.sim_lock:
            try:
                if not self.robot_xml_path and not os.path.exists("demo_robot.xml"):
                    with open("demo_robot.xml", "w", encoding="utf-8") as f:
                        f.write(XMLModelMerger.DemoRobotXML().strip())

                combined_xml = XMLModelMerger.build_combined_xml(
                    self.env_xml_path, self.robot_xml_path, timestep=self.sim_timestep
                )
                self.model = mujoco.MjModel.from_xml_string(combined_xml)
                self.model.opt.timestep = self.sim_timestep
                self._apply_subtle_contact_visuals_unlocked()
                self.data = mujoco.MjData(self.model)

                if not self.robot_xml_path and self.model.nq == 2:
                    self.data.qpos[0] = 0.4
                    self.data.qpos[1] = -0.6

                self.init_qpos = np.copy(self.data.qpos)
                self.qpos_polarity = np.ones(int(self.model.nq), dtype=np.float64)
                self.qvel_polarity = np.ones(int(self.model.nv), dtype=np.float64)

                # Default equivalent motor inertia (armature) is 0.0 for all DOFs
                self.rotor_inertia = np.zeros(int(self.model.nv), dtype=np.float64)
                self.model.dof_armature[:] = self.rotor_inertia
                mujoco.mj_setConst(self.model, self.data)

                self._locate_robot_base_handle_unlocked()
                self._reset_physics_state_unlocked()
                self.viewport.bind_model(self.model, self.data)
            except Exception as e:
                self.model = None
                self.data = None
                self.txt_collision.setPlainText(f"[Model Load Error]\nCheck XML/URDF or relative STL paths:\n{e}")

    def open_initial_joint_dialog(self):
        if self.model is None or self.init_qpos is None:
            return
        dlg = InitialJointDialog(
            self.model, self.init_qpos, self.qvel_polarity,
            self.apply_new_initial_pose_and_polarity, self
        )
        dlg.exec_()

    def apply_new_initial_pose_and_polarity(self, new_qpos: np.ndarray,
                                            new_qpos_pol: np.ndarray,
                                            new_qvel_pol: np.ndarray):
        with self.sim_lock:
            self.init_qpos = np.copy(new_qpos)
            if self.free_jnt_qpos_adr >= 0:
                adr = self.free_jnt_qpos_adr
                self.robot_base_pos = np.copy(self.init_qpos[adr:adr + 3])
            self.qpos_polarity = np.copy(new_qpos_pol)
            self.qvel_polarity = np.copy(new_qvel_pol)
            self._reset_physics_state_unlocked()
        self.viewport.update()

    def open_rotor_inertia_dialog(self):
        if self.model is None or self.rotor_inertia is None:
            return
        dlg = RotorInertiaDialog(
            self.model, self.rotor_inertia, self.inertia_txt_path,
            self.apply_new_rotor_inertia, self
        )
        dlg.exec_()

    def apply_new_rotor_inertia(self, new_inertia: np.ndarray, txt_path: str):
        """Injects equivalent motor inertia into MuJoCo model.dof_armature without modifying URDF/MJCF."""
        with self.sim_lock:
            self.rotor_inertia = np.copy(new_inertia)
            self.inertia_txt_path = txt_path
            if self.model is not None and self.data is not None:
                self.model.dof_armature[:] = self.rotor_inertia
                mujoco.mj_setConst(self.model, self.data)
                self._reset_physics_state_unlocked()
        self.viewport.update()

    def check_t0_polarity_mismatch(self, mj_grav: np.ndarray, rdl_mapped_tau: np.ndarray):
        self.t0_mj_grav = np.copy(mj_grav)
        self.t0_rdl_tau = np.copy(rdl_mapped_tau)
        inverted_dofs = []

        for i in range(int(self.model.nv)):
            if abs(mj_grav[i]) > 0.05 and abs(rdl_mapped_tau[i]) > 0.05:
                if mj_grav[i] * rdl_mapped_tau[i] < 0.0:
                    inverted_dofs.append(i)

        if inverted_dofs:
            self.polarity_warning_str = f"MISMATCH on DOFs {inverted_dofs}"
        else:
            self.polarity_warning_str = "OK (Signs match on active DOFs)"

    def _hold_initial_pose_unlocked(self):
        """Immediately sets the robot to its initial joint positions with zero velocity and zero control torque."""
        if self.model and self.data:
            self.data.qpos[:] = self.init_qpos
            self.data.qvel[:] = 0.0
            self.data.qacc[:] = 0.0
            self.data.qfrc_applied[:] = 0.0
            if self.model.nu > 0:
                self.data.ctrl[:] = 0.0
            self.data.time = 0.0
            mujoco.mj_forward(self.model, self.data)

    def _reset_physics_state_unlocked(self):
        if self.model and self.data:
            mujoco.mj_resetData(self.model, self.data)
            self._hold_initial_pose_unlocked()
            self.rdl_connected = False
            self.sim_finished = False
            self.missed_sync_count = 0
            self.last_cmd_mode = "Waiting for RDL..."
            self.polarity_warning_str = "Waiting for RDL t=0 snapshot..."
            self.rdl_net.flush_cmd_queue()

    def reset_physics_state(self):
        with self.sim_lock:
            self._reset_physics_state_unlocked()
        self.viewport.update()

    def toggle_pause(self):
        self.is_paused = not self.is_paused

    def pack_collisions_binary_fast(self) -> bytes:
        """Fast binary-only contact packer for the physics thread."""
        ncon = int(self.data.ncon)
        if ncon == 0:
            return b""

        binary_chunks = []
        wrench_local = np.zeros(6, dtype=np.float64)

        for i in range(ncon):
            con = self.data.contact[i]
            pos = con.pos
            rot_mat = con.frame.reshape(3, 3)
            normal = rot_mat[0]

            mujoco.mj_contactForce(self.model, self.data, i, wrench_local)
            force_world = rot_mat.T @ wrench_local[:3]

            binary_chunks.append(struct.pack(
                CONTACT_ITEM_FMT,
                int(con.geom1), int(con.geom2), float(con.dist),
                float(pos[0]), float(pos[1]), float(pos[2]),
                float(normal[0]), float(normal[1]), float(normal[2]),
                float(wrench_local[0]), float(wrench_local[1]), float(wrench_local[2]),
                float(wrench_local[3]), float(wrench_local[4]), float(wrench_local[5]),
                float(force_world[0]), float(force_world[1]), float(force_world[2])
            ))

        return b"".join(binary_chunks)

    def extract_contacts_for_ui(self):
        """Extracts human-readable contact info at 10Hz when refreshing the UI text panel."""
        contacts_ui_list = []
        wrench_local = np.zeros(6, dtype=np.float64)
        ncon = min(5, int(self.data.ncon))

        for i in range(ncon):
            con = self.data.contact[i]
            g1_id = int(con.geom1)
            g2_id = int(con.geom2)
            rot_mat = con.frame.reshape(3, 3)
            mujoco.mj_contactForce(self.model, self.data, i, wrench_local)
            force_world = rot_mat.T @ wrench_local[:3]

            g1_name = mujoco.mj_id2name(self.model, int(mujoco.mjtObj.mjOBJ_GEOM), g1_id) or f"geom_{g1_id}"
            g2_name = mujoco.mj_id2name(self.model, int(mujoco.mjtObj.mjOBJ_GEOM), g2_id) or f"geom_{g2_id}"
            contacts_ui_list.append({
                "pair": f"{g1_name} <-> {g2_name}",
                "dist": round(float(con.dist), 5),
                "pos": np.round(con.pos, 3).tolist(),
                "fn": round(float(wrench_local[0]), 2),
                "f_world": np.round(force_world, 2).tolist()
            })
        return contacts_ui_list

    def apply_rdl_command(self, cmd: dict, is_first_handshake_frame: bool):
        """Applies unpacked RdlCmd with polarity mapping: tau_mj = P * tau_rdl."""
        mode = cmd["mode"]
        payload = cmd["payload"]

        if mode == 0:
            self.last_cmd_mode = "0 (Computed Torque / M*PID + Grav)"
            if len(payload) >= self.model.nv and self.model.nv > 0:
                mapped_tau = payload[:self.model.nv] * self.qvel_polarity
                if is_first_handshake_frame:
                    self.check_t0_polarity_mismatch(self.data.qfrc_bias, mapped_tau)
                self.data.qfrc_applied[:] = mapped_tau
            elif len(payload) >= self.model.nu and self.model.nu > 0:
                self.data.ctrl[:] = payload[:self.model.nu]
        elif mode == 1:
            self.last_cmd_mode = "1 (Direct State Overwrite)"
            nq = min(cmd["nq"], self.model.nq)
            nv = min(cmd["nv"], self.model.nv)
            if len(payload) >= nq + nv:
                self.data.qpos[:nq] = payload[:nq] * self.qpos_polarity[:nq]
                self.data.qvel[:nv] = payload[nq:nq + nv] * self.qvel_polarity[:nv]
                mujoco.mj_forward(self.model, self.data)

    def _physics_loop(self):
        """Causal lockstep loop that stops and holds initial pose upon reaching sim_duration."""
        step_count = 0
        stat_wall_t0 = time.perf_counter()
        stat_sim_t0 = 0.0
        wall_origin = time.perf_counter()
        sim_origin = 0.0

        while self._running:
            if self._need_net_reconfig is not None:
                pub_ep, sub_ep = self._need_net_reconfig
                self._need_net_reconfig = None
                self.rdl_net.configure_sockets(pub_ep, sub_ep)
                with self.sim_lock:
                    self._reset_physics_state_unlocked()

            with self.sim_lock:
                if self.model is None or self.data is None:
                    time.sleep(0.01)
                    continue

                # Once duration is reached, strictly hold initial pose until user clicks Reset Sim / Apply Timing
                if self.sim_finished:
                    self._hold_initial_pose_unlocked()
                    self.measured_phys_hz = 0.0
                    self.measured_rtf = 0.0
                    self.rdl_net.flush_cmd_queue()

            if self.sim_finished:
                time.sleep(0.005)
                continue

            with self.sim_lock:
                if not self.rdl_connected:
                    self._hold_initial_pose_unlocked()

                ncon = int(self.data.ncon)
                contacts_bin = self.pack_collisions_binary_fast()

                # CAUSAL BARRIER: Flush any stale command in SUB buffer before sending current state x(k)
                self.rdl_net.flush_cmd_queue()

                # STEP 1: Send state x(k) to RDL
                ts_override = 0.0 if not self.rdl_connected else None
                self.rdl_net.send_sim_state(
                    self.model, self.data, self.qpos_polarity, self.qvel_polarity,
                    contacts_bin, ncon, timestamp_override=ts_override
                )

            # STEP 2: Wait exclusively for RDL's fresh torque tau(k) responding to x(k)
            wait_ms = 20 if self.rdl_connected else 2
            cmd = self.rdl_net.receive_rdl_cmd(timeout_ms=wait_ms)

            with self.sim_lock:
                if self.model is None or self.data is None or self.sim_finished:
                    continue

                if cmd is not None:
                    is_first_frame = not self.rdl_connected
                    if is_first_frame:
                        wall_origin = time.perf_counter()
                        sim_origin = float(self.data.time)
                        stat_wall_t0 = wall_origin
                        stat_sim_t0 = sim_origin
                        step_count = 0
                    self.rdl_connected = True
                    self.missed_sync_count = 0
                    self.apply_rdl_command(cmd, is_first_handshake_frame=is_first_frame)

                    # STEP 3: Step physics (skip stepping on the initial t=0 handshake frame so q_init is exact)
                    if not self.is_paused and not is_first_frame:
                        mujoco.mj_step(self.model, self.data)
                        mujoco.mj_forward(self.model, self.data)
                        step_count += 1

                        if float(self.data.time) >= self.sim_duration - 1e-9:
                            self.sim_finished = True
                            self.rdl_connected = False
                            self.last_cmd_mode = f"Finished ({self.sim_duration:.2f}s) - Holding Init Pose"
                            self._hold_initial_pose_unlocked()
                    else:
                        wall_origin = time.perf_counter()
                        sim_origin = float(self.data.time)
                else:
                    if self.rdl_connected:
                        self.missed_sync_count += 1
                        if self.missed_sync_count > 25:
                            self.rdl_connected = False
                            self.last_cmd_mode = "RDL Disconnected (Holding Pose)"

                curr_sim_time = float(self.data.time)

            # Real-time pacing with automatic catch-up
            if self.rdl_connected and not self.is_paused and not self.sim_finished:
                target_wall = wall_origin + (curr_sim_time - sim_origin)
                rem = target_wall - time.perf_counter()
                if rem > 0.0012:
                    time.sleep(0.0005)
                elif rem < -0.10:
                    wall_origin = time.perf_counter()
                    sim_origin = curr_sim_time

            elapsed_wall = time.perf_counter() - stat_wall_t0
            if elapsed_wall >= 0.25:
                self.measured_phys_hz = step_count / elapsed_wall
                self.measured_rtf = (curr_sim_time - stat_sim_t0) / elapsed_wall if self.rdl_connected else 0.0
                step_count = 0
                stat_wall_t0 = time.perf_counter()
                stat_sim_t0 = curr_sim_time

    def _gui_refresh_step(self):
        """Runs on Qt main thread at 30Hz for OpenGL viewport and 10Hz for text panel."""
        self.viewport.update()

        self.gui_tick_count += 1
        if self.gui_tick_count % 3 != 0:
            return

        with self.sim_lock:
            if self.model is None or self.data is None:
                return
            sim_time = float(self.data.time)
            dt_val = float(self.model.opt.timestep)
            dur_val = float(self.sim_duration)
            nq = int(self.model.nq)
            nv = int(self.model.nv)
            base_pos_list = np.round(self.robot_base_pos, 3).tolist()
            pol_list = self.qvel_polarity[:6].astype(int).tolist()
            armature_list = np.round(self.model.dof_armature[:6], 4).tolist()
            init_q = np.round(self.init_qpos[:6], 3).tolist()
            curr_q = np.round(self.data.qpos[:6], 3).tolist()
            mj_bias = np.round(self.data.qfrc_bias[:6], 2).tolist()
            rdl_tau = np.round(self.data.qfrc_applied[:6], 2).tolist()
            ncon = int(self.data.ncon)
            contacts_ui = self.extract_contacts_for_ui()

        if self.sim_finished:
            status_str = f"FINISHED ({dur_val:.2f}s Reached - Holding Init Pose)"
        elif self.rdl_connected:
            status_str = "CONNECTED (Lockstep Active)"
        else:
            status_str = "WAITING FOR RDL (Holding Init Pose)"

        t0_mj_str = np.round(self.t0_mj_grav[:6], 2).tolist() if self.t0_mj_grav is not None else "N/A"
        t0_rdl_str = np.round(self.t0_rdl_tau[:6], 2).tolist() if self.t0_rdl_tau is not None else "N/A"

        log_lines = [
            f"RDL Status:   {status_str}",
            f"Lockstep Hz:  {self.measured_phys_hz:.1f} Hz | RTF: {self.measured_rtf:.2f}x",
            f"Sim Time:     {sim_time:.3f} / {dur_val:.2f} s (dt={dt_val:.4f}s)",
            f"Base Pos (m): {base_pos_list} | nq={nq}, nv={nv}",
            f"Armature Vec: {armature_list}",
            f"Polarity Vec: {pol_list}",
            f"Polarity Chk: {self.polarity_warning_str}",
            f"t=0 MJ Grav:  {t0_mj_str}",
            f"t=0 RDL Tau:  {t0_rdl_str}",
            "-" * 44,
            f"Init qpos:    {init_q}",
            f"Curr qpos:    {curr_q}",
            f"Live MJ Bias: {mj_bias}",
            f"Live RDL Tau: {rdl_tau}",
            f"Active Contacts: {ncon}",
            "-" * 44
        ]
        for idx, c in enumerate(contacts_ui):
            log_lines.append(
                f"[{idx+1}] {c['pair']}\n"
                f"    Pos: {c['pos']} | Dist: {c['dist']} m\n"
                f"    Normal Fn: {c['fn']} N | World Force: {c['f_world']} N"
            )
        self.txt_collision.setPlainText("\n".join(log_lines))


def parse_args():
    parser = argparse.ArgumentParser(description="MuJoCo Real-Time Robot Simulator with RDL Interface")
    parser.add_argument("robot_model", nargs="?", default=None, help="Path to robot model file (.xml or .urdf)")
    parser.add_argument("--env", default=None, help="Path to environment XML file (optional)")
    parser.add_argument("--inertia-txt", default="rotor_inertia.txt", help="Path to sequential motor inertia TXT file")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    app = QApplication(sys.argv)
    win = RobotSimulationSystem(
        robot_xml_path=args.robot_model,
        env_xml_path=args.env,
        inertia_txt_path=args.inertia_txt
    )
    win.show()
    sys.exit(app.exec_())