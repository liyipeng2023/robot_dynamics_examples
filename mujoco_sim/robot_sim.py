import sys
import os
import json
import socket
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFileDialog, QTextEdit, QGroupBox, QOpenGLWidget,
    QFormLayout, QLineEdit, QCheckBox
)
from PyQt5.QtCore import QTimer, Qt
from OpenGL.GL import glClearColor, glClear, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT


class XMLModelMerger:
    """处理环境与机器人MJCF合并、STL相对路径转换及白色背景注入"""

    @staticmethod
    def DefaultEnvXML():
        return """
        <mujoco model="default_environment">
            <option timestep="0.002" gravity="0 0 -9.81"/>
            <visual>
                <headlight ambient="0.6 0.6 0.6" diffuse="0.5 0.5 0.5" specular="0.2 0.2 0.2"/>
                <rgba fog="1 1 1 1"/>
            </visual>
            <asset>
                <texture name="white_skybox" type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="512" height="512"/>
                <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".85 .85 .85" rgb2=".95 .95 .95"/>
                <material name="grid_mat" texture="grid" texrepeat="10 10" texuniform="true" reflectance="0.05"/>
            </asset>
            <worldbody>
                <light pos="0 0 4" dir="0 0 -1" directional="true"/>
                <geom name="ground_plane" type="plane" size="10 10 0.1" material="grid_mat" condim="3" friction="1 0.005 0.0001"/>
            </worldbody>
        </mujoco>
        """

    @staticmethod
    def DemoRobotXML():
        return """
        <mujoco model="demo_robot">
            <worldbody>
                <body name="robot_base" pos="0 0 0.6">
                    <freejoint name="root_free"/>
                    <geom name="base_geom" type="box" size="0.15 0.15 0.1" rgba="0.2 0.5 0.8 1" mass="5"/>
                    <body name="link1" pos="0 0 0.1">
                        <joint name="joint1" type="hinge" axis="0 1 0" range="-180 180" damping="0.5"/>
                        <geom name="link1_geom" type="capsule" fromto="0 0 0 0 0 0.4" size="0.04" rgba="0.9 0.4 0.2 1" mass="1"/>
                        <body name="link2" pos="0 0 0.4">
                            <joint name="joint2" type="hinge" axis="0 1 0" range="-180 180" damping="0.5"/>
                            <geom name="link2_geom" type="capsule" fromto="0 0 0 0.3 0 0" size="0.035" rgba="0.3 0.8 0.4 1" mass="0.8"/>
                        </body>
                    </body>
                </body>
            </worldbody>
            <actuator>
                <motor name="motor1" joint="joint1" gear="30" ctrllimited="true" ctrlrange="-2 2"/>
                <motor name="motor2" joint="joint2" gear="20" ctrllimited="true" ctrlrange="-2 2"/>
            </actuator>
        </mujoco>
        """

    @classmethod
    def resolve_relative_paths(cls, root: ET.Element, base_dir: str):
        """递归解析XML中的STL/OBJ/纹理/include相对路径并转换为绝对路径"""
        if not base_dir:
            return

        # 1. 处理 <include file="..."/> 嵌套引用
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

        # 2. 提取并移除 compiler 上的 meshdir 和 texturedir 属性，直接将路径写死到每个资产上
        meshdir = ""
        texturedir = ""
        for compiler in root.findall("compiler"):
            if "meshdir" in compiler.attrib:
                meshdir = compiler.attrib.pop("meshdir")
            if "texturedir" in compiler.attrib:
                texturedir = compiler.attrib.pop("texturedir")

        abs_meshdir = os.path.normpath(os.path.join(base_dir, meshdir)) if not os.path.isabs(meshdir) else meshdir
        abs_texdir = os.path.normpath(os.path.join(base_dir, texturedir)) if not os.path.isabs(texturedir) else texturedir

        # 3. 转换 <mesh>、<hfield>、<skin>、<texture> 的文件相对路径
        for asset in root.findall("asset"):
            for elem in asset:
                if "file" in elem.attrib:
                    fpath = elem.attrib["file"]
                    if not os.path.isabs(fpath):
                        target_dir = abs_texdir if elem.tag == "texture" else abs_meshdir
                        elem.attrib["file"] = os.path.normpath(os.path.join(target_dir, fpath))

    @classmethod
    def ensure_white_skybox(cls, root: ET.Element):
        """确保场景中拥有纯白色的skybox背景纹理"""
        asset = root.find("asset")
        if asset is None:
            asset = ET.SubElement(root, "asset")
        
        # 移除原有的skybox，强制替换为纯白背景
        for tex in list(asset.findall("texture")):
            if tex.attrib.get("type") == "skybox":
                asset.remove(tex)

        ET.SubElement(asset, "texture", {
            "name": "white_skybox",
            "type": "skybox",
            "builtin": "flat",
            "rgb1": "1 1 1",
            "rgb2": "1 1 1",
            "width": "512",
            "height": "512"
        })

    @classmethod
    def build_combined_xml(cls, env_path: str = None, robot_path: str = None) -> str:
        if env_path and os.path.exists(env_path):
            env_root = ET.parse(env_path).getroot()
            cls.resolve_relative_paths(env_root, os.path.dirname(os.path.abspath(env_path)))
        else:
            env_root = ET.fromstring(cls.DefaultEnvXML())

        if robot_path and os.path.exists(robot_path):
            robot_root = ET.parse(robot_path).getroot()
            cls.resolve_relative_paths(robot_root, os.path.dirname(os.path.abspath(robot_path)))
        else:
            robot_root = ET.fromstring(cls.DemoRobotXML())

        # 将机器人XML的各节点合并入环境XML
        merge_List = ["asset", "default", "worldbody", "actuator", "sensor", "contact", "equality", "tendon", "keyframe"]
        for tag in merge_List:
            for r_section in robot_root.findall(tag):
                e_section = env_root.find(tag)
                if e_section is None:
                    env_root.append(r_section)
                else:
                    for child in list(r_section):
                        e_section.append(child)

        # 合并 compiler 属性（避免覆盖角度制等设置）
        r_comp = robot_root.find("compiler")
        if r_comp is not None:
            e_comp = env_root.find("compiler")
            if e_comp is None:
                env_root.insert(0, r_comp)
            else:
                e_comp.attrib.update(r_comp.attrib)

        cls.ensure_white_skybox(env_root)
        return ET.tostring(env_root, encoding="unicode")


class RDLNetworkNode:
    """与RDL进行UDP JSON数据收发的非阻塞通信模块（无Protobuf依赖）"""

    def __init__(self, local_ip="127.0.0.1", local_port=9001, rdl_ip="127.0.0.1", rdl_port=9002):
        self.rdl_addr = (rdl_ip, rdl_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind((local_ip, local_port))
        except Exception as e:
            print(f"[RDL Network] 端口绑定警告: {e}")
        self.sock.setblocking(False)

    def update_target(self, rdl_ip: str, rdl_port: int):
        self.rdl_addr = (rdl_ip, int(rdl_port))

    def receive_commands(self):
        """接收RDL发送的控制指令，格式示例: {"ctrl": [0.5, -0.2], "qpos_set": [...]}"""
        latest_cmd = None
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
                latest_cmd = json.loads(data.decode("utf-8"))
            except BlockingIOError:
                break
            except Exception as e:
                print(f"[RDL Network] 解析RDL数据包失败: {e}")
                break
        return latest_cmd

    def send_sim_state(self, state_dict: dict):
        """发送机器人关节状态及碰撞动力学数据至RDL"""
        try:
            payload = json.dumps(state_dict).encode("utf-8")
            self.sock.sendto(payload, self.rdl_addr)
        except Exception:
            pass


class MuJoCoViewport(QOpenGLWidget):
    """修复闪退的MuJoCo OpenGL渲染窗口：采用纯Python相机矩阵控制与安全的OpenGL上下文延迟初始化"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.model = None
        self.data = None

        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.pert = mujoco.MjvPerturb()
        self.scn = None
        self.ctx = None
        self._need_reinit_gl = False  # 标记是否需要在OpenGL上下文内重建渲染资源

        self.last_x = 0
        self.last_y = 0
        self.SetDefaultCamera()
        # 允许接收键盘焦点
        self.setFocusPolicy(Qt.StrongFocus)

    def SetDefaultCamera(self):
        mujoco.mjv_defaultCamera(self.cam)
        mujoco.mjv_defaultOption(self.opt)
        mujoco.mjv_defaultPerturb(self.pert)
        self.cam.type = int(mujoco.mjtCamera.mjCAMERA_FREE)
        self.cam.distance = 3.0
        self.cam.azimuth = 135.0
        self.cam.elevation = -20.0
        self.cam.lookat[:] = [0.0, 0.0, 0.5]

    def bind_model(self, model: mujoco.MjModel, data: mujoco.MjData):
        """安全绑定模型：仅做标记，真正的MjrContext放到paintGL的合法上下文中初始化"""
        self.model = model
        self.data = data
        self._need_reinit_gl = True
        self.update()

    def _rebuild_gl_resources(self):
        """在确保OpenGL Context激活的状态下创建Scene和Context"""
        if self.model is not None:
            self.scn = mujoco.MjvScene(self.model, maxgeom=10000)
            self.ctx = mujoco.MjrContext(self.model, int(mujoco.mjtFontScale.mjFONTSCALE_150))
            self._need_reinit_gl = False

    def initializeGL(self):
        glClearColor(1.0, 1.0, 1.0, 1.0)
        self._rebuild_gl_resources()

    def paintGL(self):
        glClearColor(1.0, 1.0, 1.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        if self.model is None or self.data is None:
            return

        # 如果切换了模型，在当前安全的OpenGL线程上下文中重建资源
        if self._need_reinit_gl or self.scn is None or self.ctx is None:
            self._rebuild_gl_resources()

        dpr = self.devicePixelRatio()
        w = max(1, int(self.width() * dpr))
        h = max(1, int(self.height() * dpr))
        viewport = mujoco.MjrRect(0, 0, w, h)

        mujoco.mjv_updateScene(
            self.model, self.data, self.opt, self.pert,
            self.cam, int(mujoco.mjtCatBit.mjCAT_ALL), self.scn
        )
        mujoco.mjr_render(viewport, self.scn, self.ctx)

    def mousePressEvent(self, event):
        # 显式转换为 float，防止 PyQt 版本差异导致类型异常
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

        # 1. 鼠标左键拖拽：旋转视角 (Azimuth 水平角 / Elevation 俯仰角)
        if bool(buttons & Qt.LeftButton):
            self.cam.azimuth = (self.cam.azimuth - dx * 0.4) % 360.0
            self.cam.elevation = float(np.clip(self.cam.elevation - dy * 0.4, -89.0, 89.0))

        # 2. 鼠标右键拖拽：平移视角中心 (Lookat)
        elif bool(buttons & Qt.RightButton):
            az_rad = np.deg2rad(self.cam.azimuth)
            el_rad = np.deg2rad(self.cam.elevation)

            # 计算相机空间的前向、右向和上向单位向量
            forward = np.array([
                np.cos(el_rad) * np.cos(az_rad),
                np.cos(el_rad) * np.sin(az_rad),
                np.sin(el_rad)
            ])
            right = np.array([np.sin(az_rad), -np.cos(az_rad), 0.0])
            up = np.cross(right, forward)

            # 平移灵敏度随相机距离自适应缩放
            scale = self.cam.distance * 0.0015
            self.cam.lookat[:] += (-dx * right + dy * up) * scale

        # 3. 鼠标中键上下拖拽：缩放视角距离
        elif bool(buttons & Qt.MiddleButton):
            self.cam.distance = float(max(0.05, self.cam.distance * (1.0 + dy * 0.01)))

        else:
            return

        self.update()
        event.accept()

    def wheelEvent(self, event):
        if self.model is None:
            return
        # 滚轮缩放相机距离，使用指数缩放保证远近都很平滑
        steps = event.angleDelta().y() / 120.0
        self.cam.distance = float(max(0.05, self.cam.distance * (0.9 ** steps)))
        self.update()
        event.accept()

class RobotSimulationSystem(QMainWindow):
    """主仿真控制窗口"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MuJoCo 机器人仿真系统 (RDL网络通信 & 碰撞动力学)")
        self.resize(1280, 800)

        self.env_xml_path = None
        self.robot_xml_path = None
        self.model = None
        self.data = None
        self.is_paused = False

        self.rdl_net = RDLNetworkNode()
        self._init_ui()
        self.reload_simulation()

        # 仿真主定时器 (约 60Hz 刷新界面，内部根据 timestep 步进物理引擎)
        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self.simulation_step)
        self.sim_timer.start(16)

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        # 左侧 3D 视口
        self.viewport = MuJoCoViewport(self)
        main_layout.addWidget(self.viewport, stretch=3)

        # 右侧控制面板
        panel = QVBoxLayout()
        main_layout.addLayout(panel, stretch=1)

        # 1. 模型选择组
        model_group = QGroupBox("模型管理 (支持STL相对路径)")
        model_layout = QVBoxLayout(model_group)

        self.lbl_env = QLabel("环境模型: 默认平坦地面 (白色背景)")
        self.lbl_robot = QLabel("机器人模型: 内置双连杆演示模型")
        btn_load_env = QPushButton("选择环境 XML 模型...")
        btn_reset_env = QPushButton("恢复默认平坦地面")
        btn_load_robot = QPushButton("选择机器人 XML 模型...")
        btn_reset_sim = QPushButton("重置仿真状态")
        self.btn_pause = QPushButton("暂停 / 继续")

        btn_load_env.clicked.connect(self.select_env_xml)
        btn_reset_env.clicked.connect(self.reset_default_env)
        btn_load_robot.clicked.connect(self.select_robot_xml)
        btn_reset_sim.clicked.connect(self.reset_physics_state)
        self.btn_pause.clicked.connect(self.toggle_pause)

        model_layout.addWidget(self.lbl_env)
        model_layout.addWidget(btn_load_env)
        model_layout.addWidget(btn_reset_env)
        model_layout.addSpacing(8)
        model_layout.addWidget(self.lbl_robot)
        model_layout.addWidget(btn_load_robot)
        model_layout.addSpacing(8)
        model_layout.addWidget(btn_reset_sim)
        model_layout.addWidget(self.btn_pause)
        panel.addWidget(model_group)

        # 2. 碰撞显示与可视化控制
        vis_group = QGroupBox("碰撞检测与可视化")
        vis_layout = QVBoxLayout(vis_group)
        self.chk_show_contact = QCheckBox("在3D视图中显示接触点与接触力矢量")
        self.chk_show_contact.setChecked(True)
        self.chk_show_contact.stateChanged.connect(self.update_vis_flags)
        self.chk_show_hull = QCheckBox("显示碰撞凸包 (Convex Hull)")
        self.chk_show_hull.stateChanged.connect(self.update_vis_flags)
        vis_layout.addWidget(self.chk_show_contact)
        vis_layout.addWidget(self.chk_show_hull)
        panel.addWidget(vis_group)

        # 3. RDL 网络配置组
        net_group = QGroupBox("RDL 网络交互 (UDP JSON)")
        net_form = QFormLayout(net_group)
        self.edt_rdl_ip = QLineEdit("127.0.0.1")
        self.edt_rdl_port = QLineEdit("9002")
        btn_apply_net = QPushButton("更新目标地址")
        btn_apply_net.clicked.connect(self.apply_network_config)
        net_form.addRow("本机监听:", QLabel("127.0.0.1 : 9001"))
        net_form.addRow("RDL IP:", self.edt_rdl_ip)
        net_form.addRow("RDL 端口:", self.edt_rdl_port)
        net_form.addRow(btn_apply_net)
        panel.addWidget(net_group)

        # 4. 实时碰撞动力学监控面板
        col_group = QGroupBox("碰撞检测与碰撞动力学求解结果")
        col_layout = QVBoxLayout(col_group)
        self.txt_collision = QTextEdit()
        self.txt_collision.setReadOnly(True)
        col_layout.addWidget(self.txt_collision)
        panel.addWidget(col_group, stretch=1)

    def update_vis_flags(self):
        if self.viewport.opt:
            self.viewport.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = self.chk_show_contact.isChecked()
            self.viewport.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = self.chk_show_contact.isChecked()
            self.viewport.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = self.chk_show_hull.isChecked()
            self.viewport.update()

    def apply_network_config(self):
        ip = self.edt_rdl_ip.text().strip()
        port = int(self.edt_rdl_port.text().strip())
        self.rdl_net.update_target(ip, port)

    def select_env_xml(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择环境 XML 文件", "", "XML Files (*.xml)")
        if path:
            self.env_xml_path = path
            self.lbl_env.setText(f"环境模型: {os.path.basename(path)}")
            self.reload_simulation()

    def reset_default_env(self):
        self.env_xml_path = None
        self.lbl_env.setText("环境模型: 默认平坦地面 (白色背景)")
        self.reload_simulation()

    def select_robot_xml(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择机器人 XML 文件", "", "XML Files (*.xml)")
        if path:
            self.robot_xml_path = path
            self.lbl_robot.setText(f"机器人模型: {os.path.basename(path)}")
            self.reload_simulation()

    def reload_simulation(self):
        try:
            combined_xml = XMLModelMerger.build_combined_xml(self.env_xml_path, self.robot_xml_path)
            self.model = mujoco.MjModel.from_xml_string(combined_xml)
            self.data = mujoco.MjData(self.model)
            mujoco.mj_forward(self.model, self.data)
            self.viewport.bind_model(self.model, self.data)
            self.update_vis_flags()
        except Exception as e:
            self.txt_collision.setPlainText(f"[模型加载失败]\n请检查XML或STL文件是否存在:\n{e}")

    def reset_physics_state(self):
        if self.model and self.data:
            mujoco.mj_resetData(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            self.viewport.update()

    def toggle_pause(self):
        self.is_paused = not self.is_paused

    def solve_and_extract_collisions(self):
        """提取当前帧所有碰撞对及由碰撞动力学求解出的6D接触力"""
        contacts_info = []
        wrench = np.zeros(6, dtype=np.float64)

        for i in range(self.data.ncon):
            con = self.data.contact[i]
            g1_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, con.geom1) or f"geom_{con.geom1}"
            g2_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, con.geom2) or f"geom_{con.geom2}"

            # 求解第 i 个接触点在接触坐标系下的 6D 力/力矩: [法向力, 切向摩擦1, 切向摩擦2, 扭转力矩, 滚动摩擦1, 滚动摩擦2]
            mujoco.mj_contactForce(self.model, self.data, i, wrench)

            contacts_info.append({
                "geom1": g1_name,
                "geom2": g2_name,
                "pos": np.round(con.pos, 4).tolist(),
                "dist": float(np.round(con.dist, 5)),
                "normal_force": float(np.round(wrench[0], 3)),
                "friction_force": np.round(wrench[1:3], 3).tolist(),
                "torque": np.round(wrench[3:6], 3).tolist()
            })
        return contacts_info

    def simulation_step(self):
        if self.model is None or self.data is None:
            return

        # 1. 接收并执行来自 RDL 的网络控制指令
        cmd = self.rdl_net.receive_commands()
        if cmd is not None:
            if "ctrl" in cmd and self.model.nu > 0:
                ctrl_arr = np.array(cmd["ctrl"], dtype=np.float64)
                n = min(len(ctrl_arr), self.model.nu)
                self.data.ctrl[:n] = ctrl_arr[:n]
            if "qpos_set" in cmd:
                qpos_arr = np.array(cmd["qpos_set"], dtype=np.float64)
                n = min(len(qpos_arr), self.model.nq)
                self.data.qpos[:n] = qpos_arr[:n]
                mujoco.mj_forward(self.model, self.data)

        # 2. 步进物理与碰撞动力学仿真
        if not self.is_paused:
            steps_per_frame = max(1, int(0.016 / self.model.opt.timestep))
            for _ in range(steps_per_frame):
                mujoco.mj_step(self.model, self.data)

        # 3. 提取碰撞检测与接触力求解结果
        contacts = self.solve_and_extract_collisions()

        # 4. 更新右侧碰撞信息显示
        log_lines = [
            f"仿真时间: {self.data.time:.3f} s | 检测到碰撞点数: {len(contacts)}",
            "-" * 42
        ]
        for idx, c in enumerate(contacts[:10]):  # 最多展示前10个接触点防止刷屏卡顿
            log_lines.append(
                f"[{idx+1}] {c['geom1']} <-> {c['geom2']}\n"
                f"    位置: {c['pos']} | 穿透深度: {c['dist']} m\n"
                f"    法向力: {c['normal_force']} N | 切向摩擦: {c['friction_force']} N"
            )
        self.txt_collision.setPlainText("\n".join(log_lines))

        # 5. 通过网络向 RDL 发送当前仿真状态与碰撞动力学数据
        state_packet = {
            "time": round(float(self.data.time), 4),
            "qpos": np.round(self.data.qpos, 5).tolist(),
            "qvel": np.round(self.data.qvel, 5).tolist(),
            "ncon": len(contacts),
            "contacts": contacts
        }
        self.rdl_net.send_sim_state(state_packet)

        # 6. 刷新 OpenGL 渲染视图
        self.viewport.update()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = RobotSimulationSystem()
    win.show()
    sys.exit(app.exec_())