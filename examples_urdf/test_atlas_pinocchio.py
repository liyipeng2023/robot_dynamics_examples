import pinocchio as pin
import numpy as np
import os

def main():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_filename = "atlas.urdf"
    urdf_path = os.path.join(current_dir, urdf_filename)

    if not os.path.exists(urdf_path):
        urdf_path = urdf_filename

    print(f"[Pinocchio Loader] 正在加载 URDF 文件: {urdf_path}")

    # 加载 Atlas 模型并添加 6-DOF 浮动基座
    model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    data = model.createData()
    for i in range(model.njoints):
        print("dynamic parameters of joint {}: {}".format(i, model.inertias[i]))

    q = pin.neutral(model)
    v = 0.1 * np.ones(model.nv)
    a = 0.2 * np.ones(model.nv)

    # 设定浮动基座初始位姿
    q[0:3] = [0.0, 0.0, 0.92]
    q[3:7] = [0.0, 0.0, 0.0, 1.0]

    for i in range(7, model.nq):
        q[i] = 0.0

    v[:6] = [0.01, -0.03, 0.05, 0.0, 0.0, 0.0]

    # 更新运动学数据
    pin.forwardKinematics(model, data, q, v, a)
    pin.updateFramePlacements(model, data)

    # ==========================================
    # 新增：打印 Pinocchio 中所有关节的世界坐标系位姿 (data.oMi)
    # ==========================================
    print("\n================ Pinocchio Joint Placements ================")
    # model.njoints 包含了 universe (id=0) 以及所有的关节
    for i in range(1, model.njoints):
        joint_name = model.names[i]
        # data.oMi[i] 是第 i 个关节相对于世界坐标系的 SE3 位姿
        T_mat = data.oMi[i].homogeneous
        print(f"Joint ID {i} ({joint_name}) Pose:\n{T_mat}\n-----------------------------------")
    print("==========================================================\n")

    M = pin.crba(model, data, q)
    C = pin.computeCoriolisMatrix(model, data, q, v)
    g = pin.computeGeneralizedGravity(model, data, q)
    tau = M @ a + C @ v + g

    print("Pinocchio M*a (前 6 位):", (M @ a))
    print("Pinocchio g (前 6 位):", g[:6])

if __name__ == "__main__":
    main()