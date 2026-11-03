import pinocchio as pin
import numpy as np
import os
import time

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
    #for i in range(model.njoints):
    #    print("dynamic parameters of joint {}: {}".format(i, model.inertias[i]))
    model.gravity.linear = np.array([0.0, 0.0, -9.81])  # 设置重力加速度向量
    q = pin.neutral(model)
    v = 0.1 * np.ones(model.nv)
    a = 0.2 * np.ones(model.nv)

    # 设定浮动基座初始位姿
    q[0:3] = [0.0, 0.0, 0.92]
    q[3:7] = [0.0, 0.0, 0.0, 1.0]

    for i in range(7, model.nq):
        q[i] = 0.0

    v[:6] = [0.01, -0.03, 0.05, 0.0, 0.0, 0.0]
    #print(a[2])
    # 更新运动学数据
    #pin.forwardKinematics(model, data, q, v, a)
    #pin.updateFramePlacements(model, data)

    # 打印 Pinocchio 中所有关节的世界坐标系位姿 (data.oMi)
    print("\n================ Pinocchio Joint Placements ================")
    for i in range(1, model.njoints):
        joint_name = model.names[i]
        T_mat = data.oMi[i].homogeneous
        #print(f"Joint ID {i} ({joint_name}) Pose:\n{T_mat}\n-----------------------------------")
    print("==========================================================\n")

    M = pin.crba(model, data, q)
    C = pin.computeCoriolisMatrix(model, data, q, v)
    g = pin.computeGeneralizedGravity(model, data, q)
    tau = M @ a + C @ v# + g

    print("Pinocchio M*a (前 6 位):", (M @ a)[:6])
    print("Pinocchio g (前 6 位):", g[:6])
    print("Pinocchio tau (前 6 位):", tau[:6])

    N = 50000

    # ==========================================
    # 新增：Pinocchio CRBA (惯性矩阵计算) 测试与性能基准
    # ==========================================
    start_time = time.time()
    for _ in range(N):
        # pin.crba 内部会自动处理所需的构型运动学更新
        M_bench = pin.crba(model, data, q)
    end_time = time.time()

    crba_duration = (end_time - start_time) / N * 1e9
    print(f"\n--- CRBA Benchmark (Pinocchio) ---")
    print(f"Pinocchio 平均 CRBA 计算时间: {crba_duration:.2f} ns/call")
    print("Pinocchio CRBA 矩阵 M 的维度:", M_bench.shape)

    # ==========================================
    # Pinocchio ABA (前向动力学) 测试与性能基准
    # ==========================================
    start_time = time.time()
    for _ in range(N):
        #pin.forwardKinematics(model, data, q, v, a)
        qdd_aba = pin.aba(model, data, q, v, tau)
    end_time = time.time()
    
    aba_duration = (end_time - start_time) / N * 1e9
    print(f"\n--- Forward Dynamics Verification using ABA (Pinocchio) ---")
    print(f"Pinocchio 平均 ABA 前向动力学时间: {aba_duration:.2f} ns/call")
    print("Pinocchio ABA 计算得到的广义加速度 qdd_aba (前 6 位):", qdd_aba)
    print("原始目标加速度 a (前 6 位):", a)

if __name__ == "__main__":
    main()