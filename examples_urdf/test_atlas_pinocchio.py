import pinocchio as pin
import numpy as np
import os

def main():
    # 1. 完美解决 Python 相对路径找不到文件的问题
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_filename = "atlas.urdf" # 如果你的文件名是 atlas_2.urdf，请在此处修改
    urdf_path = os.path.join(current_dir, urdf_filename)

    if not os.path.exists(urdf_path):
        # 备选：尝试在当前工作目录直接加载
        urdf_path = urdf_filename

    print(f"[Pinocchio Loader] 正在加载 URDF 文件: {urdf_path}")

    # 2. 加载 Atlas 模型并自动添加 6-DOF 浮动基座 (FreeFlyer)
    # Pinocchio 的 buildModelFromUrdf 会自动处理并焊接 fixed 关节
    model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    data = model.createData()

    print(f"==================================================")
    print(f"成功加载机器人模型: {model.name}")
    print(f"配置空间维数 (nq): {model.nq} (应为 37: 7位浮动基座 + 30位关节)")
    print(f"切空间/速度维数 (nv): {model.nv} (应为 36: 6位空间速度 + 30位关节)")
    
    # 打印 Pinocchio 自动焊接后的总质量（可与 C++ RDL 对比）
    total_mass = sum([inertia.mass for inertia in model.inertias])
    print(f"机器人总质量 (含 Fixed Joints 焊接累加): {total_mass:.3f} kg")
    print(f"==================================================")

    # 3. 初始化广义状态向量 q, v, a (与 C++ 测试程序完全一致)
    q = pin.neutral(model)
    v = 0.1*np.ones(model.nv)
    a = 0.2*np.ones(model.nv)

    # 设定浮动基座初始位姿：平移 [x, y, z] = [0, 0, 0.92]，四元数 [qx, qy, qz, qw] = [0, 0, 0, 1]
    q[0:3] = [0.0, 0.0, 0.92]
    q[3:7] = [0.0, 0.0, 0.0, 1.0]

    # 给 30 个活动关节赋予一些非零初始微小位形以避免奇异配置
    for i in range(7, model.nq):
        q[i] = 0 #0.01 * ((i - 7) % 3)

    # 给广义速度赋予少量初始值
    v[:6] = [0.01, -0.03, 0.05, 0.0, 0.0, 0.0]

    # 更新运动学与动力学基础数据
    pin.forwardKinematics(model, data, q, v, a)

    N = 20000  # 循环基准测试次数

    # -------------------------------------------------------------
    # 4. CRBA 性能与结果测试 (计算广义惯性矩阵 M)
    # -------------------------------------------------------------
    import time
    start_time = time.time()
    for _ in range(N):
        pin.forwardKinematics(model, data, q, v, a)
        M = pin.crba(model, data, q)
    end_time = time.time()
    
    crba_duration = (end_time - start_time) / N * 1e9
    print(f"Pinocchio 平均 CRBA 计算时间: {crba_duration:.2f} ns/call")

    # 计算科里奥利矩阵 C 与重力向量 g
    C = pin.computeCoriolisMatrix(model, data, q, v)
    g = pin.computeGeneralizedGravity(model, data, q)

    tau = M @ a + C @ v + g

    print("\n--- 关键动力学结果摘要 (可与 C++ 对比) ---")
    print("浮动基座惯性矩阵块 M*a:\n", M@a)
    print("广义重力向量 g (前 6 位):", g[:6])
    print("Coriolis 矩阵 C*qd : \n", C@v)
    print("qd: \n", v)
    print("广义逆动力学力矩 tau (前 6 位):", tau)

    # -------------------------------------------------------------
    # 5. RNEA (逆动力学) 测试
    # -------------------------------------------------------------
    start_time = time.time()
    for _ in range(N):
        tau_rnea = pin.rnea(model, data, q, v, a)
    end_time = time.time()
    
    rnea_duration = (end_time - start_time) / N * 1e9
    print(f"\nPinocchio 平均 RNEA 逆动力学时间: {rnea_duration:.2f} ns/call")
    print("RNEA 计算出的广义力矩 (前 6 位):", tau_rnea[:6])

    # -------------------------------------------------------------
    # 6. ABA (前向动力学) 测试
    # -------------------------------------------------------------
    zero_tau = np.zeros(model.nv)
    start_time = time.time()
    for _ in range(N):
        qdd_aba = pin.aba(model, data, q, v, zero_tau)
    end_time = time.time()
    
    aba_duration = (end_time - start_time) / N * 1e9
    print(f"\nPinocchio 平均 ABA 前向动力学时间: {aba_duration:.2f} ns/call")
    print("ABA 计算得到的广义加速度 qdd (前 6 位):", qdd_aba[:6])

if __name__ == "__main__":
    main()