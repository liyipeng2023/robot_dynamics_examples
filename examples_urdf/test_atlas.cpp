//
// Created by liyip on 2026/11/2.
//
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32
#include "./utils/urdf_loader.h"
#include <iostream>
#include <graph.h>
#define EIGEN_USE_THREADS
#include <Eigen/Core>
#include <cmath>
#include "../utils/utils.h"

#include <robot_dynamics.h>

using namespace std;
using namespace RDL;
using namespace Eigen;

int main(int argc, char** argv) {
    std::string urdf_path = "atlas.urdf";
    std::shared_ptr<RDL::Graph> graph;

    // 加载 Atlas 模型，并明确指定开启浮动基座适配（对应 Node 0: World, Node 1: Base FLOATING）
    auto atlas_dyn = RDL::Utils::loadRobotDynamicsFromURDF(urdf_path, graph, true);

    const int dof_dyn = 36;  // 6 (浮动基座) + 30 (活动关节)
    const int dof_q = 37;    // 7 (4位四元数 + 3位位置) + 30 (活动关节)

    Eigen::VectorXd q(dof_q), qd(dof_dyn), qdd(dof_dyn);

    // 初始化 30 自由度 Atlas 加 6 自由度浮动基座的状态
    q.setZero();
    qd.setZero();
    qdd.setZero();

    // 设定浮动基座初始位姿：四元数 [w, x, y, z] = [1, 0, 0, 0]，位置 [x, y, z] = [0, 0, 0.92] (站立高度)
    q(0) = 0.0; // x
    q(1) = 0.0; // y
    q(2) = 0.0; // z
    q(3) = 1.0; // w
    q(4) = 0.0; // pos_x
    q(5) = 0.0; // pos_y
    q(6) = 0.92;// pos_z (假设 pelvis 初始站立高度)

    // 给予 30 个活动关节一些非零初始微小位形以避免奇异配置
    for (int i = 7; i < dof_q; ++i) {
        q(i) = 0.0; //0.01 * ((i-7) % 3);
    }
    qd = 0.1*Eigen::VectorXd::Ones(dof_dyn); // 给定一些初始关节速度
    qdd = 0.2*Eigen::VectorXd::Ones(dof_dyn); // 给定一些初始关节加速度
    qd(0) = 0;
    qd(1) = 0;
    qd(2) = 0;
    qd(3) = 0.01;
    qd(4) = -0.03;
    qd(5) = 0.05;
    atlas_dyn->forward_kinematics(q.data(), q.size());

    Eigen::MatrixXd M(dof_dyn, dof_dyn);
    Eigen::MatrixXd C(dof_dyn, dof_dyn);
    Eigen::VectorXd g(dof_dyn);
    //M.setZero();
    //C.setZero();

    auto start = Clock::now();
    auto end = Clock::now();

    // 1. CRBA 性能基准测试 (计算广义惯性矩阵 M)
    size_t N = 20000;
    start = Clock::now();
    for (size_t i = 0; i < N; i++)
    {
        atlas_dyn->forward_kinematics(q.data(), q.size());
        atlas_dyn->forward_dynamics_crba(qd.data(),M.data(),C.data(),g.data(),qd.size());
    }
    end = Clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Atlas (30-DOF + Floating Base) - Average CRBA calculation time: "
         << duration.count() / N << " ns/call" << endl;

    // 计算科里奥利矩阵 C 与重力向量 g
    //atlas_dyn->cal_coriolis_matrix(qd.data(), C.data(), dof_dyn);
    //atlas_dyn->cal_gravitational_torque(g.data(), g.size());

    Eigen::VectorXd tau = M * qdd + C * qd + g;

    //print("Floating-base inertia matrix block (6x6):", M.block<6, 6>(0, 0));
    //print("Joint-space inertia matrix block (top-left 6x6):", M.block<6, 6>(6, 6));
    //print("Base-to-joint inertia matrix block (6x6):", M.block<6, 6>(0, 6));
    print("M*qdd", M*qdd);
    print("C*qd", C*qd);
    //print("qd", qd);
    print("Generalized gravity force vector g:", g.transpose());
    print("Generalized inverse-dynamics force vector tau:", tau.transpose());

    // 2. 递归牛顿-欧拉（Inverse Dynamics / RNEA）性能基准测试
    start = Clock::now();
    for (size_t i = 0; i < N; i++)
    {
        atlas_dyn->forward_kinematics(q.data(), q.size());
        atlas_dyn->forward_iterations(qd.data(), qdd.data(), dof_dyn);
        atlas_dyn->backward_iterations(tau.data(), tau.size());
    }
    end = Clock::now();
    auto duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Atlas (30-DOF + Floating Base) - Average recursive inverse-dynamics time: "
         << duration_.count() / N << " ns/call" << endl;

    // 3. Articulated-Body Algorithm (ABA) 前向动力学测试
    cout << "Forward-dynamics verification using the articulated-body algorithm (ABA) for Atlas with Floating Base:" << endl;

    Eigen::VectorXd qdd_aba(dof_dyn);
    Eigen::VectorXd zero_tau(dof_dyn);
    zero_tau.setZero();

    start = Clock::now();
    for (size_t i = 0; i < N; i++)
    {
        atlas_dyn->forward_kinematics(q.data(), q.size());
        atlas_dyn->forward_dynamics_aba(qd.data(), zero_tau.data(), qdd_aba.data(), dof_dyn, false);
    }
    end = Clock::now();
    duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Atlas (30-DOF + Floating Base) - Average ABA forward-dynamics time: "
         << duration_.count() / N << " ns/call" << endl;

    print("Generalized acceleration from ABA [rad/s^2 & m/s^2]:", qdd_aba.transpose());
    print("Generalized acceleration from the mass-matrix solution:", -1.0 * (M.inverse() * (C * qd + g)).transpose());

    // 4. 获取关键连杆位姿（例如 Node 1 浮动基座 pelvis 的世界坐标系位姿）
    Eigen::Matrix4d T_root, T_ee;
    // 在 Node 0 = World 的体系下，Node 1 即为浮动基座 pelvis
    atlas_dyn->get_pose(1, T_root.data(), 16);
    // 获取末端连杆位姿（总节点数 = 1(World) + 1(Base) + 30(Joints) = 32）
    atlas_dyn->get_pose(31, T_ee.data(), 16);

    print("Atlas Floating Base Root Pose (Pelvis) T_root:", T_root);
    print("Atlas End-Link Pose T_ee:", T_ee);

    return 0;
}