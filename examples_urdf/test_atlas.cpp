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

    // 加载 Atlas 模型
    auto atlas_dyn = RDL::Utils::loadRobotDynamicsFromURDF(urdf_path, graph, true);

    const int dof_dyn = 36;
    const int dof_q = 37;

    Eigen::VectorXd q(dof_q), qd(dof_dyn), qdd(dof_dyn);

    q.setZero();
    qd.setZero();
    qdd.setZero();

    // 设定浮动基座初始位姿
    q(0) = 0.0; // x
    q(1) = 0.0; // y
    q(2) = 0.0; // z
    q(3) = 1.0; // w
    q(4) = 0.0; // pos_x
    q(5) = 0.0; // pos_y
    q(6) = 0.92;// pos_z

    for (int i = 7; i < dof_q; ++i) {
        q(i) = 0.0;
    }
    qd = 0.1*Eigen::VectorXd::Ones(dof_dyn);
    qdd = 0.2*Eigen::VectorXd::Ones(dof_dyn);
    qd(0) = 0; qd(1) = 0; qd(2) = 0;
    qd(3) = 0.01; qd(4) = -0.03; qd(5) = 0.05;

    // 1. 运行前向运动学
    atlas_dyn->forward_kinematics(q.data(), q.size());

    // ==========================================
    // 新增：打印 C++ RDL 中所有节点的的世界坐标系位姿
    // ==========================================
    cout << "\n================ C++ RDL Joint Placements ================" << endl;
    // 假设总节点数 = dof_dyn + 1 (World) = 37
    int total_nodes = 32;
    for (int i = 1; i < total_nodes; ++i) {
        Eigen::Matrix4d T_node = Eigen::Matrix4d::Identity();
        atlas_dyn->get_pose(i, T_node.data(), 16);
        cout << "Node/Joint ID " << i << " Pose:\n" << T_node << "\n-----------------------------------" << endl;
    }
    cout << "==========================================================\n" << endl;

    Eigen::MatrixXd M(dof_dyn, dof_dyn);
    Eigen::MatrixXd C(dof_dyn, dof_dyn);
    Eigen::VectorXd g(dof_dyn);

    size_t N = 2000;
    auto start = Clock::now();
    for (size_t i = 0; i < N; i++)
    {
        atlas_dyn->forward_kinematics(q.data(), q.size());
        atlas_dyn->forward_dynamics_crba(qd.data(),M.data(),C.data(),g.data(),qd.size());
    }
    auto end = Clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Atlas - Average CRBA calculation time: " << duration.count() / N << " ns/call" << endl;

    Eigen::VectorXd tau = M * qdd + C * qd + g;
    print("M*qdd", M*qdd);
    print("C*qd", C*qd);
    print("Generalized gravity force vector g:", g.transpose());

    return 0;
}