//
// Created by liyip on 2026/11/3.
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
    std::string urdf_path = "spot.urdf";
    std::shared_ptr<RDL::Graph> graph;

    auto spot_dyn = RDL::Utils::loadRobotDynamicsFromURDF(urdf_path, graph, true);

    if (!spot_dyn) {
        cerr << "[Error] Failed to load Spot URDF from " << urdf_path << endl;
        return -1;
    }


    const int dof_dyn = 18;
    const int dof_q = 19;

    Eigen::VectorXd q(dof_q), qd(dof_dyn), qdd(dof_dyn);

    q.setZero();
    qd.setZero();
    qdd.setZero();

    q(0) = 0.0; // x (quat x)
    q(1) = 0.0; // y (quat y)
    q(2) = 0.0; // z (quat z)
    q(3) = 1.0; // w (quat w)
    q(4) = 0.0; // pos_x
    q(5) = 0.0; // pos_y
    q(6) = 0.50;// pos_z

    for (int i = 7; i < dof_q; ++i) {
        q(i) = 0.0;
    }

    qd = 0.1 * Eigen::VectorXd::Ones(dof_dyn);
    qdd = 0.2 * Eigen::VectorXd::Ones(dof_dyn);

    qd(0) = 0.0;  qd(1) = 0.0;   qd(2) = 0.0;   // Angular velocity
    qd(3) = 0.01; qd(4) = -0.03; qd(5) = 0.05;  // Linear velocity

    spot_dyn->forward_kinematics(q.data(), q.size());

    cout << "\n================ C++ RDL Spot Joint Placements ================" << endl;
    int total_nodes = graph->num(); // 13 (1 World + 1 Base + 12 Active Joints)
    for (int i = 1; i < total_nodes; ++i) {
        Eigen::Matrix4d T_node = Eigen::Matrix4d::Identity();
        spot_dyn->get_pose(i, T_node.data(), 16);
        // cout << "Node/Joint ID " << i << " Pose:\n" << T_node << "\n-----------------------------------" << endl;
    }
    cout << "Total Graph Nodes: " << total_nodes << endl;
    cout << "===============================================================\n" << endl;

    VectorXd a0(6);
    a0.setZero();
    a0(5) = -g_acc;
    spot_dyn->set_spatial_gravitational_acc(a0.data(),6);

    Eigen::MatrixXd M(dof_dyn, dof_dyn);
    Eigen::MatrixXd C(dof_dyn, dof_dyn);
    Eigen::VectorXd g(dof_dyn);
    M.setZero();
    C.setZero();
    g.setZero();

    size_t N = 50000;

    auto start = Clock::now();
    for (size_t i = 0; i < N; i++) {
        spot_dyn->forward_kinematics(q.data(), q.size());
        spot_dyn->forward_dynamics_crba(qd.data(), M.data(), C.data(), g.data(), qd.size());
    }
    auto end = Clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Spot (18-DOF) - Average CRBA calculation (incl. Kinematics) time: "
         << duration.count() / N << " ns/call" << endl;

    start = Clock::now();
    for (size_t i = 0; i < N; i++) {
        spot_dyn->forward_dynamics_crba(qd.data(), M.data(), C.data(), g.data(), qd.size());
    }
    end = Clock::now();
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Spot (18-DOF) - Average CRBA calculation (only M) time: "
         << duration.count() / N << " ns/call" << endl;

    Eigen::VectorXd tau = M * qdd + C * qd + g;
    print("Generalized Mass Matrix M (top-left 6x6):", M.block<6, 6>(0, 0));
    print("Generalized gravity force vector g:", g.transpose());
    print("Generalized inverse-dynamics force vector tau:", tau.transpose());

    cout << "\n--- Forward Dynamics Verification using ABA (Spot) ---" << endl;
    Eigen::VectorXd qdd_aba(dof_dyn);
    qdd_aba.setZero();

    qdd = M.inverse()*(tau-C*qd-g);
    //tau.setZero();
    start = Clock::now();
    for (size_t i = 0; i < N; i++) {
        spot_dyn->forward_kinematics(q.data(), q.size());
        spot_dyn->forward_dynamics_aba(qd.data(), tau.data(), qdd_aba.data(), dof_dyn, false);
    }
    end = Clock::now();
    auto aba_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Spot (18-DOF) - Average ABA forward-dynamics time: "
         << aba_duration.count() / N << " ns/call" << endl;

    print("Generalized acceleration from ABA qdd_aba:", qdd_aba.transpose());
    print("Original target acceleration qdd:", qdd.transpose());

    double err = (qdd_aba - qdd).norm();
    cout << "\nABA Acceleration Error Norm: " << err << endl;
    if (err < 1e-6) {
        cout << "[Verification Passed] ABA forward dynamics is mathematically consistent with CRBA!" << endl;
    } else {
        cout << "[Verification Warning] Numerical error observed in ABA!" << endl;
    }

    return 0;
}