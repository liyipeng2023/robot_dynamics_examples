//
// Created by liyip on 2026/11/2.
//
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32
#include <iostream>
#include <graph.h>
#define EIGEN_USE_THREADS
#include <Eigen/Core>
#include <cmath>
#include "../utils/utils.h"
#include "../utils/robot_loader.h"
#include <robot_dynamics.h>

using namespace std;
using namespace RDL;
using namespace Eigen;

int main(int argc, char** argv) {
    std::string urdf_path = "atlas.urdf";
    std::map<std::string, int> joint_map;

    //auto atlas_dyn = RDL::Utils::load_RobotDynamics_from_URDF(urdf_path, joint_map, true);
    auto atlas_dyn = RDL::Utils::load_RobotDynamics(urdf_path,joint_map,RDL::Utils::RobotFileType::URDF, true);
    const int dof_dyn = atlas_dyn->DOF();//36;
    const int dof_q = atlas_dyn->dim_position();

    Eigen::VectorXd q(dof_q), qd(dof_dyn), qdd(dof_dyn);

    q.setZero();
    qd.setZero();
    qdd.setZero();

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
    //qd(0) = 0; qd(1) = 0; qd(2) = 0;
    //qd(3) = 0.01; qd(4) = -0.03; qd(5) = 0.05;
    VectorXd a0(6);
    a0.setZero();
    a0(5) = -g_acc;
    atlas_dyn->set_spatial_gravitational_acc(a0.data(),6);

    atlas_dyn->forward_kinematics(q.data(), q.size());

    cout << "\n================ C++ RDL Joint Placements ================" << endl;
    for (auto it : joint_map) {
        std::cout << "The index of " << it.first << " is " << it.second << std::endl;
    }
    int total_nodes = 32;
    for (int i = 1; i < total_nodes; ++i) {
        Eigen::Matrix4d T_node = Eigen::Matrix4d::Identity();
        atlas_dyn->get_pose(i, T_node.data(), 16);
    }
    cout << "==========================================================\n" << endl;

    Eigen::MatrixXd M(dof_dyn, dof_dyn);
    Eigen::MatrixXd C(dof_dyn, dof_dyn);
    Eigen::VectorXd g(dof_dyn);
    // note that M, C, and g must be initialized to be zero before calling the dynamics algorithms
    M.setZero();
    C.setZero();
    g.setZero();

    size_t N = 50000;
    auto start = Clock::now();
    for (size_t i = 0; i < N; i++)
    {
        atlas_dyn->forward_kinematics(q.data(), q.size());
        //atlas_dyn->forward_dynamics_crba(M.data(),qd.size());
    }
    auto end = Clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    atlas_dyn->cal_coriolis_matrix(qd.data(), C.data(), dof_dyn);
    atlas_dyn->cal_gravitational_torque(g.data(), dof_dyn);
    cout << "Atlas - Average Kinematics calculation time: " << duration.count() / N << " ns/call" << endl;

    start = Clock::now();
    for (size_t i = 0; i < N; i++)
    {
        atlas_dyn->forward_kinematics(q.data(), q.size());
        atlas_dyn->forward_dynamics_crba(M.data(),qd.size());
    }
    end = Clock::now();
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    atlas_dyn->cal_coriolis_matrix(qd.data(), C.data(), dof_dyn);
    atlas_dyn->cal_gravitational_torque(g.data(), dof_dyn);
    cout << "Atlas - Average CRBA calculation (only M) time: " << duration.count() / N << " ns/call" << endl;

    Eigen::VectorXd tau = M * qdd + C * qd + g;
    print("M*qdd", (M*qdd).transpose());
    print("C*qd", (C*qd).transpose());
    print("Generalized gravity force vector g:", g.transpose());
    print("Generalized inverse-dynamics force vector tau:", tau.transpose());


    cout << "\n--- Forward Dynamics Verification using ABA (C++ RDL) ---" << endl;
    Eigen::VectorXd qdd_aba(dof_dyn);
    qdd_aba.setZero();

    tau.setZero();
    qdd = -1.0*M.inverse()*(C*qd+g);
    start = Clock::now();
    for (size_t i = 0; i < N; i++)
    {
        atlas_dyn->forward_kinematics(q.data(), q.size());
        atlas_dyn->forward_dynamics_aba(qd.data(), tau.data(), qdd_aba.data(), dof_dyn, false);
    }
    end = Clock::now();
    auto aba_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Atlas (30-DOF + Floating Base) - Average ABA forward-dynamics time: "
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