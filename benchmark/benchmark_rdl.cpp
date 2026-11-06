#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <random>
#include <Eigen/Dense>
#include <graph.h>
#include <robot_dynamics.h>
#include "../utils/include/robot_loader.h"

int main(int argc, char** argv) {
    std::string model_file = "atlas.urdf";
    if (argc > 1) {
        model_file = argv[1];
    }

    std::cout << "========================================\n";
    std::cout << "[RDL Benchmark] Loading Model: " << model_file << "\n";

    std::map<std::string, int> joint_map;
    auto rdl_dyn = RDL::Utils::load_RobotDynamics(
        model_file, joint_map, 
        (model_file.find(".xml") != std::string::npos ? RDL::Utils::RobotFileType::MJCF : RDL::Utils::RobotFileType::URDF), 
        true
    );
    if (!rdl_dyn) {
        std::cerr << "Error loading model into RDL: " << model_file << std::endl;
        return -1;
    }

    const int dof_dyn = rdl_dyn->DOF();
    const int dof_q = rdl_dyn->dim_position();
    std::cout << "[RDL Benchmark] Loaded Successfully. DOF (nv): " << dof_dyn << ", Position DOFs (nq): " << dof_q << "\n";
    std::cout << "========================================\n";

    Eigen::VectorXd q = Eigen::VectorXd::Zero(dof_q);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::MatrixXd M(dof_dyn, dof_dyn);
    Eigen::MatrixXd C(dof_dyn, dof_dyn);
    Eigen::VectorXd g(dof_dyn);
    Eigen::VectorXd qdd_aba(dof_dyn);

    q(3) = 1.0; // 浮动基座四元数 w
    q(6) = 0.92; // 默认高度

    std::mt19937 rng(1337);
    std::uniform_real_distribution<double> dist(-0.5, 0.5);

    const size_t NUM_ITERATIONS = 50000;

    // ----------------------------------------------------
    // 1. CRBA Benchmark
    // ----------------------------------------------------
    M.setZero();
    auto start_crba = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = 7; k < dof_q; ++k) q(k) = dist(rng);
        rdl_dyn->forward_kinematics(q.data(), q.size());
        rdl_dyn->forward_dynamics_crba(M.data(), dof_dyn);
    }
    auto end_crba = std::chrono::high_resolution_clock::now();
    double crba_us = std::chrono::duration<double, std::micro>(end_crba - start_crba).count();
    std::cout << "--> RDL CRBA Average Time:   " << (crba_us / NUM_ITERATIONS) << " us/call\n";
    //std::cout << "M: " << M << std::endl;
    // ----------------------------------------------------
    // 2. RNEA (Coriolis + Gravity) Benchmark
    // ----------------------------------------------------

    auto start_rnea = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {

        for (int k = 7; k < dof_q; ++k) q(k) = dist(rng);
        for (int k = 0; k < dof_dyn; ++k) v(k) = dist(rng);
        for (int k = 0; k < dof_dyn; ++k) a(k) = dist(rng);
        rdl_dyn->forward_kinematics(q.data(), q.size());
        rdl_dyn->forward_iterations(v.data(),a.data(),a.size());
        rdl_dyn->backward_iterations(tau.data(),tau.size());
        //rdl_dyn->cal_coriolis_matrix(v.data(), C.data(), dof_dyn);
        //rdl_dyn->cal_gravitational_torque(g.data(), dof_dyn);
    }
    auto end_rnea = std::chrono::high_resolution_clock::now();
    double rnea_us = std::chrono::duration<double, std::micro>(end_rnea - start_rnea).count();
    std::cout << "--> RDL RNEA Average Time:  " << (rnea_us / NUM_ITERATIONS) << " us/call\n";
    //std::cout << "tau: " << tau.transpose() << std::endl;
    // ----------------------------------------------------
    // 3. ABA Benchmark
    // ----------------------------------------------------

    auto start_aba = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = 7; k < dof_q; ++k) q(k) = dist(rng);
        for (int k = 0; k < dof_dyn; ++k) {
            v(k) = dist(rng);
            tau(k) = dist(rng);
        }
        rdl_dyn->forward_kinematics(q.data(), q.size());
        rdl_dyn->forward_dynamics_aba(v.data(), tau.data(), qdd_aba.data(), dof_dyn, false);
    }
    auto end_aba = std::chrono::high_resolution_clock::now();
    double aba_us = std::chrono::duration<double, std::micro>(end_aba - start_aba).count();
    std::cout << "--> RDL ABA Average Time:    " << (aba_us / NUM_ITERATIONS) << " us/call\n";
    std::cout << "========================================\n";
    //std::cout << "qdd_aba: " << qdd_aba.transpose() << std::endl;

    return 0;
}