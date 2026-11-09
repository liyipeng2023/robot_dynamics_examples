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
    bool is_floating_base = true;

    if (argc > 1) model_file = argv[1];
    if (argc > 2) is_floating_base = (std::stoi(argv[2]) != 0);

    std::cout << "========================================\n";
    std::cout << "[RDL Benchmark] Loading Model: " << model_file 
              << " (Floating Base: " << (is_floating_base ? "Yes" : "No") << ")\n";

    std::map<std::string, int> joint_map;
    auto rdl_dyn = RDL::Utils::load_RobotDynamics(
        model_file, joint_map, 
        (model_file.find(".xml") != std::string::npos ? RDL::Utils::RobotFileType::MJCF : RDL::Utils::RobotFileType::URDF), 
        is_floating_base
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
    Eigen::VectorXd qdd_aba(dof_dyn);

    if (is_floating_base && dof_q >= 7) {
        q(3) = 1.0;
        q(6) = 0.92;
    }

    std::mt19937 rng(1337);
    std::uniform_real_distribution<double> dist(-0.5, 0.5);

    const size_t NUM_ITERATIONS = 100000;
    int q_start_idx = is_floating_base ? 7 : 0;

    // ----------------------------------------------------
    // 0. Forward Kinematics (FK) Benchmark
    // ----------------------------------------------------
    auto start_fk = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < dof_q; ++k) q(k) = dist(rng);
        rdl_dyn->forward_kinematics(q.data(), q.size());
    }
    auto end_fk = std::chrono::high_resolution_clock::now();
    double fk_us = std::chrono::duration<double, std::micro>(end_fk - start_fk).count() / NUM_ITERATIONS;
    std::cout << "--> RDL FK Average Time:        " << fk_us << " us/call\n";

    // ----------------------------------------------------
    // 1. CRBA Benchmark
    // ----------------------------------------------------
    M.setZero();
    auto start_crba = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < dof_q; ++k) q(k) = dist(rng);
        rdl_dyn->forward_kinematics(q.data(), q.size());
        rdl_dyn->forward_dynamics_crba(M.data(), dof_dyn);
    }
    auto end_crba = std::chrono::high_resolution_clock::now();
    double crba_total_us = std::chrono::duration<double, std::micro>(end_crba - start_crba).count() / NUM_ITERATIONS;
    double crba_pure_us = std::max(0.0, crba_total_us - fk_us);
    std::cout << "--> RDL CRBA Total Time:      " << crba_total_us << " us/call\n";
    std::cout << "--> RDL CRBA Pure Time:       " << crba_pure_us << " us/call\n";

    // ----------------------------------------------------
    // 2. RNEA Benchmark
    // ----------------------------------------------------
    auto start_rnea = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < dof_q; ++k) q(k) = dist(rng);
        for (int k = 0; k < dof_dyn; ++k) { v(k) = dist(rng); a(k) = dist(rng); }
        rdl_dyn->forward_kinematics(q.data(), q.size());
        rdl_dyn->forward_iterations(v.data(), a.data(), a.size());
        rdl_dyn->backward_iterations(tau.data(), tau.size());
    }
    auto end_rnea = std::chrono::high_resolution_clock::now();
    double rnea_total_us = std::chrono::duration<double, std::micro>(end_rnea - start_rnea).count() / NUM_ITERATIONS;
    double rnea_pure_us = std::max(0.0, rnea_total_us - fk_us);
    std::cout << "--> RDL RNEA Total Time:      " << rnea_total_us << " us/call\n";
    std::cout << "--> RDL RNEA Pure Time:       " << rnea_pure_us << " us/call\n";

    // ----------------------------------------------------
    // 3. ABA Benchmark
    // ----------------------------------------------------
    auto start_aba = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < dof_q; ++k) q(k) = dist(rng);
        for (int k = 0; k < dof_dyn; ++k) { v(k) = dist(rng); tau(k) = dist(rng); }
        rdl_dyn->forward_kinematics(q.data(), q.size());
        rdl_dyn->forward_dynamics_aba(v.data(), tau.data(), qdd_aba.data(), dof_dyn, false);
    }
    auto end_aba = std::chrono::high_resolution_clock::now();
    double aba_total_us = std::chrono::duration<double, std::micro>(end_aba - start_aba).count() / NUM_ITERATIONS;
    double aba_pure_us = std::max(0.0, aba_total_us - fk_us);
    std::cout << "--> RDL ABA Total Time:       " << aba_total_us << " us/call\n";
    std::cout << "--> RDL ABA Pure Time:        " << aba_pure_us << " us/call\n";
    std::cout << "========================================\n";

    return 0;
}