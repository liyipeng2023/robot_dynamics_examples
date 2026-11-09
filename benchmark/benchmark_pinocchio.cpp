#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <random>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

int main(int argc, char** argv) {
    std::string model_file = "atlas.urdf";
    bool is_floating_base = true;

    if (argc > 1) {
        model_file = argv[1];
    }
    if (argc > 2) {
        is_floating_base = (std::stoi(argv[2]) != 0);
    }

    std::cout << "========================================\n";
    std::cout << "[Pinocchio Benchmark] Loading Model: " << model_file 
              << " (Floating Base: " << (is_floating_base ? "Yes" : "No") << ")\n";

    pinocchio::Model model;
    try {
        if (is_floating_base) {
            pinocchio::urdf::buildModel(model_file, pinocchio::JointModelFreeFlyer(), model);
        } else {
            pinocchio::urdf::buildModel(model_file, model);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error loading model: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "[Pinocchio Benchmark] Model Name: " << model.name << "\n";
    std::cout << "[Pinocchio Benchmark] Total Joints (nv): " << model.nv << "\n";
    std::cout << "[Pinocchio Benchmark] Total Position DOFs (nq): " << model.nq << "\n";
    std::cout << "========================================\n";

    pinocchio::Data data(model);

    std::mt19937 rng(1337);
    std::uniform_real_distribution<double> dist(-0.5, 0.5);

    Eigen::VectorXd q = pinocchio::neutral(model);
    if (is_floating_base && model.nq >= 7) {
        q(3) = 0.0; q(4) = 0.0; q(5) = 0.0; q(6) = 1.0;
        q(2) = 0.92;
    }

    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);

    const size_t NUM_ITERATIONS = 100000;
    int q_start_idx = is_floating_base ? 7 : 0;

    // ----------------------------------------------------
    // 0. Forward Kinematics (FK) Benchmark
    // ----------------------------------------------------
    auto start_fk = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < model.nq; ++k) q(k) = dist(rng);
        pinocchio::forwardKinematics(model, data, q);
    }
    auto end_fk = std::chrono::high_resolution_clock::now();
    double fk_us = std::chrono::duration<double, std::micro>(end_fk - start_fk).count() / NUM_ITERATIONS;
    std::cout << "--> Pinocchio FK Average Time:     " << fk_us << " us/call\n";

    // ----------------------------------------------------
    // 1. CRBA Benchmark
    // ----------------------------------------------------
    auto start_crba = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < model.nq; ++k) q(k) = dist(rng);
        pinocchio::crba(model, data, q);
    }
    auto end_crba = std::chrono::high_resolution_clock::now();
    double crba_total_us = std::chrono::duration<double, std::micro>(end_crba - start_crba).count() / NUM_ITERATIONS;
    double crba_pure_us = std::max(0.0, crba_total_us - fk_us);
    std::cout << "--> Pinocchio CRBA Total Time:   " << crba_total_us << " us/call\n";
    std::cout << "--> Pinocchio CRBA Pure Time:    " << crba_pure_us << " us/call\n";

    // ----------------------------------------------------
    // 2. RNEA Benchmark
    // ----------------------------------------------------
    auto start_rnea = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < model.nq; ++k) q(k) = dist(rng);
        for (int k = 0; k < model.nv; ++k) { v(k) = dist(rng); a(k) = dist(rng); }
        pinocchio::rnea(model, data, q, v, a);
    }
    auto end_rnea = std::chrono::high_resolution_clock::now();
    double rnea_total_us = std::chrono::duration<double, std::micro>(end_rnea - start_rnea).count() / NUM_ITERATIONS;
    double rnea_pure_us = std::max(0.0, rnea_total_us - fk_us);
    std::cout << "--> Pinocchio RNEA Total Time:   " << rnea_total_us << " us/call\n";
    std::cout << "--> Pinocchio RNEA Pure Time:    " << rnea_pure_us << " us/call\n";

    // ----------------------------------------------------
    // 3. ABA Benchmark
    // ----------------------------------------------------
    auto start_aba = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < model.nq; ++k) q(k) = dist(rng);
        for (int k = 0; k < model.nv; ++k) { v(k) = dist(rng); tau(k) = dist(rng); }
        pinocchio::aba(model, data, q, v, tau);
    }
    auto end_aba = std::chrono::high_resolution_clock::now();
    double aba_total_us = std::chrono::duration<double, std::micro>(end_aba - start_aba).count() / NUM_ITERATIONS;
    double aba_pure_us = std::max(0.0, aba_total_us - fk_us);
    std::cout << "--> Pinocchio ABA Total Time:    " << aba_total_us << " us/call\n";
    std::cout << "--> Pinocchio ABA Pure Time:     " << aba_pure_us << " us/call\n";
    std::cout << "========================================\n";

    return 0;
}