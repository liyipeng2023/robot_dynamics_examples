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
        q(3) = 0.0;
        q(4) = 0.0;
        q(5) = 0.0;
        q(6) = 1.0; // 四元数 w = 1.0
        q(2) = 0.92; // 默认高度
    }

    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);

    const size_t NUM_ITERATIONS = 50000;
    int q_start_idx = is_floating_base ? 7 : 0;

    // ----------------------------------------------------
    // 1. CRBA Benchmark
    // ----------------------------------------------------
    pinocchio::forwardKinematics(model, data, q);
    auto start_crba = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < model.nq; ++k) {
            q(k) = dist(rng);
        }
        pinocchio::crba(model, data, q);
    }
    auto end_crba = std::chrono::high_resolution_clock::now();
    double crba_us = std::chrono::duration<double, std::micro>(end_crba - start_crba).count();
    std::cout << "--> Pinocchio CRBA Average Time: " << (crba_us / NUM_ITERATIONS) << " us/call\n";

    // ----------------------------------------------------
    // 2. RNEA Benchmark
    // ----------------------------------------------------
    auto start_rnea = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < model.nq; ++k) {
            q(k) = dist(rng);
        }
        for (int k = 0; k < model.nv; ++k) {
            v(k) = dist(rng);
            a(k) = dist(rng);
        }
        pinocchio::rnea(model, data, q, v, a);
    }
    auto end_rnea = std::chrono::high_resolution_clock::now();
    double rnea_us = std::chrono::duration<double, std::micro>(end_rnea - start_rnea).count();
    std::cout << "--> Pinocchio RNEA Average Time: " << (rnea_us / NUM_ITERATIONS) << " us/call\n";

    // ----------------------------------------------------
    // 3. ABA Benchmark
    // ----------------------------------------------------
    auto start_aba = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < model.nq; ++k) {
            q(k) = dist(rng);
        }
        for (int k = 0; k < model.nv; ++k) {
            v(k) = dist(rng);
            tau(k) = dist(rng);
        }
        pinocchio::aba(model, data, q, v, tau);
    }
    auto end_aba = std::chrono::high_resolution_clock::now();
    double aba_us = std::chrono::duration<double, std::micro>(end_aba - start_aba).count();
    std::cout << "--> Pinocchio ABA Average Time:  " << (aba_us / NUM_ITERATIONS) << " us/call\n";
    std::cout << "========================================\n";

    return 0;
}