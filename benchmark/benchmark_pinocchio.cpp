#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <random>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

int main(int argc, char** argv) {
    std::string model_file = "atlas.urdf";
    if (argc > 1) {
        model_file = argv[1];
    }

    std::cout << "========================================\n";
    std::cout << "[Pinocchio Benchmark] Loading Model: " << model_file << "\n";

    pinocchio::Model model;
    try {
        if (model_file.find(".xml") != std::string::npos || model_file.find(".mjcf") != std::string::npos) {
            pinocchio::mjcf::buildModel(model_file, model);
        } else {
            pinocchio::urdf::buildModel(model_file, pinocchio::JointModelFreeFlyer(), model);
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
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);
    Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

    const size_t NUM_ITERATIONS = 50000;

    // ----------------------------------------------------
    // 1. CRBA Benchmark
    // ----------------------------------------------------
    auto start_crba = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        q = pinocchio::randomConfiguration(model);
        pinocchio::forwardKinematics(model, data, q);
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
        q = pinocchio::randomConfiguration(model);
        v.setRandom();
        a.setRandom();
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
        q = pinocchio::randomConfiguration(model);
        v.setRandom();
        tau.setRandom();
        pinocchio::aba(model, data, q, v, tau);
    }
    auto end_aba = std::chrono::high_resolution_clock::now();
    double aba_us = std::chrono::duration<double, std::micro>(end_aba - start_aba).count();
    std::cout << "--> Pinocchio ABA Average Time:  " << (aba_us / NUM_ITERATIONS) << " us/call\n";
    std::cout << "========================================\n";

    return 0;
}