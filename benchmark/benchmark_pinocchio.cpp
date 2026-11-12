#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <random>
#include <algorithm>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

double computePercentile(std::vector<double>& times, double percentile) {
    if (times.empty()) return 0.0;
    std::sort(times.begin(), times.end());
    size_t idx = static_cast<size_t>(percentile * (times.size() - 1));
    return times[idx];
}

int main(int argc, char** argv) {
    std::string model_file = "atlas.urdf";
    bool is_floating_base = true;

    if (argc > 1) model_file = argv[1];
    if (argc > 2) is_floating_base = (std::stoi(argv[2]) != 0);

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
    std::uniform_real_distribution<double> dist(-0.1, 0.1);

    Eigen::VectorXd q = pinocchio::neutral(model);
    if (is_floating_base && model.nq >= 7) {
        q(3) = 0.0; q(4) = 0.0; q(5) = 0.0; q(6) = 1.0;
        q(2) = 0.92;
    }

    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);

    const size_t NUM_ITERATIONS = 150000;
    const int WARMUP_ITERATIONS = 1000;
    int q_start_idx = is_floating_base ? 7 : 0;

    // Warm-up cache
    for (int i = 0; i < WARMUP_ITERATIONS; ++i) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::crba(model, data, q);
        pinocchio::rnea(model, data, q, v, a);
        pinocchio::aba(model, data, q, v, tau);
    }

    // ----------------------------------------------------
    // 0. Forward Kinematics (FK) Benchmark
    // ----------------------------------------------------
    std::vector<double> fk_samples;
    fk_samples.reserve(NUM_ITERATIONS);
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < model.nq; ++k) q(k) += dist(rng) * 0.01;

        auto start = std::chrono::high_resolution_clock::now();
        pinocchio::forwardKinematics(model, data, q);
        auto end = std::chrono::high_resolution_clock::now();

        fk_samples.push_back(std::chrono::duration<double, std::micro>(end - start).count());
    }
    double fk_p50 = computePercentile(fk_samples, 0.50);
    double fk_p99 = computePercentile(fk_samples, 0.99);
    std::cout << "--> Pinocchio FK Median Time:      " << fk_p50 << " us/call\n";
    std::cout << "--> Pinocchio FK P99 Time:         " << fk_p99 << " us/call\n";

    // ----------------------------------------------------
    // 1. CRBA Benchmark
    // ----------------------------------------------------
    std::vector<double> crba_samples;
    crba_samples.reserve(NUM_ITERATIONS);
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < model.nq; ++k) q(k) += dist(rng) * 0.01;

        auto start = std::chrono::high_resolution_clock::now();
        pinocchio::crba(model, data, q);
        auto end = std::chrono::high_resolution_clock::now();

        crba_samples.push_back(std::chrono::duration<double, std::micro>(end - start).count());
    }
    double crba_total_p50 = computePercentile(crba_samples, 0.50);
    double crba_total_p99 = computePercentile(crba_samples, 0.99);
    double crba_pure_p50  = std::max(0.0, crba_total_p50 - fk_p50);
    double crba_pure_p99  = std::max(0.0, crba_total_p99 - fk_p99);

    std::cout << "--> Pinocchio CRBA Total Time:     " << crba_total_p50 << " us/call\n";
    std::cout << "--> Pinocchio CRBA Total P99:      " << crba_total_p99 << " us/call\n";
    std::cout << "--> Pinocchio CRBA Pure Time:      " << crba_pure_p50 << " us/call\n";
    std::cout << "--> Pinocchio CRBA Pure P99:       " << crba_pure_p99 << " us/call\n";

    // ----------------------------------------------------
    // 2. RNEA Benchmark
    // ----------------------------------------------------
    std::vector<double> rnea_samples;
    rnea_samples.reserve(NUM_ITERATIONS);
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < model.nq; ++k) q(k) += dist(rng) * 0.01;
        for (int k = 0; k < model.nv; ++k) { v(k) = dist(rng); a(k) = dist(rng); }

        auto start = std::chrono::high_resolution_clock::now();
        pinocchio::rnea(model, data, q, v, a);
        auto end = std::chrono::high_resolution_clock::now();

        rnea_samples.push_back(std::chrono::duration<double, std::micro>(end - start).count());
    }
    double rnea_total_p50 = computePercentile(rnea_samples, 0.50);
    double rnea_total_p99 = computePercentile(rnea_samples, 0.99);
    double rnea_pure_p50  = std::max(0.0, rnea_total_p50 - fk_p50);
    double rnea_pure_p99  = std::max(0.0, rnea_total_p99 - fk_p99);

    std::cout << "--> Pinocchio RNEA Total Time:     " << rnea_total_p50 << " us/call\n";
    std::cout << "--> Pinocchio RNEA Total P99:      " << rnea_total_p99 << " us/call\n";
    std::cout << "--> Pinocchio RNEA Pure Time:      " << rnea_pure_p50 << " us/call\n";
    std::cout << "--> Pinocchio RNEA Pure P99:       " << rnea_pure_p99 << " us/call\n";

    // ----------------------------------------------------
    // 3. ABA Benchmark
    // ----------------------------------------------------
    std::vector<double> aba_samples;
    aba_samples.reserve(NUM_ITERATIONS);
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < model.nq; ++k) q(k) += dist(rng) * 0.01;
        for (int k = 0; k < model.nv; ++k) { v(k) = dist(rng); tau(k) = dist(rng); }

        auto start = std::chrono::high_resolution_clock::now();
        pinocchio::aba(model, data, q, v, tau);
        auto end = std::chrono::high_resolution_clock::now();

        aba_samples.push_back(std::chrono::duration<double, std::micro>(end - start).count());
    }
    double aba_total_p50 = computePercentile(aba_samples, 0.50);
    double aba_total_p99 = computePercentile(aba_samples, 0.99);
    double aba_pure_p50  = std::max(0.0, aba_total_p50 - fk_p50);
    double aba_pure_p99  = std::max(0.0, aba_total_p99 - fk_p99);

    std::cout << "--> Pinocchio ABA Total Time:      " << aba_total_p50 << " us/call\n";
    std::cout << "--> Pinocchio ABA Total P99:       " << aba_total_p99 << " us/call\n";
    std::cout << "--> Pinocchio ABA Pure Time:       " << aba_pure_p50 << " us/call\n";
    std::cout << "--> Pinocchio ABA Pure P99:        " << aba_pure_p99 << " us/call\n";
    std::cout << "========================================\n";

    return 0;
}