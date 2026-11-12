#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <graph.h>
#include <robot_dynamics.h>
#include "../utils/include/robot_loader.h"

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
    M.setZero();

    if (is_floating_base && dof_q >= 7) {
        q(3) = 1.0;
        q(6) = 0.92;
    }

    std::mt19937 rng(1337);
    std::uniform_real_distribution<double> dist(-0.1, 0.1);

    const size_t NUM_ITERATIONS = 150000;
    const int WARMUP_ITERATIONS = 1000;
    int q_start_idx = is_floating_base ? 7 : 0;

    // Warm-up cache
    for (int i = 0; i < WARMUP_ITERATIONS; ++i) {
        rdl_dyn->forward_kinematics(q.data(), q.size());
        rdl_dyn->forward_dynamics_crba(M.data(), dof_dyn);
        rdl_dyn->forward_iterations(v.data(), a.data(), a.size());
        rdl_dyn->backward_iterations(tau.data(), tau.size());
        rdl_dyn->forward_dynamics_aba(v.data(), tau.data(), qdd_aba.data(), dof_dyn, false);
    }
    // ----------------------------------------------------
    // 0. Forward Kinematics (FK) Benchmark
    // ----------------------------------------------------
    std::vector<double> fk_samples;
    fk_samples.reserve(NUM_ITERATIONS);
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < dof_q; ++k) q(k) += dist(rng) * 0.01;

        auto start = std::chrono::high_resolution_clock::now();
        rdl_dyn->forward_kinematics(q.data(), q.size());
        auto end = std::chrono::high_resolution_clock::now();

        fk_samples.push_back(std::chrono::duration<double, std::micro>(end - start).count());
    }
    double fk_p50 = computePercentile(fk_samples, 0.50);
    double fk_p99 = computePercentile(fk_samples, 0.99);
    std::cout << "--> RDL FK Median Time:        " << fk_p50 << " us/call\n";
    std::cout << "--> RDL FK P99 Time:           " << fk_p99 << " us/call\n";

    // ----------------------------------------------------
    // 1. CRBA Benchmark
    // ----------------------------------------------------
    std::vector<double> crba_samples;
    crba_samples.reserve(NUM_ITERATIONS);
    M.setZero();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < dof_q; ++k) q(k) += dist(rng) * 0.01;

        auto start = std::chrono::high_resolution_clock::now();
        rdl_dyn->forward_kinematics(q.data(), q.size());
        rdl_dyn->forward_dynamics_crba(M.data(), dof_dyn);
        auto end = std::chrono::high_resolution_clock::now();

        crba_samples.push_back(std::chrono::duration<double, std::micro>(end - start).count());
    }
    double crba_total_p50 = computePercentile(crba_samples, 0.50);
    double crba_total_p99 = computePercentile(crba_samples, 0.99);
    double crba_pure_p50  = std::max(0.0, crba_total_p50 - fk_p50);
    double crba_pure_p99  = std::max(0.0, crba_total_p99 - fk_p99);

    std::cout << "--> RDL CRBA Total Time:       " << crba_total_p50 << " us/call\n";
    std::cout << "--> RDL CRBA Total P99:        " << crba_total_p99 << " us/call\n";
    std::cout << "--> RDL CRBA Pure Time:        " << crba_pure_p50 << " us/call\n";
    std::cout << "--> RDL CRBA Pure P99:         " << crba_pure_p99 << " us/call\n";

    // ----------------------------------------------------
    // 2. RNEA Benchmark
    // ----------------------------------------------------
    std::vector<double> rnea_samples;
    rnea_samples.reserve(NUM_ITERATIONS);
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < dof_q; ++k) q(k) += dist(rng) * 0.01;
        for (int k = 0; k < dof_dyn; ++k) { v(k) = dist(rng); a(k) = dist(rng); }

        auto start = std::chrono::high_resolution_clock::now();
        rdl_dyn->forward_kinematics(q.data(), q.size());
        rdl_dyn->forward_iterations(v.data(), a.data(), a.size());
        rdl_dyn->backward_iterations(tau.data(), tau.size());
        auto end = std::chrono::high_resolution_clock::now();

        rnea_samples.push_back(std::chrono::duration<double, std::micro>(end - start).count());
    }
    double rnea_total_p50 = computePercentile(rnea_samples, 0.50);
    double rnea_total_p99 = computePercentile(rnea_samples, 0.99);
    double rnea_pure_p50  = std::max(0.0, rnea_total_p50 - fk_p50);
    double rnea_pure_p99  = std::max(0.0, rnea_total_p99 - fk_p99);

    std::cout << "--> RDL RNEA Total Time:       " << rnea_total_p50 << " us/call\n";
    std::cout << "--> RDL RNEA Total P99:        " << rnea_total_p99 << " us/call\n";
    std::cout << "--> RDL RNEA Pure Time:        " << rnea_pure_p50 << " us/call\n";
    std::cout << "--> RDL RNEA Pure P99:         " << rnea_pure_p99 << " us/call\n";

    // ----------------------------------------------------
    // 3. ABA Benchmark
    // ----------------------------------------------------
    std::vector<double> aba_samples;
    aba_samples.reserve(NUM_ITERATIONS);
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < dof_q; ++k) q(k) += dist(rng) * 0.01;
        for (int k = 0; k < dof_dyn; ++k) { v(k) = dist(rng); tau(k) = dist(rng); }

        auto start = std::chrono::high_resolution_clock::now();
        rdl_dyn->forward_kinematics(q.data(), q.size());
        rdl_dyn->forward_dynamics_aba(v.data(), tau.data(), qdd_aba.data(), dof_dyn, false);
        auto end = std::chrono::high_resolution_clock::now();

        aba_samples.push_back(std::chrono::duration<double, std::micro>(end - start).count());
    }
    double aba_total_p50 = computePercentile(aba_samples, 0.50);
    double aba_total_p99 = computePercentile(aba_samples, 0.99);
    double aba_pure_p50  = std::max(0.0, aba_total_p50 - fk_p50);
    double aba_pure_p99  = std::max(0.0, aba_total_p99 - fk_p99);

    std::cout << "--> RDL ABA Total Time:        " << aba_total_p50 << " us/call\n";
    std::cout << "--> RDL ABA Total P99:         " << aba_total_p99 << " us/call\n";
    std::cout << "--> RDL ABA Pure Time:         " << aba_pure_p50 << " us/call\n";
    std::cout << "--> RDL ABA Pure P99:          " << aba_pure_p99 << " us/call\n";
    std::cout << "========================================\n";

    return 0;
}