#include <iostream>
#include <chrono>
#include <vector>
#include <string>

// Pinocchio 头文件
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

int main(int argc, char** argv) {
    // 1. 检查命令行参数
    std::string urdf_filename = "spot.urdf"; // 默认路径，可通过命令行参数传入
    if (argc > 1) {
        urdf_filename = argv[1];
    }

    std::cout << "========================================\n";
    std::cout << "[Pinocchio Benchmark] Loading URDF: " << urdf_filename << "\n";

    // 2. 加载 URDF 模型（包含 6-DOF 浮动基座 FreeFlyer）
    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model);
    } catch (const std::exception& e) {
        std::cerr << "Error loading URDF: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "[Pinocchio Benchmark] Model Name: " << model.name << "\n";
    std::cout << "[Pinocchio Benchmark] Total Joints (nv): " << model.nv << "\n";
    std::cout << "[Pinocchio Benchmark] Total Position DOFs (nq): " << model.nq << "\n";
    std::cout << "========================================\n";

    // 创建算法关联的数据结构 Data
    pinocchio::Data data(model);

    // 3. 初始化测试状态 (q, v, tau)
    Eigen::VectorXd q = pinocchio::neutral(model);
    Eigen::VectorXd v = Eigen::VectorXd::Ones(model.nv) * 0.1;
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);

    const size_t NUM_ITERATIONS = 10000; // 迭代 10 万次

    // ----------------------------------------------------
    // 测试 1: 前向运动学 + CRBA (惯性矩阵 M)
    // ----------------------------------------------------
    auto start_kin = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        // 显式计算前向运动学
        pinocchio::forwardKinematics(model, data, q);
        // 计算 CRBA 惯性矩阵 M
        //pinocchio::crba(model, data, q);
    }
    auto end_kin = std::chrono::high_resolution_clock::now();
    
    double total_kin_us = std::chrono::duration<double, std::micro>(end_kin - start_kin).count();
    double avg_kin_ns = (total_kin_us * 1000.0) / NUM_ITERATIONS;

    std::cout << "--> Forward Kinematics Average Time: " << avg_kin_ns << " ns/call (" 
              << avg_kin_ns / 1000.0 << " us/call)\n";

    auto start_crba = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        // 显式计算前向运动学
        pinocchio::forwardKinematics(model, data, q);
        // 计算 CRBA 惯性矩阵 M
        pinocchio::crba(model, data, q);
    }
    auto end_crba = std::chrono::high_resolution_clock::now();
    
    double total_crba_us = std::chrono::duration<double, std::micro>(end_crba - start_crba).count();
    double avg_crba_ns = (total_crba_us * 1000.0) / NUM_ITERATIONS;

    std::cout << "--> CRBA (FK + M) Average Time: " << avg_crba_ns << " ns/call (" 
              << avg_crba_ns / 1000.0 << " us/call)\n";

    // ----------------------------------------------------
    // 测试 2: 前向运动学 + ABA (前向动力学)
    // ----------------------------------------------------
    // 热身循环 (Warm-up)
    for (size_t i = 0; i < 1000; ++i) {
        pinocchio::forwardKinematics(model, data, q, v);
        pinocchio::aba(model, data, q, v, tau);
    }

    auto start_aba = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        // 显式计算前向运动学（包含速度）
        pinocchio::forwardKinematics(model, data, q, v);
        // 计算 ABA 前向动力学
        pinocchio::aba(model, data, q, v, tau);
    }
    auto end_aba = std::chrono::high_resolution_clock::now();

    double total_aba_us = std::chrono::duration<double, std::micro>(end_aba - start_aba).count();
    double avg_aba_ns = (total_aba_us * 1000.0) / NUM_ITERATIONS;

    std::cout << "--> ABA (FK + Forward Dynamics) Average Time: " << avg_aba_ns << " ns/call (" 
              << avg_aba_ns / 1000.0 << " us/call)\n";
    std::cout << "========================================\n";

    return 0;
}