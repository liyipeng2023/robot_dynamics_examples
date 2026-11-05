#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <random>

// RDL 头文件
#include <graph.h>
#include <robot_dynamics.h>
#include "../utils/robot_loader.h"

// Pinocchio 头文件
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

bool compareVector(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2, double tol = 1e-5, const std::string& name = "") {
    if (v1.size() != v2.size()) {
        std::cerr << "[" << name << "] Size mismatch: RDL(" << v1.size() << ") vs Pinocchio(" << v2.size() << ")\n";
        return false;
    }
    double max_err = (v1 - v2).lpNorm<Eigen::Infinity>();
    if (max_err > tol) {
        std::cerr << "[" << name << "] Max error exceeds tolerance: " << max_err << " (tol: " << tol << ")\n";
        return false;
    }
    return true;
}

bool compareMatrix(const Eigen::MatrixXd& m1, const Eigen::MatrixXd& m2, double tol = 1e-5, const std::string& name = "") {
    if (m1.rows() != m2.rows() || m1.cols() != m2.cols()) {
        std::cerr << "[" << name << "] Dimension mismatch!\n";
        return false;
    }
    double max_err = (m1 - m2).lpNorm<Eigen::Infinity>();
    if (max_err > tol) {
        std::cerr << "[" << name << "] Matrix max error exceeds tolerance: " << max_err << " (tol: " << tol << ")\n";
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    std::string model_file = "atlas.urdf";
    if (argc > 1) {
        model_file = argv[1];
    }

    std::cout << "========================================\n";
    std::cout << "[Comparison Test] Loading model: " << model_file << "\n";
    std::cout << "========================================\n";

    pinocchio::Model pin_model;
    try {
        if (model_file.find(".xml") != std::string::npos || model_file.find(".mjcf") != std::string::npos) {
            pinocchio::mjcf::buildModel(model_file, pin_model);
        } else {
            pinocchio::urdf::buildModel(model_file, pinocchio::JointModelFreeFlyer(), pin_model);
        }
    } catch (const std::exception& e) {
        std::cerr << "Pinocchio load error: " << e.what() << "\n";
        return -1;
    }
    pinocchio::Data pin_data(pin_model);


    std::map<std::string, int> joint_map;
    auto rdl_dyn = RDL::Utils::load_RobotDynamics(
        model_file, joint_map, 
        (model_file.find(".xml") != std::string::npos ? RDL::Utils::RobotFileType::MJCF : RDL::Utils::RobotFileType::URDF), 
        true
    );
    if (!rdl_dyn) {
        std::cerr << "RDL model load failed for: " << model_file << "\n";
        return -1;
    }

    int nv = pin_model.nv;
    int nq = pin_model.nq;
    int dof_dyn = rdl_dyn->DOF();
    int dof_q = rdl_dyn->dim_position();

    if (nv != dof_dyn || nq != dof_q) {
        std::cerr << "[Warning] Dimension mismatch between Pinocchio and RDL! nv: " << nv << " vs " << dof_dyn << "\n";
    }


    Eigen::VectorXd a0(6);
    a0.setZero();
    a0(5) = -9.81;
    pin_model.gravity.linear() << 0, 0, -9.81; // Pinocchio 重力设置
    rdl_dyn->set_spatial_gravitational_acc(a0.data(), 6);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-0.5, 0.5);

    const int num_tests = 100;
    bool all_passed = true;

    for (int t = 0; t < num_tests; ++t) {
        std::cout << "--- Test Iteration " << (t + 1) << "/" << num_tests << " ---\n";

        // 生成随机构型 q, 速度 v, 加速度 a, 力矩 tau
        Eigen::VectorXd q = pinocchio::randomConfiguration(pin_model);
        Eigen::VectorXd v = Eigen::VectorXd::NullaryExpr(nv, [&]() { return dist(rng); });
        Eigen::VectorXd a = Eigen::VectorXd::NullaryExpr(nv, [&]() { return dist(rng); });
        Eigen::VectorXd tau = Eigen::VectorXd::NullaryExpr(nv, [&]() { return dist(rng); });

        // 前向运动学更新
        pinocchio::forwardKinematics(pin_model, pin_data, q, v, a);
        rdl_dyn->forward_kinematics(q.data(), q.size());

        // --- 1. CRBA 对比 (惯性矩阵 M) ---
        pinocchio::crba(pin_model, pin_data, q);
        pin_data.M.triangularView<Eigen::StrictlyLower>() = pin_data.M.transpose().triangularView<Eigen::StrictlyLower>();

        Eigen::MatrixXd rdl_M(dof_dyn, dof_dyn);
        rdl_M.setZero();
        rdl_dyn->forward_dynamics_crba(rdl_M.data(), dof_dyn);

        if (!compareMatrix(pin_data.M, rdl_M, 1e-10, "CRBA (M matrix)")) all_passed = false;

        // --- 2. RNEA 对比 (逆动力学 tau) ---
        Eigen::VectorXd pin_tau_rnea = pinocchio::rnea(pin_model, pin_data, q, v, a);

        // RDL 逆动力学计算: tau = M*a + C*v + g
        Eigen::MatrixXd rdl_C(dof_dyn, dof_dyn);
        Eigen::VectorXd rdl_g(dof_dyn);
        rdl_C.setZero(); rdl_g.setZero();
        rdl_dyn->cal_coriolis_matrix(v.data(), rdl_C.data(), dof_dyn);
        rdl_dyn->cal_gravitational_torque(rdl_g.data(), dof_dyn);
        Eigen::VectorXd rdl_tau_rnea = rdl_M * a + rdl_C * v + rdl_g;

        if (!compareVector(pin_tau_rnea, rdl_tau_rnea, 1e-10, "RNEA (tau)")) all_passed = false;

        // --- 3. ABA 对比 (前向动力学 qdd) ---
        pinocchio::aba(pin_model, pin_data, q, v, tau);
        Eigen::VectorXd pin_acc_aba = pin_data.ddq;

        Eigen::VectorXd rdl_acc_aba(dof_dyn);
        rdl_acc_aba.setZero();
        rdl_dyn->forward_dynamics_aba(v.data(), tau.data(), rdl_acc_aba.data(), dof_dyn, false);

        if (!compareVector(pin_acc_aba, rdl_acc_aba, 1e-10, "ABA (ddq)")) all_passed = false;
    }

    if (all_passed) {
        std::cout << "\n[SUCCESS] All algorithm outputs match between RDL and Pinocchio!\n";
    } else {
        std::cout << "\n[FAILED] Some discrepancies found.\n";
    }

    return 0;
}