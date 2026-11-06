#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <random>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <graph.h>
#include <robot_dynamics.h>
#include "../utils/include/robot_loader.h"



// Convert Pinocchio configuration q [pos(3), quat(4), joints...] to RDL [quat(4), pos(3), joints...]
Eigen::VectorXd pinToRdlQ(const Eigen::VectorXd& q_pin, bool has_floating_base) {
    if (!has_floating_base) {
        return q_pin;
    }
    Eigen::VectorXd q_rdl = q_pin;
    if (q_pin.size() >= 7) {
        q_rdl(0) = q_pin(3); // qx
        q_rdl(1) = q_pin(4); // qy
        q_rdl(2) = q_pin(5); // qz
        q_rdl(3) = q_pin(6); // qw
        q_rdl(4) = q_pin(0); // x
        q_rdl(5) = q_pin(1); // y
        q_rdl(6) = q_pin(2); // z
    }
    return q_rdl;
}

// Convert tangent space vectors (v, a, tau, qdd) between Pinocchio [linear, angular] and RDL [angular, linear]
Eigen::VectorXd convertTangent(const Eigen::VectorXd& vec, bool has_floating_base) {
    if (!has_floating_base) {
        return vec;
    }
    Eigen::VectorXd vec_converted = vec;
    if (vec.size() >= 6) {
        vec_converted(0) = vec(3);
        vec_converted(1) = vec(4);
        vec_converted(2) = vec(5);
        vec_converted(3) = vec(0);
        vec_converted(4) = vec(1);
        vec_converted(5) = vec(2);
    }
    return vec_converted;
}

// Convert inertia matrix M from RDL [angular, linear] to Pinocchio [linear, angular] layout
Eigen::MatrixXd rdlToPinMatrix(const Eigen::MatrixXd& M_rdl, bool has_floating_base) {
    if (!has_floating_base) {
        return M_rdl;
    }
    Eigen::MatrixXd M_pin = M_rdl;
    if (M_rdl.rows() >= 6 && M_rdl.cols() >= 6) {
        Eigen::MatrixXd block_aa = M_rdl.block<3,3>(0,0);
        Eigen::MatrixXd block_al = M_rdl.block<3,3>(0,3);
        Eigen::MatrixXd block_la = M_rdl.block<3,3>(3,0);
        Eigen::MatrixXd block_ll = M_rdl.block<3,3>(3,3);

        M_pin.block<3,3>(0,0) = block_ll;
        M_pin.block<3,3>(0,3) = block_la;
        M_pin.block<3,3>(3,0) = block_al;
        M_pin.block<3,3>(3,3) = block_aa;

        if (M_rdl.rows() > 6) {
            int n_joints = M_rdl.rows() - 6;
            Eigen::MatrixXd cross_lin_j = M_rdl.block(0, 6, 3, n_joints);
            Eigen::MatrixXd cross_ang_j = M_rdl.block(3, 6, 3, n_joints);
            M_pin.block(0, 6, 3, n_joints) = cross_ang_j;
            M_pin.block(3, 6, 3, n_joints) = cross_lin_j;

            Eigen::MatrixXd cross_j_lin = M_rdl.block(6, 0, n_joints, 3);
            Eigen::MatrixXd cross_j_ang = M_rdl.block(6, 3, n_joints, 3);
            M_pin.block(6, 0, n_joints, 3) = cross_j_ang;
            M_pin.block(6, 3, n_joints, 3) = cross_j_lin;
        }
    }
    return M_pin;
}

bool compareVector(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2, double tol = 1e-8, const std::string& name = "") {
    if (v1.size() != v2.size()) {
        std::cerr << "[" << name << "] Size mismatch: v1(" << v1.size() << ") vs v2(" << v2.size() << ")\n";
        return false;
    }
    double max_err = (v1 - v2).lpNorm<Eigen::Infinity>();
    if (max_err > tol) {
        std::cerr << "[" << name << "] Max error exceeds tolerance: " << max_err << " (tol: " << tol << ")\n";
        return false;
    }
    std::cout << "[" << name << "] Max error: " << max_err << std::endl;
    return true;
}

bool compareMatrix(const Eigen::MatrixXd& m1, const Eigen::MatrixXd& m2, double tol = 1e-8, const std::string& name = "") {
    if (m1.rows() != m2.rows() || m1.cols() != m2.cols()) {
        std::cerr << "[" << name << "] Dimension mismatch!\n";
        return false;
    }
    double max_err = (m1 - m2).lpNorm<Eigen::Infinity>();
    if (max_err > tol) {
        std::cerr << "[" << name << "] Matrix max error exceeds tolerance: " << max_err << " (tol: " << tol << ")\n";
        return false;
    }
    std::cout << "[" << name << "] Max error: " << max_err << std::endl;
    return true;
}

int main(int argc, char** argv) {
    std::string model_file = "atlas.urdf";
    bool has_floating_base = true; // Default to floating base enabled

    if (argc > 1) {
        model_file = argv[1];
    }
    if (argc > 2) {
        has_floating_base = (std::stoi(argv[2]) != 0);
    }

    std::cout << "========================================\n";
    std::cout << "[Comparison Test] Loading model: " << model_file << "\n";
    std::cout << "[Comparison Test] Floating Base: " << (has_floating_base ? "Enabled (1)" : "Disabled (0)") << "\n";
    std::cout << "========================================\n";

    pinocchio::Model pin_model;
    try {
        if (has_floating_base) {
            pinocchio::urdf::buildModel(model_file, pinocchio::JointModelFreeFlyer(), pin_model);
        } else {
            pinocchio::urdf::buildModel(model_file, pin_model);
        }
    } catch (const std::exception& e) {
        std::cerr << "Pinocchio load error: " << e.what() << "\n";
        return -1;
    }
    pinocchio::Data pin_data(pin_model);

    std::map<std::string, int> joint_map;
    auto rdl_dyn = RDL::Utils::load_RobotDynamics(
        model_file, joint_map,
        RDL::Utils::RobotFileType::URDF,
        has_floating_base
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
        std::cerr << "[Warning] Dimension mismatch! Pinocchio(nv=" << nv << ", nq=" << nq
                  << ") vs RDL(dof=" << dof_dyn << ", nq=" << dof_q << ")\n";
    }

    Eigen::VectorXd a0(6);
    a0.setZero();
    a0(5) = -9.81;
    pin_model.gravity.linear() << 0, 0, -9.81;
    rdl_dyn->set_spatial_gravitational_acc(a0.data(), 6);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-0.5, 0.5);

    const int num_tests = 100;
    bool all_passed = true;

    for (int t = 0; t < num_tests; ++t) {
        std::cout << "--- Test Iteration " << (t + 1) << "/" << num_tests << " ---\n";

        // Generate safe random configuration to prevent configuration crashes
        Eigen::VectorXd q_pin(nq);
        for (int i = 0; i < nq; ++i) {
            q_pin(i) = dist(rng);
        }

        // Normalize quaternion if floating base is present
        if (has_floating_base && nq >= 7) {
            if (q_pin.segment<4>(3).norm() > 1e-6) {
                q_pin.segment<4>(3).normalize();
            } else {
                q_pin.segment<4>(3) << 0, 0, 0, 1;
            }
        }

        Eigen::VectorXd v_pin = Eigen::VectorXd::NullaryExpr(nv, [&]() { return dist(rng); });
        Eigen::VectorXd a_pin = Eigen::VectorXd::NullaryExpr(nv, [&]() { return dist(rng); });
        Eigen::VectorXd tau_pin = Eigen::VectorXd::NullaryExpr(nv, [&]() { return dist(rng); });

        Eigen::VectorXd q_rdl = pinToRdlQ(q_pin, has_floating_base);
        Eigen::VectorXd v_rdl = convertTangent(v_pin, has_floating_base);
        Eigen::VectorXd a_rdl = convertTangent(a_pin, has_floating_base);
        Eigen::VectorXd tau_rdl = convertTangent(tau_pin, has_floating_base);

        pinocchio::forwardKinematics(pin_model, pin_data, q_pin, v_pin, a_pin);
        rdl_dyn->forward_kinematics(q_rdl.data(), q_rdl.size());

        // 1. CRBA Comparison (Inertia Matrix M)
        pinocchio::crba(pin_model, pin_data, q_pin);
        pin_data.M.triangularView<Eigen::StrictlyLower>() = pin_data.M.transpose().triangularView<Eigen::StrictlyLower>();

        Eigen::MatrixXd rdl_M(dof_dyn, dof_dyn);
        rdl_M.setZero();
        rdl_dyn->forward_dynamics_crba(rdl_M.data(), dof_dyn);

        Eigen::MatrixXd rdl_M_converted = rdlToPinMatrix(rdl_M, has_floating_base);

        if (!compareMatrix(pin_data.M, rdl_M_converted, 1e-7, "CRBA (M matrix)")) {
            all_passed = false;
        }

        // 2. RNEA Comparison (Inverse Dynamics tau)
        Eigen::VectorXd pin_tau_rnea = pinocchio::rnea(pin_model, pin_data, q_pin, v_pin, a_pin);

        Eigen::VectorXd rdl_tau_rnea_rdl(dof_dyn);
        rdl_tau_rnea_rdl.setZero();
        rdl_dyn->forward_iterations(v_rdl.data(), a_rdl.data(), dof_dyn);
        rdl_dyn->backward_iterations(rdl_tau_rnea_rdl.data(), dof_dyn);

        Eigen::VectorXd rdl_tau_rnea_converted = convertTangent(rdl_tau_rnea_rdl, has_floating_base);

        if (!compareVector(pin_tau_rnea, rdl_tau_rnea_converted, 1e-7, "RNEA (tau)")) {
            all_passed = false;
        }

        // 3. ABA Comparison (Forward Dynamics qdd)
        pinocchio::aba(pin_model, pin_data, q_pin, v_pin, tau_pin);
        Eigen::VectorXd pin_acc_aba = pin_data.ddq;

        Eigen::VectorXd rdl_acc_aba_rdl(dof_dyn);
        rdl_acc_aba_rdl.setZero();
        rdl_dyn->forward_dynamics_aba(v_rdl.data(), tau_rdl.data(), rdl_acc_aba_rdl.data(), dof_dyn, false);

        Eigen::VectorXd rdl_acc_aba_converted = convertTangent(rdl_acc_aba_rdl, has_floating_base);

        if (!compareVector(pin_acc_aba, rdl_acc_aba_converted, 1e-7, "ABA (ddq)")) {
            all_passed = false;
        }
    }

    if (all_passed) {
        std::cout << "\n[SUCCESS] All algorithm outputs match perfectly between RDL and Pinocchio!\n";
    } else {
        std::cout << "\n[FAILED] Discrepancies found.\n";
    }

    return 0;
}