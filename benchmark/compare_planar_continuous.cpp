// pinocchio/fwd.hpp 必须置于所有头文件最前端
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32

#include <graph.h>
#include <robot_dynamics.h>
#include <math_utils.h>
#include "../utils/include/utils.h"

#include <Eigen/Dense>
#define EIGEN_USE_THREADS
#include <Eigen/Core>

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

using namespace std;
using namespace RDL;
using namespace Eigen;

struct ErrorMetrics
{
    double pose_err = 0.0;
    double M_err = 0.0;
    double C_err = 0.0;
    double Cv_err = 0.0;
    double g_err = 0.0;
    double h_err = 0.0;
    double rnea_err = 0.0;
    double aba_zero_err = 0.0;
    double aba_rand_err = 0.0;
    double com_err = 0.0;
    double com_vel_err = 0.0;
    double Jcom_err = 0.0;

    void update_max(const ErrorMetrics& other)
    {
        pose_err     = std::max(pose_err, other.pose_err);
        M_err        = std::max(M_err, other.M_err);
        C_err        = std::max(C_err, other.C_err);
        Cv_err       = std::max(Cv_err, other.Cv_err);
        g_err        = std::max(g_err, other.g_err);
        h_err        = std::max(h_err, other.h_err);
        rnea_err     = std::max(rnea_err, other.rnea_err);
        aba_zero_err = std::max(aba_zero_err, other.aba_zero_err);
        aba_rand_err = std::max(aba_rand_err, other.aba_rand_err);
        com_err      = std::max(com_err, other.com_err);
        com_vel_err  = std::max(com_vel_err, other.com_vel_err);
        Jcom_err     = std::max(Jcom_err, other.Jcom_err);
    }

    double max_all() const
    {
        return std::max({pose_err, M_err, C_err, Cv_err, g_err, h_err,
                         rnea_err, aba_zero_err, aba_rand_err,
                         com_err, com_vel_err, Jcom_err});
    }
};

int main()
{
    Eigen::setNbThreads(1);
    const double pi = 3.141592653589793238462643383279502884;

    // =========================================================================
    // 1. 公共几何与动力学参数准备
    // =========================================================================
    MDH dh_1(0, 0, 0, 0, false);
    MDH dh0(0, 0, 0, 0, false);
    MDH dh1(0, 0, 0, 0.1273, false);
    MDH dh2(-pi / 2, 0, 0, 0, false);
    MDH dh3(0, 0, 0.612, 0, false);
    MDH dh4(0, 0, 0.5723, 0.1639, false);
    MDH dh5(-pi / 2, 0, 0, 0.1157, false);
    MDH dh6(pi / 2, 0, 0, 0.0922, false);

    vector<Eigen::Matrix4d> T(8);
    mdh_to_transformation(dh_1, JointType::REVOLUTE, 0, T[0].data(), 16);
    mdh_to_transformation(dh0, JointType::REVOLUTE, 0, T[1].data(), 16);
    mdh_to_transformation(dh1, JointType::REVOLUTE, 0, T[2].data(), 16);
    mdh_to_transformation(dh2, JointType::REVOLUTE, 0, T[3].data(), 16);
    mdh_to_transformation(dh3, JointType::REVOLUTE, 0, T[4].data(), 16);
    mdh_to_transformation(dh4, JointType::REVOLUTE, 0, T[5].data(), 16);
    mdh_to_transformation(dh5, JointType::REVOLUTE, 0, T[6].data(), 16);
    mdh_to_transformation(dh6, JointType::REVOLUTE, 0, T[7].data(), 16);

    // 原始连杆惯性参数: {ixx, ixy, ixz, iyy, iyz, izz, m, xc, yc, zc}
    const vector<vector<double>> raw_dyn_params = {
        {29.053,   0.0, 0.0, 29.053,   0.0, 200.43,   777.8,  0.00008,  0.00244,  -0.037},   // link 1 (Planar base)
        {0.029053, 0.0, 0.0, 0.029053, 0.0, 3.0243,   7.778,  0.00008,  0.00244,  -0.037},   // link 2 (Continuous)
        {0.023247, 0.0, 0.0, 1.157850, 0.0, 5.157850, 12.930, 0.00001,  0.15061,   0.38757}, // link 3 (Revolute)
        {0.012586, 0.0, 0.0, 0.408420, 0.0, 2.408420, 3.870, -0.00012,  0.06112,   0.1984},  // link 4 (Revolute)
        {0.005267, 0.0, 0.0, 0.005267, 0.0, 0.25625,  1.960, -0.00021, -0.00112,   0.02269}, // link 5 (Revolute)
        {0.005267, 0.0, 0.0, 0.005267, 0.0, 0.25625,  1.960, -0.00021,  0.00112,   0.002269},// link 6 (Revolute)
        {0.000405, 0.0, 0.0, 0.000405, 0.0, 0.1577,   0.202,  0.0,     -0.001156, -0.00149}  // link 7 (Revolute)
    };

    // =========================================================================
    // 2. 构建 RDL 模型
    // =========================================================================
    int num = 8;
    shared_ptr<Graph> graph(createGraph(num), deleteGraph);
    for (int i = 0; i < 7; ++i)
    {
        graph->add_edge(i, i + 1);
    }

    shared_ptr<RobotDynamics> robot_dyn(createRobotDynamics(graph.get()), deleteRobotDynamics);

    for (size_t i = 1; i <= 7; ++i)
    {
        robot_dyn->set_fixed_transformation(i, T[i].data(), 16);
    }

    robot_dyn->set_joint_type(1, JointType::PLANAR, 0);
    robot_dyn->set_joint_type(2, JointType::CONTINUOUS, 0);
    for (size_t i = 3; i <= 7; ++i)
    {
        robot_dyn->set_joint_type(i, JointType::REVOLUTE, 0);
    }

    for (size_t i = 0; i < 7; ++i)
    {
        const auto& p = raw_dyn_params[i];
        DynamicParam param(p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9]);
        robot_dyn->set_dyn_param(i + 1, param);
    }

    robot_dyn->build_model();

    // =========================================================================
    // 3. 构建 Pinocchio 模型
    // =========================================================================
    pinocchio::Model pin_model;
    pin_model.name = "UR_Planar_Continuous_Robot";

    pinocchio::JointIndex parent_id = 0;
    for (size_t i = 0; i < 7; ++i)
    {
        const auto& p = raw_dyn_params[i];
        Eigen::Vector3d com(p[7], p[8], p[9]);
        Eigen::Matrix3d I_mat;
        I_mat << p[0], p[1], p[2],
                 p[1], p[3], p[4],
                 p[2], p[4], p[5];
        pinocchio::Inertia inert(p[6], com, I_mat);

        // 直接使用 T[i+1] 的旋转与平移构造 SE3，保证静态变换完全一致
        pinocchio::SE3 T_static(T[i + 1].block<3, 3>(0, 0), T[i + 1].block<3, 1>(0, 3));

        pinocchio::JointIndex joint_id = 0;
        if (i == 0)
        {
            joint_id = pin_model.addJoint(parent_id, pinocchio::JointModelPlanar(), T_static, "joint_1_planar");
        }
        else if (i == 1)
        {
            joint_id = pin_model.addJoint(parent_id, pinocchio::JointModelRUBZ(), T_static, "joint_2_continuous");
        }
        else
        {
            joint_id = pin_model.addJoint(parent_id, pinocchio::JointModelRZ(), T_static, "joint_" + std::to_string(i + 1));
        }

        pin_model.appendBodyToJoint(joint_id, inert, pinocchio::SE3::Identity());
        parent_id = joint_id;
    }

    pinocchio::Data pin_data(pin_model);

    // =========================================================================
    // 4. 构造切空间置换矩阵 P (9x9)
    //    RDL 速度顺序:       [wz_1, vx_1, vy_1, wz_2, qd_3, ..., qd_7]
    //    Pinocchio 速度顺序: [vx_1, vy_1, wz_1, wz_2, qd_3, ..., qd_7]
    //    满足: v_rdl = P * v_pin  <==>  v_pin = P.transpose() * v_rdl
    // =========================================================================
    const int nq = 11;
    const int nv = 9;
    Eigen::Matrix<double, nv, nv> P = Eigen::Matrix<double, nv, nv>::Identity();
    P.block<3, 3>(0, 0) << 0.0, 0.0, 1.0,
                           1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0;

    // =========================================================================
    // 5. 随机生成 100 组状态并逐项对比
    // =========================================================================
    std::mt19937 rng(2026);
    std::uniform_real_distribution<double> angle_dist(-pi, pi);
    std::uniform_real_distribution<double> pos_dist(-2.0, 2.0);
    std::uniform_real_distribution<double> vel_dist(-2.0, 2.0);
    std::uniform_real_distribution<double> acc_dist(-3.0, 3.0);
    std::uniform_real_distribution<double> tau_dist(-10.0, 10.0);

    const size_t num_trials = 1000;
    ErrorMetrics max_metrics;

    for (size_t iter = 0; iter < num_trials; ++iter)
    {
        // 5.1 随机生成物理位形并分别按 RDL 和 Pinocchio 的顺序填充 q
        double theta_p = angle_dist(rng);
        double x_p     = pos_dist(rng);
        double y_p     = pos_dist(rng);
        double theta_c = angle_dist(rng);

        Eigen::Matrix<double, nq, 1> q_rdl, q_pin;

        // RDL Planar: 姿态在前 [cos, sin]，位置在后 [x, y]
        q_rdl(0) = std::cos(theta_p);
        q_rdl(1) = std::sin(theta_p);
        q_rdl(2) = x_p;
        q_rdl(3) = y_p;

        // Pinocchio Planar: 位置在前 [x, y]，姿态在后 [cos, sin]
        q_pin(0) = x_p;
        q_pin(1) = y_p;
        q_pin(2) = std::cos(theta_p);
        q_pin(3) = std::sin(theta_p);

        // Continuous (Joint 2): 两边均为 [cos, sin]
        q_rdl(4) = std::cos(theta_c);
        q_rdl(5) = std::sin(theta_c);
        q_pin(4) = std::cos(theta_c);
        q_pin(5) = std::sin(theta_c);

        // Revolute (Joints 3~7): 两边一致
        for (int j = 6; j < nq; ++j)
        {
            double th = angle_dist(rng);
            q_rdl(j) = th;
            q_pin(j) = th;
        }

        // 5.2 随机生成广义速度、广义加速度与测试力矩
        Eigen::Matrix<double, nv, 1> v_rdl, a_rdl, tau_rand_rdl;
        for (int j = 0; j < nv; ++j)
        {
            v_rdl(j)        = vel_dist(rng);
            a_rdl(j)        = acc_dist(rng);
            tau_rand_rdl(j) = tau_dist(rng);
        }

        // 映射到 Pinocchio 的切空间顺序: v_pin = P^T * v_rdl
        Eigen::Matrix<double, nv, 1> v_pin        = P.transpose() * v_rdl;
        Eigen::Matrix<double, nv, 1> a_pin        = P.transpose() * a_rdl;
        Eigen::Matrix<double, nv, 1> tau_rand_pin = P.transpose() * tau_rand_rdl;
        Eigen::Matrix<double, nv, 1> zero_tau     = Eigen::Matrix<double, nv, 1>::Zero();

        // ---------------------------------------------------------------------
        // 5.3 执行 RDL 运动学与动力学计算
        // ---------------------------------------------------------------------
        robot_dyn->forward_kinematics(q_rdl.data(), q_rdl.size());

        Eigen::Matrix<double, nv, nv> M_rdl, C_rdl;
        Eigen::Matrix<double, nv, 1> g_rdl, h_rdl, tau_rnea_rdl, qdd_aba_zero_rdl, qdd_aba_rand_rdl;
        Eigen::Vector3d com_rdl, com_vel_rdl;
        Eigen::Matrix<double, 3, nv> Jcom_rdl;

        robot_dyn->forward_dynamics_crba(M_rdl.data(), nv);
        robot_dyn->cal_coriolis_matrix(v_rdl.data(), C_rdl.data(), nv);
        robot_dyn->cal_gravitational_torque(g_rdl.data(), nv);
        robot_dyn->cal_nonlinear_effect(v_rdl.data(), h_rdl.data(), nv);

        // RDL RNEA
        robot_dyn->forward_iterations(v_rdl.data(), a_rdl.data(), nv);
        robot_dyn->backward_iterations(tau_rnea_rdl.data(), nv);

        // RDL ABA (零力矩与随机力矩)
        robot_dyn->forward_dynamics_aba(v_rdl.data(), zero_tau.data(), qdd_aba_zero_rdl.data(), nv, false);
        robot_dyn->forward_dynamics_aba(v_rdl.data(), tau_rand_rdl.data(), qdd_aba_rand_rdl.data(), nv, false);

        // RDL CoM & J_com
        robot_dyn->cal_center_of_mass(com_rdl.data(), 3);
        robot_dyn->cal_center_of_mass_vel(v_rdl.data(), nv, com_vel_rdl.data(), 3);
        robot_dyn->cal_jacobian_com(Jcom_rdl.data(), 3, nv);

        // ---------------------------------------------------------------------
        // 5.4 执行 Pinocchio 运动学与动力学计算
        // ---------------------------------------------------------------------
        pinocchio::forwardKinematics(pin_model, pin_data, q_pin, v_pin, a_pin);

        pinocchio::crba(pin_model, pin_data, q_pin);
        // Pinocchio 的 crba 仅计算上三角，需对称化下三角部分
        pin_data.M.triangularView<Eigen::StrictlyLower>() =
            pin_data.M.transpose().triangularView<Eigen::StrictlyLower>();

        pinocchio::computeCoriolisMatrix(pin_model, pin_data, q_pin, v_pin);
        pinocchio::computeGeneralizedGravity(pin_model, pin_data, q_pin);
        Eigen::Matrix<double, nv, 1> h_pin = pinocchio::nonLinearEffects(pin_model, pin_data, q_pin, v_pin);
        Eigen::Matrix<double, nv, 1> tau_rnea_pin = pinocchio::rnea(pin_model, pin_data, q_pin, v_pin, a_pin);

        Eigen::Matrix<double, nv, 1> qdd_aba_zero_pin = pinocchio::aba(pin_model, pin_data, q_pin, v_pin, zero_tau);
        Eigen::Matrix<double, nv, 1> qdd_aba_rand_pin = pinocchio::aba(pin_model, pin_data, q_pin, v_pin, tau_rand_pin);

        pinocchio::centerOfMass(pin_model, pin_data, q_pin, v_pin);
        pinocchio::jacobianCenterOfMass(pin_model, pin_data, q_pin);

        // ---------------------------------------------------------------------
        // 5.5 坐标系置换对齐并统计当前轮次最大绝对误差
        // ---------------------------------------------------------------------
        ErrorMetrics cur;

        // (a) 各连杆齐次变换矩阵 T_1 ~ T_7 误差
        for (int i = 1; i <= 7; ++i)
        {
            Eigen::Matrix4d T_i_rdl;
            robot_dyn->get_pose(i, T_i_rdl.data(), 16);
            Eigen::Matrix4d T_i_pin = pin_data.oMi[i].toHomogeneousMatrix();
            double err = (T_i_rdl - T_i_pin).cwiseAbs().maxCoeff();
            cur.pose_err = std::max(cur.pose_err, err);
        }

        // (b) 动力学矩阵与向量转换到 RDL 顺序后求误差
        Eigen::Matrix<double, nv, nv> M_pin_aligned = P * pin_data.M * P.transpose();
        Eigen::Matrix<double, nv, nv> C_pin_aligned = P * pin_data.C * P.transpose();
        Eigen::Matrix<double, nv, 1>  g_pin_aligned = P * pin_data.g;
        Eigen::Matrix<double, nv, 1>  h_pin_aligned = P * h_pin;
        Eigen::Matrix<double, nv, 1>  tau_rnea_pin_aligned = P * tau_rnea_pin;
        Eigen::Matrix<double, nv, 1>  qdd_zero_pin_aligned = P * qdd_aba_zero_pin;
        Eigen::Matrix<double, nv, 1>  qdd_rand_pin_aligned = P * qdd_aba_rand_pin;
        Eigen::Matrix<double, 3, nv>  Jcom_pin_aligned     = pin_data.Jcom * P.transpose();

        cur.M_err        = (M_rdl - M_pin_aligned).cwiseAbs().maxCoeff();
        //cur.C_err        = (C_rdl - C_pin_aligned).cwiseAbs().maxCoeff();
        Eigen::Matrix<double, nv, nv> dM_rdl = C_rdl + C_rdl.transpose();
        Eigen::Matrix<double, nv, nv> dM_pin = C_pin_aligned + C_pin_aligned.transpose();
        cur.C_err = (dM_rdl - dM_pin).cwiseAbs().maxCoeff();
        cur.Cv_err       = (C_rdl * v_rdl - C_pin_aligned * v_rdl).cwiseAbs().maxCoeff();
        cur.g_err        = (g_rdl - g_pin_aligned).cwiseAbs().maxCoeff();
        cur.h_err        = (h_rdl - h_pin_aligned).cwiseAbs().maxCoeff();
        cur.rnea_err     = (tau_rnea_rdl - tau_rnea_pin_aligned).cwiseAbs().maxCoeff();
        cur.aba_zero_err = (qdd_aba_zero_rdl - qdd_zero_pin_aligned).cwiseAbs().maxCoeff();
        cur.aba_rand_err = (qdd_aba_rand_rdl - qdd_rand_pin_aligned).cwiseAbs().maxCoeff();
        cur.com_err      = (com_rdl - pin_data.com[0]).cwiseAbs().maxCoeff();
        cur.com_vel_err  = (com_vel_rdl - pin_data.vcom[0]).cwiseAbs().maxCoeff();
        cur.Jcom_err     = (Jcom_rdl - Jcom_pin_aligned).cwiseAbs().maxCoeff();

        max_metrics.update_max(cur);
    }

    // =========================================================================
    // 6. 打印 100 次随机构型对比汇总报告
    // =========================================================================
    cout << "\n====================================================================\n";
    cout << "   RDL vs Pinocchio (Planar + Continuous + 5 Revolute) 100次随机对比\n";
    cout << "====================================================================\n";
    cout << std::scientific << std::setprecision(6);
    cout << left << setw(36) << "1. Forward Kinematics Pose (T1~T7)" << "Max Abs Error: " << max_metrics.pose_err << "\n";
    cout << left << setw(36) << "2. Mass Matrix (CRBA M)"            << "Max Abs Error: " << max_metrics.M_err << "\n";
    cout << left << setw(36) << "3. Coriolis Matrix (C)"             << "Max Abs Error: " << max_metrics.C_err << "\n";
    cout << left << setw(36) << "4. Coriolis Term (C * qd)"          << "Max Abs Error: " << max_metrics.Cv_err << "\n";
    cout << left << setw(36) << "5. Gravity Vector (g)"              << "Max Abs Error: " << max_metrics.g_err << "\n";
    cout << left << setw(36) << "6. Nonlinear Effects (h = C*qd+g)"  << "Max Abs Error: " << max_metrics.h_err << "\n";
    cout << left << setw(36) << "7. Inverse Dynamics (RNEA tau)"     << "Max Abs Error: " << max_metrics.rnea_err << "\n";
    cout << left << setw(36) << "8. Forward Dynamics (ABA tau=0)"    << "Max Abs Error: " << max_metrics.aba_zero_err << "\n";
    cout << left << setw(36) << "9. Forward Dynamics (ABA rand tau)" << "Max Abs Error: " << max_metrics.aba_rand_err << "\n";
    cout << left << setw(36) << "10. Center of Mass Position (com)"  << "Max Abs Error: " << max_metrics.com_err << "\n";
    cout << left << setw(36) << "11. Center of Mass Velocity (vcom)" << "Max Abs Error: " << max_metrics.com_vel_err << "\n";
    cout << left << setw(36) << "12. CoM Jacobian (J_com)"           << "Max Abs Error: " << max_metrics.Jcom_err << "\n";
    cout << "====================================================================\n";

    if (max_metrics.max_all() < 1e-10)
    {
        cout << "[PASS] All 100 random trials matched within machine precision (< 1e-10)!\n";
    }
    else
    {
        cout << "[WARN] Some metrics exceeded 1e-10 (Overall Max Error = " << max_metrics.max_all() << ").\n";
    }

    return 0;
}