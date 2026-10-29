#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif

#include <graph.h>
#include <robot_dynamics.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Dense>

#include "math_utils.h"
#include "./utils/utils.h"

using namespace std;
using namespace RDL;
using namespace Eigen;

int main()
{
    // 显式单线程，防止多线程调度干扰耗时统计
    Eigen::setNbThreads(1);

    int num = 7;
    shared_ptr<Graph> graph(createGraph(num), deleteGraph);

    // 拓扑结构：0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6
    for (int i = 0; i < 6; ++i) {
        graph->add_edge(i, i + 1);
    }

    // 创建动力学对象
    shared_ptr<RobotDynamics> robot_dyn(createRobotDynamics(graph.get()), deleteRobotDynamics);

    // MDH 参数定义 (alpha, theta, a, d)
    MDH dh0(0, 0, 0, 0, false);
    MDH dh1(0, 0, 0, 0.1273, false);
    MDH dh2(-M_PI / 2, 0, 0, 0, false);
    MDH dh3(0, 0, 0.612, 0, false);
    MDH dh4(0, 0, 0.5723, 0.1639, false);
    MDH dh5(-M_PI / 2, 0, 0, 0.1157, false);
    MDH dh6(M_PI / 2, 0, 0, 0.0922, false);

    vector<Eigen::Matrix4d> T(7);
    mdh_to_transformation(dh0, JointType::REVOLUTE, 0, T[0].data(), 16);
    mdh_to_transformation(dh1, JointType::REVOLUTE, 0, T[1].data(), 16);
    mdh_to_transformation(dh2, JointType::REVOLUTE, 0, T[2].data(), 16);
    mdh_to_transformation(dh3, JointType::REVOLUTE, 0, T[3].data(), 16);
    mdh_to_transformation(dh4, JointType::REVOLUTE, 0, T[4].data(), 16);
    mdh_to_transformation(dh5, JointType::REVOLUTE, 0, T[5].data(), 16);
    mdh_to_transformation(dh6, JointType::REVOLUTE, 0, T[6].data(), 16);

    for (int i = 1; i <= 6; ++i) {
        robot_dyn->set_fixed_transformation(i, T[i].data(), 16);
        robot_dyn->set_joint_type(i, JointType::REVOLUTE, 0);
    }

    // 惯性与动力学参数配置
    DynamicParam link1_param(0.029053, 0.0, 0.0, 0.029053, 0.0, 0.0243,   7.778,  0.00008,  0.00244,  -0.037);
    DynamicParam link2_param(0.023247, 0.0, 0.0, 1.157850, 0.0, 1.157850, 12.930, 0.00001,  0.15061,  0.38757);
    DynamicParam link3_param(0.012586, 0.0, 0.0, 0.408420, 0.0, 0.408420,  3.870, -0.00012, 0.06112,  0.1984);
    DynamicParam link4_param(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.005625,  1.960, -0.00021, -0.00112, 0.02269);
    DynamicParam link5_param(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.005625,  1.960, -0.00021, 0.00112,  0.002269);
    DynamicParam link6_param(0.000405, 0.0, 0.0, 0.000405, 0.0, 0.000577,  0.202,  0.0,     -0.001156, -0.00149);

    robot_dyn->set_dyn_param(1, link1_param);
    robot_dyn->set_dyn_param(2, link2_param);
    robot_dyn->set_dyn_param(3, link3_param);
    robot_dyn->set_dyn_param(4, link4_param);
    robot_dyn->set_dyn_param(5, link5_param);
    robot_dyn->set_dyn_param(6, link6_param);

    robot_dyn->build_model();

    // 初始状态变量
    Eigen::VectorXd q(6), qd(6), qdd(6), tau(6);
    q << 0, -M_PI / 4, 0, -M_PI / 2, 0, M_PI / 3;
    qd << 0.27, -0.5, -0.2, 0.7, -1.3, 0.2;
    qdd << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

    Eigen::MatrixXd M(6, 6), C(6, 6);
    Eigen::VectorXd g(6);

    const size_t WARMUP_N = 1000;
    const size_t TEST_N = 50000;

    // CPU 预热，消除 Branch Prediction 与 Cache 未命中影响
    for (size_t i = 0; i < WARMUP_N; ++i) {
        robot_dyn->forward_kinematics(q.data(), q.size());
        robot_dyn->forward_dynamics_crba(qd.data(), M.data(), C.data(), g.data(), qd.size());
    }

    std::cout << "========================================================\n";
    std::cout << "  RDL Performance Test: Fixed-Base 6-DOF Robot\n";
    std::cout << "  Iterations: " << TEST_N << "\n";
    std::cout << "========================================================\n\n";

    // 1. 正运动学 (Forward Kinematics) 测试
    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < TEST_N; ++i) {
        robot_dyn->forward_kinematics(q.data(), q.size());
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto ns_fk = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cout << "[FK] Forward Kinematics: " << ns_fk / TEST_N << " ns / call\n";

    // 2. CRBA 复合体惯性法 (Mass, Coriolis, Gravity) 测试
    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < TEST_N; ++i) {
        robot_dyn->forward_kinematics(q.data(), q.size());
        robot_dyn->forward_dynamics_crba(qd.data(), M.data(), C.data(), g.data(), qd.size());
    }
    end = std::chrono::high_resolution_clock::now();
    auto ns_crba = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cout << "[CRBA] M, C, G Calculation: " << ns_crba / TEST_N << " ns / call\n";

    // 3. 逆动力学 RNEA (Recursive Newton-Euler) 测试
    tau = M * qdd + C * qd + g;
    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < TEST_N; ++i) {
        robot_dyn->forward_kinematics(q.data(), q.size());
        robot_dyn->forward_iterations(qd.data(), qdd.data(), qd.size());
        robot_dyn->backward_iterations(tau.data(), tau.size());
    }
    end = std::chrono::high_resolution_clock::now();
    auto ns_id = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cout << "[ID] Inverse Dynamics (RNEA): " << ns_id / TEST_N << " ns / call\n";

    // 4. 正动力学 ABA (Articulated-Body Algorithm) 测试
    Eigen::VectorXd qdd_aba(6);
    Eigen::VectorXd zero_tau = Eigen::VectorXd::Zero(6);
    start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < TEST_N; ++i) {
        robot_dyn->forward_kinematics(q.data(), q.size());
        robot_dyn->forward_dynamics_aba(qd.data(), zero_tau.data(), qdd_aba.data(), 6, false);
    }
    end = std::chrono::high_resolution_clock::now();
    auto ns_aba = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cout << "[FD] Forward Dynamics (ABA): " << ns_aba / TEST_N << " ns / call\n";

    std::cout << "\n========================================================\n";
    return 0;
}