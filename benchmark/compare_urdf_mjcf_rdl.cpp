#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <random>
#include <algorithm>
#include <cmath>
#include <map>
#include <Eigen/Dense>
#include <graph.h>
#include <robot_dynamics.h>
#include "../utils/include/robot_loader.h"
#include "../utils/include/urdf_parser_impl.h"
#include "../utils/include/mjcf_parser_impl.h"

int main(int argc, char** argv) {
    std::string model_input = "../models/franka-fr3";
    bool is_floating_base = false;

    if (argc > 1) model_input = argv[1];
    if (argc > 2) is_floating_base = (std::stoi(argv[2]) != 0);

    std::string base_path = model_input;
    size_t dot_pos = base_path.find_last_of('.');
    if (dot_pos != std::string::npos) {
        std::string ext = base_path.substr(dot_pos);
        if (ext == ".urdf" || ext == ".xml") {
            base_path = base_path.substr(0, dot_pos);
        }
    }

    std::string urdf_file = base_path + ".urdf";
    std::string xml_file  = base_path + ".xml";

    std::cout << "========================================================\n";
    std::cout << "[RDL Parser Compare] Model: " << base_path
              << " | Floating Base: " << (is_floating_base ? "Yes" : "No") << "\n";

    // 1. 直接对比底层 Parser 输出的 IR 节点表 (RobotNodeInfo)
    std::vector<RDL::Utils::RobotNodeInfo> nodes_urdf, nodes_xml;
    RDL::Utils::URDFParser urdf_parser;
    RDL::Utils::MJCFParser mjcf_parser;

    if (!urdf_parser.parse(urdf_file, nodes_urdf, is_floating_base)) {
        std::cerr << "[ERROR] Failed to parse URDF: " << urdf_file << "\n";
        return -1;
    }
    if (!mjcf_parser.parse(xml_file, nodes_xml, is_floating_base)) {
        std::cerr << "[ERROR] Failed to parse XML: " << xml_file << "\n";
        return -2;
    }

    std::cout << "[Node Check] URDF nodes: " << nodes_urdf.size()
              << " vs XML nodes: " << nodes_xml.size() << "\n";

    size_t min_nodes = std::min(nodes_urdf.size(), nodes_xml.size());
    for (size_t i = 1; i < min_nodes; ++i) {
        double tf_diff = (nodes_urdf[i].transform - nodes_xml[i].transform).lpNorm<Eigen::Infinity>();
        double in_diff = (nodes_urdf[i].accumulated_spatial_inertia - nodes_xml[i].accumulated_spatial_inertia).lpNorm<Eigen::Infinity>();
        if (tf_diff > 1e-5 || in_diff > 1e-5 || nodes_urdf[i].parent_id != nodes_xml[i].parent_id) {
            std::cout << "  [Diff at Node " << i << "] "
                      << "URDF(" << nodes_urdf[i].joint_name << ", p=" << nodes_urdf[i].parent_id << ") vs "
                      << "XML(" << nodes_xml[i].joint_name << ", p=" << nodes_xml[i].parent_id << ") | "
                      << "T_err=" << std::scientific << std::setprecision(2) << tf_diff
                      << ", Inertia_err=" << in_diff << "\n";
        }
    }

    // 2. 加载 RDL 动力学模型进行 100 次随机状态对比
    std::map<std::string, int> joint_map_urdf, joint_map_xml;
    auto rdl_urdf = RDL::Utils::load_RobotDynamics(
        urdf_file, joint_map_urdf, RDL::Utils::RobotFileType::URDF, is_floating_base
    );
    auto rdl_xml = RDL::Utils::load_RobotDynamics(
        xml_file, joint_map_xml, RDL::Utils::RobotFileType::MJCF, is_floating_base
    );

    if (!rdl_urdf || !rdl_xml) return -3;

    std::vector<RDL::Utils::RobotNodeInfo> u_nodes, x_nodes;
    RDL::Utils::URDFParser().parse(urdf_file, u_nodes, is_floating_base);
    RDL::Utils::MJCFParser().parse(xml_file, x_nodes, is_floating_base);

    std::cout << "URDF nodes count: " << u_nodes.size() << ", XML nodes count: " << x_nodes.size() << "\n";
    for (size_t i = 1; i < std::min(u_nodes.size(), x_nodes.size()); ++i) {
        double t_err = (u_nodes[i].transform - x_nodes[i].transform).lpNorm<Eigen::Infinity>();
        double i_err = (u_nodes[i].accumulated_spatial_inertia - x_nodes[i].accumulated_spatial_inertia).lpNorm<Eigen::Infinity>();
        if (t_err > 1e-4 || i_err > 1e-4 || u_nodes[i].joint_name != x_nodes[i].joint_name || u_nodes[i].parent_id != x_nodes[i].parent_id) {
            std::cout << "[MISMATCH Node " << i << "]\n"
                      << "  URDF: node=" << u_nodes[i].node_name << ", joint=" << u_nodes[i].joint_name
                      << ", parent=" << u_nodes[i].parent_id << ", type=" << (int)u_nodes[i].joint_type << "\n"
                      << "  XML : node=" << x_nodes[i].node_name << ", joint=" << x_nodes[i].joint_name
                      << ", parent=" << x_nodes[i].parent_id << ", type=" << (int)x_nodes[i].joint_type << "\n"
                      << "  T_err = " << t_err << ", Inertia_err = " << i_err << "\n";
            if (t_err > 1e-4) {
                std::cout << "  URDF T:\n" << u_nodes[i].transform << "\n  XML T:\n" << x_nodes[i].transform << "\n";
            }
            if (i_err > 1e-4) {
                std::cout << "  URDF I:\n" << u_nodes[i].accumulated_spatial_inertia << "\n  XML I:\n" << x_nodes[i].accumulated_spatial_inertia << "\n";
            }
            break; // 只打印第一个出错的节点即可定位根源
        }
    }

    const int dof_dyn = rdl_urdf->DOF();
    const int dof_q   = rdl_urdf->dim_position();

    std::cout << "[RDL Compare] Loaded Successfully. DOF (nv): " << dof_dyn
              << ", Position DOFs (nq): " << dof_q << "\n";

    if (dof_dyn != rdl_xml->DOF() || dof_q != rdl_xml->dim_position()) {
        std::cerr << "[ERROR] DOF mismatch! URDF nv=" << dof_dyn << ", XML nv=" << rdl_xml->DOF() << "\n";
        return -4;
    }

    Eigen::VectorXd q   = Eigen::VectorXd::Zero(dof_q);
    Eigen::VectorXd v   = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd a   = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(dof_dyn);

    Eigen::MatrixXd M_urdf   = Eigen::MatrixXd::Zero(dof_dyn, dof_dyn);
    Eigen::MatrixXd M_xml    = Eigen::MatrixXd::Zero(dof_dyn, dof_dyn);
    Eigen::VectorXd tau_urdf = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd tau_xml  = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd qdd_urdf = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd qdd_xml  = Eigen::VectorXd::Zero(dof_dyn);

    if (is_floating_base && dof_q >= 7) {
        q(3) = 1.0;
        q(6) = 0.92;
    }

    std::mt19937 rng(1337);
    std::uniform_real_distribution<double> dist(-0.1, 0.1);

    const size_t NUM_ITERATIONS = 100;
    const int q_start_idx = is_floating_base ? 7 : 0;

    double max_crba_err = 0.0, sum_crba_err = 0.0;
    double max_rnea_err = 0.0, sum_rnea_err = 0.0;
    double max_aba_err  = 0.0, sum_aba_err  = 0.0;

    for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
        for (int k = q_start_idx; k < dof_q; ++k) q(k) += dist(rng) * 0.01;
        for (int k = 0; k < dof_dyn; ++k) {
            v(k)   = dist(rng);
            a(k)   = dist(rng);
            tau(k) = dist(rng);
        }

        rdl_urdf->forward_kinematics(q.data(), q.size());
        rdl_xml->forward_kinematics(q.data(), q.size());

        M_urdf.setZero();
        M_xml.setZero();
        rdl_urdf->forward_dynamics_crba(M_urdf.data(), dof_dyn);
        rdl_xml->forward_dynamics_crba(M_xml.data(), dof_dyn);
        double err_crba = (M_urdf - M_xml).lpNorm<Eigen::Infinity>();
        max_crba_err = std::max(max_crba_err, err_crba);
        sum_crba_err += err_crba;

        rdl_urdf->forward_iterations(v.data(), a.data(), a.size());
        rdl_urdf->backward_iterations(tau_urdf.data(), tau_urdf.size());
        rdl_xml->forward_iterations(v.data(), a.data(), a.size());
        rdl_xml->backward_iterations(tau_xml.data(), tau_xml.size());
        double err_rnea = (tau_urdf - tau_xml).lpNorm<Eigen::Infinity>();
        max_rnea_err = std::max(max_rnea_err, err_rnea);
        sum_rnea_err += err_rnea;

        rdl_urdf->forward_dynamics_aba(v.data(), tau.data(), qdd_urdf.data(), dof_dyn, false);
        rdl_xml->forward_dynamics_aba(v.data(), tau.data(), qdd_xml.data(), dof_dyn, false);
        double err_aba = (qdd_urdf - qdd_xml).lpNorm<Eigen::Infinity>();
        max_aba_err = std::max(max_aba_err, err_aba);
        sum_aba_err += err_aba;
    }
    std::cout << "qdd_urdf: " << qdd_urdf.transpose() << std::endl;
    std::cout << "qdd_xml: " << qdd_xml.transpose() << std::endl;
    std::cout << std::scientific << std::setprecision(6);
    std::cout << "[CRBA (M matrix)] Max error: " << max_crba_err << "\n";
    std::cout << "[RNEA (tau)] Max error: "      << max_rnea_err << "\n";
    std::cout << "[ABA (ddq)] Max error: "       << max_aba_err  << "\n";
    std::cout << "========================================================\n";

    // 返回 0 以保证 Python 脚本正常收集所有模型的误差数据而不中断
    return 0;
}