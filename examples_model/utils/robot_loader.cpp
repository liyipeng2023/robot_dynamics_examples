// robot_loader.cpp
#include "robot_loader.h"
#include "urdf_parser_impl.h"
#include "mjcf_parser_impl.h"
#include <iostream>

namespace RDL {
namespace Utils {

static void extractInertiaFromSpatial(const Matrix6d& I_spatial, double& mass, Eigen::Vector3d& com, Eigen::Matrix3d& I_com) {
    Eigen::Matrix3d E = Eigen::Matrix3d::Identity();
    mass = (I_spatial(3,3) + I_spatial(4,4) + I_spatial(5,5)) / 3.0;
    if (mass < 1e-12) {
        mass = 0.0;
        com.setZero();
        I_com.setZero();
        return;
    }

    Eigen::Matrix3d c_skew = I_spatial.block<3,3>(0,3) / mass;
    com(0) = c_skew(2,1);
    com(1) = c_skew(0,2);
    com(2) = c_skew(1,0);

    Eigen::Matrix3d I_origin = I_spatial.block<3,3>(0,0);
    I_com = I_origin - mass * (com.squaredNorm() * E - com * com.transpose());
}

std::shared_ptr<RobotDynamics> load_RobotDynamics(
    const std::string& file_path,
    std::map<std::string, int>& joint_map,
    RobotFileType file_type,
    bool is_floating_base)
{
    // 1. 创建对应的解析器策略
    std::unique_ptr<IRobotParser> parser;
    switch (file_type) {
        case RobotFileType::URDF:
            parser = std::make_unique<URDFParser>();
            break;
        case RobotFileType::MJCF:
            parser = std::make_unique<MJCFParser>();
            break;
    }

    // 2. 解析为通用 IR 节点表示
    std::vector<RobotNodeInfo> nodes;
    if (!parser || !parser->parse(file_path, nodes, is_floating_base)) {
        std::cerr << "[Robot Loader] Error parsing file: " << file_path << "\n";
        return nullptr;
    }

    int num_nodes = nodes.size();

    // 3. 构建动力学 Graph
    std::shared_ptr<Graph> internal_graph(createGraph(num_nodes), deleteGraph);
    for (int i = 1; i < num_nodes; ++i) {
        internal_graph->add_edge(nodes[i].parent_id, i);
    }

    // 4. 生成 关节名称 -> Node ID 映射
    joint_map.clear();
    for (int i = 1; i < num_nodes; ++i) {
        if (!nodes[i].joint_name.empty()) {
            joint_map[nodes[i].joint_name] = i;
        }
    }

    // 5. 统一构建动力学模型
    std::shared_ptr<RobotDynamics> robot_dyn(createRobotDynamics(internal_graph.get()), deleteRobotDynamics);

    for (int i = 0; i < num_nodes; ++i) {
        const auto& node = nodes[i];
        if (i == 0) continue;

        if (node.joint_type == IRJointType::FLOATING) {
            robot_dyn->set_joint_type(i, JointType::FLOATING, 0.0);
        } else if (node.joint_type == IRJointType::PRISMATIC) {
            robot_dyn->set_joint_type(i, JointType::PRISMATIC, 0.0);
        } else {
            robot_dyn->set_joint_type(i, JointType::REVOLUTE, 0.0);
        }

        robot_dyn->set_fixed_transformation(i, node.transform.data(), 16);

        // 提取与对齐旋转转换
        double mass = 0.0;
        Eigen::Vector3d com_raw = Eigen::Vector3d::Zero();
        Eigen::Matrix3d I_com_raw = Eigen::Matrix3d::Identity();
        extractInertiaFromSpatial(node.accumulated_spatial_inertia, mass, com_raw, I_com_raw);

        Eigen::Vector3d com_aligned = node.R_align.transpose() * com_raw;
        Eigen::Matrix3d I_com_aligned = node.R_align.transpose() * I_com_raw * node.R_align;

        DynamicParam link_param(
            I_com_aligned(0,0), I_com_aligned(0,1), I_com_aligned(0,2),
            I_com_aligned(1,1), I_com_aligned(1,2),
            I_com_aligned(2,2),
            mass,
            com_aligned(0), com_aligned(1), com_aligned(2)
        );

        robot_dyn->set_dyn_param(i, link_param);
    }

    robot_dyn->build_model();
    return robot_dyn;
}

} // namespace Utils
} // namespace RDL