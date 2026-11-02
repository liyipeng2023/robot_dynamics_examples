//
// Created by liyip on 2026/11/2.
//
#include "urdf_loader.h"
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <Eigen/Dense>

namespace RDL {
namespace Utils {

// 辅助函数：urdf::Pose 转 Eigen::Matrix4d
static Eigen::Matrix4d urdfPoseToMatrix(const urdf::Pose& pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;
    Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    return T;
}

// 定义 6x6 空间惯性矩阵类型
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// 1. 根据质量、质心和质心惯性张量构建 6x6 空间惯性矩阵
static Matrix6d computeSpatialInertia(double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& I_com) {
    Matrix6d I_spatial = Matrix6d::Zero();
    if (mass <= 0.0) return I_spatial;

    Eigen::Matrix3d E = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d c_skew;
    c_skew <<  0,    -com(2),  com(1),
               com(2),   0,   -com(0),
              -com(1), com(0),   0;

    // 平行轴定理：关于连杆坐标系原点的惯性张量
    Eigen::Matrix3d I_origin = I_com + mass * (com.squaredNorm() * E - com * com.transpose());

    I_spatial.block<3,3>(0,0) = I_origin;
    I_spatial.block<3,3>(0,3) = mass * c_skew;
    I_spatial.block<3,3>(3,0) = mass * c_skew.transpose();
    I_spatial.block<3,3>(3,3) = mass * E;
    return I_spatial;
}

// 2. 空间惯性矩阵坐标变换 (T 从子坐标系到父坐标系)
static Matrix6d transformSpatialInertia(const Matrix6d& I_c, const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d p = T.block<3,1>(0,3);

    Eigen::Matrix3d p_skew;
    p_skew <<  0,    -p(2),  p(1),
               p(2),   0,   -p(0),
              -p(1), p(0),   0;

    Matrix6d X_star = Matrix6d::Zero();
    X_star.block<3,3>(0,0) = R;
    X_star.block<3,3>(0,3) = p_skew * R;
    X_star.block<3,3>(3,3) = R;

    return X_star * I_c * X_star.transpose();
}

// 3. 从 6x6 空间惯性矩阵反解出等效的质量、质心位置及质心惯性张量
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

// 4. 递归累加固定关节子树（实现 Pinocchio 的 Fixed Joint Welding）
static void accumulateFixedSubtree(
    const urdf::LinkSharedPtr& link,
    const Eigen::Matrix4d& accum_transform,
    const std::shared_ptr<urdf::ModelInterface>& model,
    Matrix6d& total_spatial_inertia)
{
    if (!link) return;

    double m = 0.0;
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    Eigen::Matrix3d I_com = Eigen::Matrix3d::Zero();

    if (link->inertial) {
        m = link->inertial->mass;
        com << link->inertial->origin.position.x,
               link->inertial->origin.position.y,
               link->inertial->origin.position.z;
        I_com << link->inertial->ixx, link->inertial->ixy, link->inertial->ixz,
                 link->inertial->ixy, link->inertial->iyy, link->inertial->iyz,
                 link->inertial->ixz, link->inertial->iyz, link->inertial->izz;
    }

    if (m > 0.0) {
        Matrix6d I_local = computeSpatialInertia(m, com, I_com);
        Matrix6d I_transformed = transformSpatialInertia(I_local, accum_transform);
        total_spatial_inertia += I_transformed;
    }

    // 沿所有 FIXED 关节递归向下焊接，遇到活动关节则停止
    for (const auto& joint : link->child_joints) {
        if (joint->type == urdf::Joint::FIXED) {
            auto child_link = std::const_pointer_cast<urdf::Link>(model->getLink(joint->child_link_name));
            if (child_link) {
                Eigen::Matrix4d child_transform = accum_transform * urdfPoseToMatrix(joint->parent_to_joint_origin_transform);
                accumulateFixedSubtree(child_link, child_transform, model, total_spatial_inertia);
            }
        }
    }
}

// 辅助结构体：存储节点动力学信息
struct RobotNodeInfo {
    std::string link_name;
    std::shared_ptr<urdf::Link> link;
    urdf::JointSharedPtr active_joint;
    Eigen::Matrix4d transform;
    int parent_id;
    Matrix6d accumulated_spatial_inertia; // 焊接后的复合空间惯性
};

// 检查子树是否有活动关节
static bool hasActiveJoints(const urdf::LinkSharedPtr& link, const std::shared_ptr<urdf::ModelInterface>& model) {
    if (!link) return false;
    for (const auto& joint : link->child_joints) {
        if (joint->type == urdf::Joint::REVOLUTE ||
            joint->type == urdf::Joint::CONTINUOUS ||
            joint->type == urdf::Joint::PRISMATIC) {
            return true;
        }
        auto child_link = std::const_pointer_cast<urdf::Link>(model->getLink(joint->child_link_name));
        if (child_link && hasActiveJoints(child_link, model)) {
            return true;
        }
    }
    return false;
}

std::shared_ptr<RobotDynamics> loadRobotDynamicsFromURDF(
    const std::string& urdf_file,
    std::shared_ptr<Graph>& out_graph,
    bool is_floating_base)
{
    std::shared_ptr<urdf::ModelInterface> model = urdf::parseURDFFile(urdf_file);
    if (!model) {
        std::cerr << "[URDF Loader] Error: Failed to parse URDF file: " << urdf_file << "\n";
        return nullptr;
    }
    std::cout << "[URDF Loader] Successfully parsed robot: " << model->getName() << "\n";

    auto root_link = std::const_pointer_cast<urdf::Link>(model->getRoot());
    if (!root_link) {
        std::cerr << "[URDF Loader] Error: Root link not found in URDF.\n";
        return nullptr;
    }

    urdf::LinkSharedPtr effective_root = root_link;
    Eigen::Matrix4d base_accum_transform = Eigen::Matrix4d::Identity();

    if (!effective_root->inertial || effective_root->inertial->mass == 0.0) {
        for (const auto& joint : effective_root->child_joints) {
            if (joint->type == urdf::Joint::FIXED) {
                auto child_link = std::const_pointer_cast<urdf::Link>(model->getLink(joint->child_link_name));
                if (child_link && child_link->inertial && child_link->inertial->mass > 0.0) {
                    effective_root = child_link;
                    base_accum_transform = urdfPoseToMatrix(joint->parent_to_joint_origin_transform);
                    break;
                }
            }
        }
    }

    std::vector<RobotNodeInfo> nodes;

    // Node 0: World
    RobotNodeInfo world_node;
    world_node.link_name = "world";
    world_node.link = nullptr;
    world_node.active_joint = nullptr;
    world_node.transform = Eigen::Matrix4d::Identity();
    world_node.parent_id = -1;
    world_node.accumulated_spatial_inertia = Matrix6d::Zero();
    nodes.push_back(world_node);

    // Node 1: Base Link (pelvis)
    RobotNodeInfo base_node;
    base_node.link_name = effective_root->name;
    base_node.link = effective_root;
    base_node.active_joint = nullptr;
    base_node.transform = base_accum_transform;
    base_node.parent_id = 0;

    // 累加 Base 及其固定子树的复合惯性（对齐 Pinocchio 焊接）
    Matrix6d base_inertia = Matrix6d::Zero();
    accumulateFixedSubtree(effective_root, Eigen::Matrix4d::Identity(), model, base_inertia);
    base_node.accumulated_spatial_inertia = base_inertia;
    nodes.push_back(base_node);

    // BFS 遍历提取活动关节
    struct QueueItem {
        urdf::LinkSharedPtr link;
        int parent_id;
        Eigen::Matrix4d accum_transform;
    };

    std::queue<QueueItem> q;
    q.push({effective_root, 1, Eigen::Matrix4d::Identity()});

    while (!q.empty()) {
        auto current = q.front();
        q.pop();

        for (const auto& joint : current.link->child_joints) {
            auto child_link = std::const_pointer_cast<urdf::Link>(model->getLink(joint->child_link_name));
            if (!child_link) continue;

            Eigen::Matrix4d joint_transform = current.accum_transform * urdfPoseToMatrix(joint->parent_to_joint_origin_transform);

            // A. 活动关节
            if (joint->type == urdf::Joint::REVOLUTE ||
                joint->type == urdf::Joint::CONTINUOUS ||
                joint->type == urdf::Joint::PRISMATIC) {

                RobotNodeInfo node_info;
                node_info.link_name = child_link->name;
                node_info.link = child_link;
                node_info.active_joint = joint;
                node_info.transform = joint_transform;
                node_info.parent_id = current.parent_id;

                // 累加该活动关节连杆及其固定子树的复合惯性
                Matrix6d sub_inertia = Matrix6d::Zero();
                accumulateFixedSubtree(child_link, Eigen::Matrix4d::Identity(), model, sub_inertia);
                node_info.accumulated_spatial_inertia = sub_inertia;

                int new_node_id = nodes.size();
                nodes.push_back(node_info);

                q.push({child_link, new_node_id, Eigen::Matrix4d::Identity()});
            }
            // B. 固定关节（如果通往更深层的活动关节，则继续向下搜索）
            else if (joint->type == urdf::Joint::FIXED) {
                if (hasActiveJoints(child_link, model)) {
                    q.push({child_link, current.parent_id, joint_transform});
                }
            }
        }
    }

    int num_nodes = nodes.size();
    std::cout << "[URDF Loader] Pinocchio-Style Welded Tree Nodes: " << num_nodes
              << " (Node 0: World, Node 1: Base, " << (num_nodes - 2) << " Active Joints)\n";

    out_graph = std::shared_ptr<Graph>(createGraph(num_nodes), deleteGraph);
    for (int i = 1; i < num_nodes; ++i) {
        out_graph->add_edge(nodes[i].parent_id, i);
    }

    std::shared_ptr<RobotDynamics> robot_dyn(createRobotDynamics(out_graph.get()), deleteRobotDynamics);

    // 配置模型参数
    for (int i = 0; i < num_nodes; ++i) {
        const auto& node = nodes[i];

        if (i == 0) {
            //robot_dyn->set_joint_type(0, JointType::FIXED, 0);
            //DynamicParam world_param(0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0);
            //robot_dyn->set_dyn_param(0, world_param);
            continue;
        }

        if (i == 1) {
            if (is_floating_base) {
                robot_dyn->set_joint_type(1, JointType::FLOATING, 0);
            } else {
                robot_dyn->set_joint_type(1, JointType::FIXED, 0);
            }
            robot_dyn->set_fixed_transformation(1, node.transform.data(), 16);
        } else {
            auto joint = node.active_joint;
            if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                robot_dyn->set_joint_type(i, JointType::REVOLUTE, 0);
            } else if (joint->type == urdf::Joint::PRISMATIC) {
                robot_dyn->set_joint_type(i, JointType::PRISMATIC, 0);
            }
            robot_dyn->set_fixed_transformation(i, node.transform.data(), 16);
        }

        // 从复合空间惯性中提取等效质量、质心与惯性张量
        double mass = 0.0;
        Eigen::Vector3d com = Eigen::Vector3d::Zero();
        Eigen::Matrix3d I_com = Eigen::Matrix3d::Zero();
        extractInertiaFromSpatial(node.accumulated_spatial_inertia, mass, com, I_com);

        DynamicParam link_param(
            I_com(0,0), I_com(0,1), I_com(0,2),
            I_com(1,1), I_com(1,2),
            I_com(2,2),
            mass,
            com(0), com(1), com(2)
        );
        robot_dyn->set_dyn_param(i, link_param);
    }

    robot_dyn->build_model();
    std::cout << "[URDF Loader] Pinocchio-equivalent welded model built successfully!\n";

    return robot_dyn;
}

} // namespace Utils
} // namespace RDL