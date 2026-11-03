//
// Created by liyip on 2026/11/2.
//
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32
#include "urdf_loader.h"
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <Eigen/Dense>

namespace RDL {
namespace Utils {

// Helper function: Convert urdf::Pose to Eigen::Matrix4d
static Eigen::Matrix4d urdfPoseToMatrix(const urdf::Pose& pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;
    Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    return T;
}

// Define 6x6 spatial inertia matrix type
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// 1. Compute 6x6 spatial inertia matrix from mass, CoM, and inertia tensor about CoM
static Matrix6d computeSpatialInertia(double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& I_com) {
    Matrix6d I_spatial = Matrix6d::Zero();
    if (mass <= 0.0) return I_spatial;

    Eigen::Matrix3d E = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d c_skew;
    c_skew <<  0,    -com(2),  com(1),
               com(2),   0,   -com(0),
              -com(1), com(0),   0;

    Eigen::Matrix3d I_origin = I_com + mass * (com.squaredNorm() * E - com * com.transpose());

    I_spatial.block<3,3>(0,0) = I_origin;
    I_spatial.block<3,3>(0,3) = mass * c_skew;
    I_spatial.block<3,3>(3,0) = mass * c_skew.transpose();
    I_spatial.block<3,3>(3,3) = mass * E;
    return I_spatial;
}

// 2. Spatial inertia matrix coordinate transformation
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

// 3. Extract equivalent mass, center of mass position, and inertia tensor about CoM from 6x6 spatial inertia matrix
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

// 4. Recursively accumulate fixed joint subtree (Fixed Joint Welding)
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

// Auxiliary structure: Store node dynamics and coordinate transformation information
struct RobotNodeInfo {
    std::string link_name;
    std::shared_ptr<urdf::Link> link;
    urdf::JointSharedPtr active_joint;
    Eigen::Matrix4d transform;             // Corrected relative fixed pose T_fixed
    int parent_id;
    Matrix6d accumulated_spatial_inertia; // Composite spatial inertia before rotation
    Eigen::Matrix3d R_align;              // Self-axis alignment rotation matrix
};

// Check if the subtree has active joints
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

// 5. Recursive DFS topology extraction function (precisely passes parent_R_align and performs bidirectional coordinate correction)
static void collectActiveJointsDFS(
    const urdf::LinkSharedPtr& current_link,
    int current_parent_id,
    const Eigen::Matrix4d& current_transform,
    const Eigen::Matrix3d& parent_R_align,
    const std::shared_ptr<urdf::ModelInterface>& model,
    std::vector<RobotNodeInfo>& nodes)
{
    if (!current_link) return;

    for (const auto& joint : current_link->child_joints) {
        auto child_link = std::const_pointer_cast<urdf::Link>(model->getLink(joint->child_link_name));
        if (!child_link) continue;

        Eigen::Matrix4d joint_transform = current_transform * urdfPoseToMatrix(joint->parent_to_joint_origin_transform);

        // A. Active joint
        if (joint->type == urdf::Joint::REVOLUTE ||
            joint->type == urdf::Joint::CONTINUOUS ||
            joint->type == urdf::Joint::PRISMATIC) {

            // 1. Compute alignment rotation matrix R_align from the current joint's physical axis to RDL local Z-axis
            Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
            if (axis.norm() < 1e-6) {
                axis << 0, 0, 1;
            } else {
                axis.normalize();
            }

            Eigen::Vector3d z_axis(0, 0, 1);
            Eigen::Matrix3d R_align = Eigen::Matrix3d::Identity();

            if (axis.isApprox(z_axis, 1e-6)) {
                R_align = Eigen::Matrix3d::Identity();
            } else if (axis.isApprox(-z_axis, 1e-6)) {
                R_align = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
            } else {
                R_align = Eigen::Quaterniond::FromTwoVectors(z_axis, axis).toRotationMatrix();
            }

            // 2. Correct the joint fixed pose matrix T_fixed:
            // T_fixed = [parent_R_align^T, 0; 0, 1] * joint_transform * [R_align, 0; 0, 1]
            Eigen::Matrix4d T_parent_inv = Eigen::Matrix4d::Identity();
            T_parent_inv.block<3,3>(0,0) = parent_R_align.transpose();

            Eigen::Matrix4d T_child_align = Eigen::Matrix4d::Identity();
            T_child_align.block<3,3>(0,0) = R_align;

            Eigen::Matrix4d final_transform = T_parent_inv * joint_transform * T_child_align;

            // 3. Collect composite spatial inertia of the active joint and its fixed subtree
            Matrix6d sub_inertia = Matrix6d::Zero();
            accumulateFixedSubtree(child_link, Eigen::Matrix4d::Identity(), model, sub_inertia);

            RobotNodeInfo node_info;
            node_info.link_name = child_link->name;
            node_info.link = child_link;
            node_info.active_joint = joint;
            node_info.transform = final_transform;
            node_info.parent_id = current_parent_id;
            node_info.accumulated_spatial_inertia = sub_inertia;
            node_info.R_align = R_align;

            int new_node_id = nodes.size();
            nodes.push_back(node_info);

            // Recursively traverse child nodes downwards, passing current R_align as parent_R_align for lower-level child nodes
            collectActiveJointsDFS(child_link, new_node_id, Eigen::Matrix4d::Identity(), R_align, model, nodes);
        }
        // B. Fixed joint: penetrate through fixed joint groups
        else if (joint->type == urdf::Joint::FIXED) {
            if (hasActiveJoints(child_link, model)) {
                collectActiveJointsDFS(child_link, current_parent_id, joint_transform, parent_R_align, model, nodes);
            }
        }
    }
}

std::shared_ptr<RobotDynamics> load_RobotDynamics_from_URDF(
    const std::string& urdf_file,
    std::map<std::string, int>& joint_map,
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

    std::vector<RobotNodeInfo> nodes;

    // Node 0: World
    RobotNodeInfo world_node;
    world_node.link_name = "world";
    world_node.link = nullptr;
    world_node.active_joint = nullptr;
    world_node.transform = Eigen::Matrix4d::Identity();
    world_node.parent_id = -1;
    world_node.accumulated_spatial_inertia = Matrix6d::Zero();
    world_node.R_align = Eigen::Matrix3d::Identity();
    nodes.push_back(world_node);

    if (is_floating_base) {
        // --- Floating base mode: forcefully lock Floating Base as Node 1 ---
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

        // Node 1: Base Link (Floating Base / Pelvis)
        RobotNodeInfo base_node;
        base_node.link_name = effective_root->name;
        base_node.link = effective_root;
        base_node.active_joint = nullptr;
        base_node.transform = base_accum_transform;
        base_node.parent_id = 0;
        base_node.R_align = Eigen::Matrix3d::Identity(); // Floating base does not require axis alignment

        Matrix6d base_inertia = Matrix6d::Zero();
        accumulateFixedSubtree(effective_root, Eigen::Matrix4d::Identity(), model, base_inertia);
        base_node.accumulated_spatial_inertia = base_inertia;
        nodes.push_back(base_node);

        // R_align of floating base node (Node 1) is Identity
        collectActiveJointsDFS(effective_root, 1, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity(), model, nodes);

    } else {
        // --- Fixed base mode: R_align of World (Node 0) is Identity ---
        collectActiveJointsDFS(root_link, 0, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity(), model, nodes);
    }

    int num_nodes = nodes.size();
    std::cout << "[URDF Loader] Total Graph Nodes: " << num_nodes
              << " (Floating Base: " << (is_floating_base ? "Yes (Node 1)" : "No") << ")\n";

    // Create internal Graph structure
    std::shared_ptr<Graph> internal_graph(createGraph(num_nodes), deleteGraph);
    for (int i = 1; i < num_nodes; ++i) {
        internal_graph->add_edge(nodes[i].parent_id, i);
    }

    // Populate joint_map (Joint Name -> Node ID)
    joint_map.clear();
    for (int i = 1; i < num_nodes; ++i) {
        if (is_floating_base && i == 1) {
            //joint_map["floating_base"] = 1;
            joint_map[nodes[i].link_name] = 1;
        } else if (nodes[i].active_joint) {
            joint_map[nodes[i].active_joint->name] = i;
        }
    }

    std::shared_ptr<RobotDynamics> robot_dyn(createRobotDynamics(internal_graph.get()), deleteRobotDynamics);

    // Configure model parameters
    for (int i = 0; i < num_nodes; ++i) {
        const auto& node = nodes[i];

        if (i == 0) {
            continue;
        }

        if (is_floating_base && i == 1) {
            // Node 1: floating base
            robot_dyn->set_joint_type(1, JointType::FLOATING, 0.0);
            robot_dyn->set_fixed_transformation(1, node.transform.data(), 16);
        } else {
            auto joint = node.active_joint;

            if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                robot_dyn->set_joint_type(i, JointType::REVOLUTE, 0.0);
            } else if (joint->type == urdf::Joint::PRISMATIC) {
                robot_dyn->set_joint_type(i, JointType::PRISMATIC, 0.0);
            }
            robot_dyn->set_fixed_transformation(i, node.transform.data(), 16);
        }

        // 1. Extract raw mass, center of mass position (com), and inertia tensor (I_com) from the unrotated composite spatial inertia matrix
        double mass = 0.0;
        Eigen::Vector3d com_raw = Eigen::Vector3d::Zero();
        Eigen::Matrix3d I_com_raw = Eigen::Matrix3d::Identity();
        extractInertiaFromSpatial(node.accumulated_spatial_inertia, mass, com_raw, I_com_raw);

        // 2. Critical fix: Perform strict inverse rotation transformation on CoM coordinates and inertia tensor using its own alignment rotation matrix R_align
        // c' = R_align^T * c
        Eigen::Vector3d com_aligned = node.R_align.transpose() * com_raw;
        // I_com' = R_align^T * I_com * R_align
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
    std::cout << "[URDF Loader] Model built successfully with full joint transformation alignment and aligned CoM/Inertia!\n";

    return robot_dyn;
}

} // namespace Utils
} // namespace RDL