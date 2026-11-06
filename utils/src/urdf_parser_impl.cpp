// urdf_parser_impl.cpp
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32
#include "urdf_parser_impl.h"
#include <iostream>
#include <cmath>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>

namespace RDL {
namespace Utils {

static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<  0,    -v(2),  v(1),
          v(2),  0,    -v(0),
         -v(1),  v(0),  0;
    return S;
}

static Eigen::Matrix3d urdfRotationToMatrix(const urdf::Rotation& r) {
    Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
    if (q.norm() < 1e-12) return Eigen::Matrix3d::Identity();
    q.normalize();
    return q.toRotationMatrix();
}

// Convert URDF Pose (position + quaternion) to 4x4 Homogeneous Transformation Matrix
static Eigen::Matrix4d urdfPoseToMatrix(const urdf::Pose& pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;
    T.block<3, 3>(0, 0) = urdfRotationToMatrix(pose.rotation);
    return T;
}

// 6x6 spatial inertia (angular first, linear second) about the frame origin.
// com / I_com must both be expressed in that same frame (I_com is about the CoM).
static Matrix6d computeSpatialInertia(double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& I_com) {
    Matrix6d I_spatial = Matrix6d::Zero();
    if (mass <= 0.0) return I_spatial;

    const Eigen::Matrix3d E = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d c_skew = skew(com);

    // Parallel axis theorem
    Eigen::Matrix3d I_origin = I_com + mass * (com.squaredNorm() * E - com * com.transpose());

    I_spatial.block<3,3>(0,0) = I_origin;
    I_spatial.block<3,3>(0,3) = mass * c_skew;
    I_spatial.block<3,3>(3,0) = mass * c_skew.transpose();
    I_spatial.block<3,3>(3,3) = mass * E;
    return I_spatial;
}

// Transform a spatial inertia from child frame to parent frame.
// T_child_to_parent = pose of the child frame expressed in the parent frame (R, p).
// I_parent = M * I_child * M^T,  M = [R, p^R; 0, R]   (angular-first convention)
static Matrix6d transformSpatialInertiaToParent(const Matrix6d& I_child, const Eigen::Matrix4d& T_child_to_parent) {
    const Eigen::Matrix3d R = T_child_to_parent.block<3,3>(0,0);
    const Eigen::Vector3d p = T_child_to_parent.block<3,1>(0,3);

    Matrix6d M = Matrix6d::Zero();
    M.block<3,3>(0,0) = R;
    M.block<3,3>(0,3) = skew(p) * R;   // upper-right block (was wrongly placed at lower-left)
    M.block<3,3>(3,3) = R;

    return M * I_child * M.transpose();
}

// Spatial inertia of a single link, expressed in the LINK frame
// (takes <inertial><origin xyz rpy> into account, including its rotation).
static Matrix6d linkSpatialInertia(const urdf::LinkSharedPtr& link) {
    if (!link || !link->inertial) return Matrix6d::Zero();
    const auto& in = link->inertial;
    if (in->mass <= 0.0) return Matrix6d::Zero();

    Eigen::Vector3d com(in->origin.position.x, in->origin.position.y, in->origin.position.z);
    Eigen::Matrix3d I_inertial_frame;
    I_inertial_frame << in->ixx, in->ixy, in->ixz,
                        in->ixy, in->iyy, in->iyz,
                        in->ixz, in->iyz, in->izz;

    // Inertia tensor is given in the inertial frame -> rotate into the link frame
    const Eigen::Matrix3d R_i = urdfRotationToMatrix(in->origin.rotation);
    Eigen::Matrix3d I_com = R_i * I_inertial_frame * R_i.transpose();

    return computeSpatialInertia(in->mass, com, I_com);
}

// Rigid cluster lumping: accumulate spatial inertia of the link and all of its
// fixed-joint-connected descendants, expressed in the frame of the starting link.
static void accumulateRigidCluster(
    const urdf::LinkSharedPtr& link,
    const Eigen::Matrix4d& accum_transform,
    const std::shared_ptr<urdf::ModelInterface>& model,
    Matrix6d& cluster_inertia)
{
    if (!link) return;

    cluster_inertia += transformSpatialInertiaToParent(linkSpatialInertia(link), accum_transform);

    for (const auto& joint : link->child_joints) {
        if (joint->type == urdf::Joint::FIXED) {
            auto child_link = std::const_pointer_cast<urdf::Link>(model->getLink(joint->child_link_name));
            if (child_link) {
                Eigen::Matrix4d child_transform = accum_transform * urdfPoseToMatrix(joint->parent_to_joint_origin_transform);
                accumulateRigidCluster(child_link, child_transform, model, cluster_inertia);
            }
        }
    }
}

static bool isActiveJoint(const urdf::JointSharedPtr& joint) {
    return joint->type == urdf::Joint::REVOLUTE ||
           joint->type == urdf::Joint::CONTINUOUS ||
           joint->type == urdf::Joint::PRISMATIC;
}

// Check if a link subtree contains any active moving joints
static bool hasActiveJoints(const urdf::LinkSharedPtr& link, const std::shared_ptr<urdf::ModelInterface>& model) {
    if (!link) return false;
    for (const auto& joint : link->child_joints) {
        if (isActiveJoint(joint)) return true;
        auto child_link = std::const_pointer_cast<urdf::Link>(model->getLink(joint->child_link_name));
        if (child_link && hasActiveJoints(child_link, model)) return true;
    }
    return false;
}

// Parse topology via DFS; fixed joints are collapsed and their transforms accumulated
// into the downstream active joints, their inertia lumped into the owning active body.
static void parseTopologyAndLumpFixedJointsDFS(
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

        if (isActiveJoint(joint)) {
            Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
            if (axis.norm() < 1e-6) axis << 0, 0, 1;
            else axis.normalize();

            Eigen::Vector3d z_axis(0, 0, 1);
            Eigen::Matrix3d R_align = Eigen::Matrix3d::Identity();

            if (axis.isApprox(z_axis, 1e-6)) {
                R_align = Eigen::Matrix3d::Identity();
            } else if (axis.isApprox(-z_axis, 1e-6)) {
                R_align = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
            } else {
                R_align = Eigen::Quaterniond::FromTwoVectors(z_axis, axis).toRotationMatrix();
            }

            Eigen::Matrix4d T_parent_inv = Eigen::Matrix4d::Identity();
            T_parent_inv.block<3,3>(0,0) = parent_R_align.transpose();

            Eigen::Matrix4d T_child_align = Eigen::Matrix4d::Identity();
            T_child_align.block<3,3>(0,0) = R_align;

            Eigen::Matrix4d final_transform = T_parent_inv * joint_transform * T_child_align;

            Matrix6d cluster_inertia = Matrix6d::Zero();
            accumulateRigidCluster(child_link, Eigen::Matrix4d::Identity(), model, cluster_inertia);

            RobotNodeInfo node_info;
            node_info.node_name = child_link->name;
            node_info.joint_name = joint->name;
            node_info.joint_type = (joint->type == urdf::Joint::PRISMATIC) ? IRJointType::PRISMATIC : IRJointType::REVOLUTE;
            node_info.transform = final_transform;
            node_info.parent_id = current_parent_id;
            node_info.accumulated_spatial_inertia = cluster_inertia;
            node_info.R_align = R_align;

            int new_node_id = static_cast<int>(nodes.size());
            nodes.push_back(node_info);

            parseTopologyAndLumpFixedJointsDFS(child_link, new_node_id, Eigen::Matrix4d::Identity(), R_align, model, nodes);
        }
        else if (joint->type == urdf::Joint::FIXED) {
            // Collapse the fixed joint; its inertia was already lumped into the owning body.
            if (hasActiveJoints(child_link, model)) {
                parseTopologyAndLumpFixedJointsDFS(child_link, current_parent_id, joint_transform, parent_R_align, model, nodes);
            }
        }
        else {
            std::cerr << "[URDFParser] Warning: unsupported joint type for joint '" << joint->name
                      << "' (floating/planar), its subtree is skipped.\n";
        }
    }
}

bool URDFParser::parse(
    const std::string& file_path,
    std::vector<RobotNodeInfo>& nodes,
    bool is_floating_base)
{
    std::shared_ptr<urdf::ModelInterface> model = urdf::parseURDFFile(file_path);
    if (!model) {
        std::cerr << "[URDFParser] Error: Failed to parse URDF file: " << file_path << "\n";
        return false;
    }

    auto root_link = std::const_pointer_cast<urdf::Link>(model->getRoot());
    if (!root_link) return false;

    nodes.clear();

    // Node 0: World Frame
    RobotNodeInfo world_node;
    world_node.node_name = "world";
    world_node.joint_name = "world_joint";
    world_node.joint_type = IRJointType::FIXED;
    world_node.transform = Eigen::Matrix4d::Identity();
    world_node.parent_id = -1;
    nodes.push_back(world_node);

    if (is_floating_base) {
        // Like Pinocchio's free-flyer: the floating base frame is the URDF root link frame.
        // All fixed-joint-connected subtrees of the root are lumped into the base inertia,
        // and traversal starts from the root so no branch is lost even if the root is massless.
        RobotNodeInfo base_node;
        base_node.node_name = root_link->name;
        base_node.joint_name = root_link->name + "_floating_joint";
        base_node.joint_type = IRJointType::FLOATING;
        base_node.transform = Eigen::Matrix4d::Identity();
        base_node.parent_id = 0;
        base_node.R_align = Eigen::Matrix3d::Identity();

        Matrix6d base_inertia = Matrix6d::Zero();
        accumulateRigidCluster(root_link, Eigen::Matrix4d::Identity(), model, base_inertia);
        base_node.accumulated_spatial_inertia = base_inertia;
        nodes.push_back(base_node);

        parseTopologyAndLumpFixedJointsDFS(root_link, 1, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity(), model, nodes);
    } else {
        parseTopologyAndLumpFixedJointsDFS(root_link, 0, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity(), model, nodes);
    }

    return true;
}

} // namespace Utils
} // namespace RDL