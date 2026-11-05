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

static Eigen::Matrix4d urdfPoseToMatrix(const urdf::Pose& pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;
    Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    return T;
}

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
        total_spatial_inertia += transformSpatialInertia(I_local, accum_transform);
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

        if (joint->type == urdf::Joint::REVOLUTE ||
            joint->type == urdf::Joint::CONTINUOUS ||
            joint->type == urdf::Joint::PRISMATIC) {

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

            Matrix6d sub_inertia = Matrix6d::Zero();
            accumulateFixedSubtree(child_link, Eigen::Matrix4d::Identity(), model, sub_inertia);

            RobotNodeInfo node_info;
            node_info.node_name = child_link->name;
            node_info.joint_name = joint->name;
            node_info.joint_type = (joint->type == urdf::Joint::PRISMATIC) ? IRJointType::PRISMATIC : IRJointType::REVOLUTE;
            node_info.transform = final_transform;
            node_info.parent_id = current_parent_id;
            node_info.accumulated_spatial_inertia = sub_inertia;
            node_info.R_align = R_align;

            int new_node_id = nodes.size();
            nodes.push_back(node_info);

            collectActiveJointsDFS(child_link, new_node_id, Eigen::Matrix4d::Identity(), R_align, model, nodes);
        }
        else if (joint->type == urdf::Joint::FIXED) {
            if (hasActiveJoints(child_link, model)) {
                collectActiveJointsDFS(child_link, current_parent_id, joint_transform, parent_R_align, model, nodes);
            }
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

    // Node 0: World
    RobotNodeInfo world_node;
    world_node.node_name = "world";
    world_node.joint_name = "world_joint";
    world_node.joint_type = IRJointType::FIXED;
    world_node.transform = Eigen::Matrix4d::Identity();
    world_node.parent_id = -1;
    nodes.push_back(world_node);

    if (is_floating_base) {
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

        RobotNodeInfo base_node;
        base_node.node_name = effective_root->name;
        base_node.joint_name = effective_root->name + "_floating_joint";
        base_node.joint_type = IRJointType::FLOATING;
        base_node.transform = base_accum_transform;
        base_node.parent_id = 0;
        base_node.R_align = Eigen::Matrix3d::Identity();

        Matrix6d base_inertia = Matrix6d::Zero();
        accumulateFixedSubtree(effective_root, Eigen::Matrix4d::Identity(), model, base_inertia);
        base_node.accumulated_spatial_inertia = base_inertia;
        nodes.push_back(base_node);

        collectActiveJointsDFS(effective_root, 1, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity(), model, nodes);
    } else {
        collectActiveJointsDFS(root_link, 0, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity(), model, nodes);
    }

    return true;
}

} // namespace Utils
} // namespace RDL