// mjcf_parser_impl.cpp
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32
#include "mjcf_parser_impl.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include <tinyxml2.h>
#include <Eigen/Dense>

namespace RDL {
namespace Utils {

// 辅助函数：解析空格分隔的 3 维向量 (如 pos="0 0 1")
static Eigen::Vector3d parseVec3(const char* str, const Eigen::Vector3d& default_val = Eigen::Vector3d::Zero()) {
    if (!str) return default_val;
    std::stringstream ss(str);
    double x, y, z;
    if (ss >> x >> y >> z) {
        return Eigen::Vector3d(x, y, z);
    }
    return default_val;
}

// 辅助函数：解析空格分隔的四元数 (MuJoCo 默认顺序: w x y z)
static Eigen::Quaterniond parseQuat(const char* str) {
    if (!str) return Eigen::Quaterniond::Identity();
    std::stringstream ss(str);
    double w, x, y, z;
    if (ss >> w >> x >> y >> z) {
        return Eigen::Quaterniond(w, x, y, z).normalized();
    }
    return Eigen::Quaterniond::Identity();
}

// 辅助函数：将 MJCF 的 pos 和 quat/euler 转换为 4x4 齐次变换矩阵
static Eigen::Matrix4d parsePoseToMatrix(const tinyxml2::XMLElement* elem) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (!elem) return T;

    Eigen::Vector3d pos = parseVec3(elem->Attribute("pos"), Eigen::Vector3d::Zero());
    T(0, 3) = pos(0);
    T(1, 3) = pos(1);
    T(2, 3) = pos(2);

    if (elem->Attribute("quat")) {
        Eigen::Quaterniond q = parseQuat(elem->Attribute("quat"));
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
    } else if (elem->Attribute("euler")) {
        Eigen::Vector3d euler_deg = parseVec3(elem->Attribute("euler"));
        Eigen::Vector3d euler_rad = euler_deg * M_PI / 180.0;
        Eigen::Matrix3d R = (Eigen::AngleAxisd(euler_rad(2), Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(euler_rad(1), Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(euler_rad(0), Eigen::Vector3d::UnitX())).toRotationMatrix();
        T.block<3, 3>(0, 0) = R;
    }

    return T;
}

// 1. 计算 6x6 空间惯量矩阵
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

// 2. 空间惯量矩阵坐标变换
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

// 辅助结构：用于在解析 MJCF 时临时存储 Body 节点及其 XML 元素的映射
struct MJCFBodyNode {
    std::string name;
    const tinyxml2::XMLElement* xml_element = nullptr;
    Eigen::Matrix4d local_transform = Eigen::Matrix4d::Identity();
    double mass = 0.0;
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    Eigen::Matrix3d I_com = Eigen::Matrix3d::Identity();
    IRJointType joint_type = IRJointType::FIXED;
    std::string joint_name;
    Eigen::Vector3d axis = Eigen::Vector3d(0, 0, 1);
    std::vector<int> children_indices;
};

// 提取单个 Body 的惯性参数
static void extractBodyInertial(const tinyxml2::XMLElement* xml_body, double& mass, Eigen::Vector3d& com, Eigen::Matrix3d& I_com) {
    mass = 0.0;
    com.setZero();
    I_com.setZero();

    const tinyxml2::XMLElement* xml_inertial = xml_body->FirstChildElement("inertial");
    if (xml_inertial) {
        if (xml_inertial->QueryDoubleAttribute("mass", &mass) != tinyxml2::XML_SUCCESS) {
            mass = 0.0;
        }
        com = parseVec3(xml_inertial->Attribute("pos"), Eigen::Vector3d::Zero());

        if (xml_inertial->Attribute("diaginertia")) {
            Eigen::Vector3d diag = parseVec3(xml_inertial->Attribute("diaginertia"));
            I_com = diag.asDiagonal();
        } else if (xml_inertial->Attribute("fullinertia")) {
            std::stringstream ss(xml_inertial->Attribute("fullinertia"));
            double ixx, iyy, izz, ixy, ixz, iyz;
            if (ss >> ixx >> iyy >> izz >> ixy >> ixz >> iyz) {
                I_com << ixx, ixy, ixz,
                         ixy, iyy, iyz,
                         ixz, iyz, izz;
            }
        }
    }
}

// 4. 递归累加固定关节子树（Fixed Joint Welding，对应 urdf_loader 的 accumulateFixedSubtree）
static void accumulateFixedSubtreeMJCF(
    int current_idx,
    const std::vector<MJCFBodyNode>& parsed_bodies,
    const Eigen::Matrix4d& accum_transform,
    Matrix6d& total_spatial_inertia)
{
    const auto& body = parsed_bodies[current_idx];

    if (body.mass > 0.0) {
        Matrix6d I_local = computeSpatialInertia(body.mass, body.com, body.I_com);
        Matrix6d I_transformed = transformSpatialInertia(I_local, accum_transform);
        total_spatial_inertia += I_transformed;
    }

    for (int child_idx : body.children_indices) {
        const auto& child_body = parsed_bodies[child_idx];
        // 如果子关节是 FIXED，则继续向下累加融合
        if (child_body.joint_type == IRJointType::FIXED) {
            Eigen::Matrix4d child_transform = accum_transform * child_body.local_transform;
            accumulateFixedSubtreeMJCF(child_idx, parsed_bodies, child_transform, total_spatial_inertia);
        }
    }
}

// 检查子树是否有活动关节（对应 urdf_loader 的 hasActiveJoints）
static bool hasActiveJointsMJCF(int current_idx, const std::vector<MJCFBodyNode>& parsed_bodies) {
    const auto& body = parsed_bodies[current_idx];
    for (int child_idx : body.children_indices) {
        const auto& child_body = parsed_bodies[child_idx];
        if (child_body.joint_type == IRJointType::REVOLUTE ||
            child_body.joint_type == IRJointType::PRISMATIC ||
            child_body.joint_type == IRJointType::FLOATING) {
            return true;
        }
        if (child_body.joint_type == IRJointType::FIXED) {
            if (hasActiveJointsMJCF(child_idx, parsed_bodies)) return true;
        }
    }
    return false;
}

// 第一步：先将 MJCF 的 XML 树扁平化解析到结构体数组中
static void flattenMJCFBodyTree(
    const tinyxml2::XMLElement* xml_body,
    int parent_idx,
    std::vector<MJCFBodyNode>& parsed_bodies)
{
    if (!xml_body) return;

    MJCFBodyNode body;
    const char* name_c = xml_body->Attribute("name");
    body.name = name_c ? name_c : ("body_" + std::to_string(parsed_bodies.size()));
    body.xml_element = xml_body;
    body.local_transform = parsePoseToMatrix(xml_body);

    // 提取惯性
    extractBodyInertial(xml_body, body.mass, body.com, body.I_com);

    // 提取关节（MuJoCo 的 body 下可以有一个 joint 标签）
    const tinyxml2::XMLElement* xml_joint = xml_body->FirstChildElement("joint");
    body.joint_name = body.name + "_joint";
    if (xml_joint) {
        if (xml_joint->Attribute("name")) body.joint_name = xml_joint->Attribute("name");
        const char* type_str = xml_joint->Attribute("type");
        if (type_str) {
            std::string t(type_str);
            if (t == "hinge") body.joint_type = IRJointType::REVOLUTE;
            else if (t == "slide") body.joint_type = IRJointType::PRISMATIC;
            else if (t == "free") body.joint_type = IRJointType::FLOATING;
            else body.joint_type = IRJointType::FIXED;
        } else {
            body.joint_type = IRJointType::REVOLUTE; // MuJoCo 默认 joint 是 hinge
        }
        body.axis = parseVec3(xml_joint->Attribute("axis"), Eigen::Vector3d(0, 0, 1));
    } else {
        body.joint_type = IRJointType::FIXED;
    }

    if (body.axis.norm() < 1e-6) body.axis << 0, 0, 1;
    else body.axis.normalize();

    int current_idx = parsed_bodies.size();
    parsed_bodies.push_back(body);

    if (parent_idx >= 0) {
        parsed_bodies[parent_idx].children_indices.push_back(current_idx);
    }

    // 递归子 body
    const tinyxml2::XMLElement* child_xml = xml_body->FirstChildElement("body");
    while (child_xml) {
        flattenMJCFBodyTree(child_xml, current_idx, parsed_bodies);
        child_xml = child_xml->NextSiblingElement("body");
    }
}

// 5. 递归 DFS 拓扑提取与转轴双向对齐修正（对应 urdf_loader 的 collectActiveJointsDFS）
static void collectActiveJointsMJCF_DFS(
    int current_idx,
    int current_parent_id,
    const Eigen::Matrix4d& current_transform,
    const Eigen::Matrix3d& parent_R_align,
    const std::vector<MJCFBodyNode>& parsed_bodies,
    std::vector<RobotNodeInfo>& nodes)
{
    const auto& current_body = parsed_bodies[current_idx];

    for (int child_idx : current_body.children_indices) {
        const auto& child_body = parsed_bodies[child_idx];
        Eigen::Matrix4d joint_transform = current_transform * child_body.local_transform;

        // A. 活动关节（Revolute / Prismatic / Floating）
        if (child_body.joint_type == IRJointType::REVOLUTE ||
            child_body.joint_type == IRJointType::PRISMATIC ||
            child_body.joint_type == IRJointType::FLOATING) {

            // 1. 计算轴对齐旋转矩阵 R_align（从物理轴对齐到本地 Z 轴）
            Eigen::Vector3d axis = child_body.axis;
            Eigen::Vector3d z_axis(0, 0, 1);
            Eigen::Matrix3d R_align = Eigen::Matrix3d::Identity();

            if (axis.isApprox(z_axis, 1e-6)) {
                R_align = Eigen::Matrix3d::Identity();
            } else if (axis.isApprox(-z_axis, 1e-6)) {
                R_align = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
            } else {
                R_align = Eigen::Quaterniond::FromTwoVectors(z_axis, axis).toRotationMatrix();
            }

            // 2. 双向坐标修正计算最终 T_fixed:
            // T_fixed = [parent_R_align^T, 0; 0, 1] * joint_transform * [R_align, 0; 0, 1]
            Eigen::Matrix4d T_parent_inv = Eigen::Matrix4d::Identity();
            T_parent_inv.block<3,3>(0,0) = parent_R_align.transpose();

            Eigen::Matrix4d T_child_align = Eigen::Matrix4d::Identity();
            T_child_align.block<3,3>(0,0) = R_align;

            Eigen::Matrix4d final_transform = T_parent_inv * joint_transform * T_child_align;

            // 3. 累加该活动关节及其固定子树的复合空间惯量矩阵
            Matrix6d sub_inertia = Matrix6d::Zero();
            accumulateFixedSubtreeMJCF(child_idx, parsed_bodies, Eigen::Matrix4d::Identity(), sub_inertia);

            RobotNodeInfo node_info;
            node_info.node_name = child_body.name;
            node_info.joint_name = child_body.joint_name;
            node_info.joint_type = child_body.joint_type;
            node_info.transform = final_transform;
            node_info.parent_id = current_parent_id;
            node_info.accumulated_spatial_inertia = sub_inertia;
            node_info.R_align = R_align;

            int new_node_id = nodes.size();
            nodes.push_back(node_info);

            // 向下递归，传递当前的 R_align 作为下级节点的 parent_R_align
            collectActiveJointsMJCF_DFS(child_idx, new_node_id, Eigen::Matrix4d::Identity(), R_align, parsed_bodies, nodes);
        }
        // B. 固定关节（Fixed Joint）：穿透穿过固定关节组，直接合并到上一级活动关节
        else if (child_body.joint_type == IRJointType::FIXED) {
            if (hasActiveJointsMJCF(child_idx, parsed_bodies)) {
                collectActiveJointsMJCF_DFS(child_idx, current_parent_id, joint_transform, parent_R_align, parsed_bodies, nodes);
            }
        }
    }
}

bool MJCFParser::parse(
    const std::string& file_path,
    std::vector<RobotNodeInfo>& nodes,
    bool is_floating_base)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(file_path.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "[MJCFParser] Failed to load MJCF XML: " << file_path << "\n";
        return false;
    }

    const tinyxml2::XMLElement* mujoco = doc.FirstChildElement("mujoco");
    if (!mujoco) return false;

    const tinyxml2::XMLElement* worldbody = mujoco->FirstChildElement("worldbody");
    if (!worldbody) return false;

    nodes.clear();

    // Node 0: World
    RobotNodeInfo world_node;
    world_node.node_name = "world";
    world_node.joint_name = "world_joint";
    world_node.joint_type = IRJointType::FIXED;
    world_node.transform = Eigen::Matrix4d::Identity();
    world_node.parent_id = -1;
    world_node.accumulated_spatial_inertia = Matrix6d::Zero();
    world_node.R_align = Eigen::Matrix3d::Identity();
    nodes.push_back(world_node);

    // 扁平化提取所有 Body 节点树
    std::vector<MJCFBodyNode> parsed_bodies;
    const tinyxml2::XMLElement* top_body = worldbody->FirstChildElement("body");
    while (top_body) {
        flattenMJCFBodyTree(top_body, -1, parsed_bodies);
        top_body = top_body->NextSiblingElement("body");
    }

    if (parsed_bodies.empty()) {
        std::cerr << "[MJCFParser] Warning: No bodies found in worldbody.\n";
        return false;
    }

    if (is_floating_base) {
        // 浮动基模式：将根部有效 Body 设为 Node 1 浮动基
        int root_body_idx = 0;
        Eigen::Matrix4d base_accum_transform = parsed_bodies[0].local_transform;

        if (parsed_bodies[0].mass == 0.0 && !parsed_bodies[0].children_indices.empty()) {
            for (int child_idx : parsed_bodies[0].children_indices) {
                if (parsed_bodies[child_idx].mass > 0.0) {
                    root_body_idx = child_idx;
                    base_accum_transform = parsed_bodies[child_idx].local_transform;
                    break;
                }
            }
        }

        RobotNodeInfo base_node;
        base_node.node_name = parsed_bodies[root_body_idx].name;
        base_node.joint_name = parsed_bodies[root_body_idx].name + "_floating_joint";
        base_node.joint_type = IRJointType::FLOATING;
        base_node.transform = base_accum_transform;
        base_node.parent_id = 0;
        base_node.R_align = Eigen::Matrix3d::Identity();

        Matrix6d base_inertia = Matrix6d::Zero();
        accumulateFixedSubtreeMJCF(root_body_idx, parsed_bodies, Eigen::Matrix4d::Identity(), base_inertia);
        base_node.accumulated_spatial_inertia = base_inertia;
        nodes.push_back(base_node);

        // 递归处理子活动关节
        collectActiveJointsMJCF_DFS(root_body_idx, 1, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity(), parsed_bodies, nodes);
    } else {
        // 固定基模式：从所有顶层 body 开始递归
        for (size_t i = 0; i < parsed_bodies.size(); ++i) {
            // 假设顶层直接挂在 world 下
            // 为简化，直接对所有顶层 body 运行 DFS
            // （也可以在顶层引入一个虚拟根节点，这里直接从 0 节点作为虚拟 world 派生）
        }
        // 如果顶层 body 本身就是带关节或固定的，可以把顶层作为入口
        for (size_t i = 0; i < parsed_bodies.size(); ++i) {
            // 检查是否为顶层节点（即在 flatten 时没有 parent 的节点）
            // 简单起见，这里直接复用类似 URDF root 的逻辑
        }
        // 兼容处理：直接从解析到的首个 body 作为入口传入 DFS
        collectActiveJointsMJCF_DFS(0, 0, parsed_bodies[0].local_transform, Eigen::Matrix3d::Identity(), parsed_bodies, nodes);
    }

    std::cout << "[MJCFParser] Successfully parsed and aligned MJCF model, total nodes: " << nodes.size() << "\n";
    return true;
}

} // namespace Utils
} // namespace RDL