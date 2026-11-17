// mjcf_parser_impl.cpp
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32
#include "mjcf_parser_impl.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <tinyxml2.h>
#include <Eigen/Dense>

namespace RDL {
namespace Utils {

static Eigen::Vector3d parseVec3(const char* str, const Eigen::Vector3d& default_val = Eigen::Vector3d::Zero()) {
    if (!str) return default_val;
    std::stringstream ss(str);
    double x, y, z;
    if (ss >> x >> y >> z) return Eigen::Vector3d(x, y, z);
    return default_val;
}

static Eigen::Vector4d parseVec4(const char* str, const Eigen::Vector4d& default_val = Eigen::Vector4d::Zero()) {
    if (!str) return default_val;
    std::stringstream ss(str);
    double w, x, y, z;
    if (ss >> w >> x >> y >> z) return Eigen::Vector4d(w, x, y, z);
    return default_val;
}

static std::vector<double> parseVecX(const char* str) {
    std::vector<double> res;
    if (!str) return res;
    std::stringstream ss(str);
    double val;
    while (ss >> val) res.push_back(val);
    return res;
}

static Eigen::Quaterniond parseQuat(const char* str) {
    if (!str) return Eigen::Quaterniond::Identity();
    std::stringstream ss(str);
    double w, x, y, z;
    if (ss >> w >> x >> y >> z) {
        Eigen::Quaterniond q(w, x, y, z);
        if (q.norm() > 1e-12) return q.normalized();
    }
    return Eigen::Quaterniond::Identity();
}

static Eigen::Matrix3d parseEulerToRot(const Eigen::Vector3d& euler_rad, const std::string& seq) {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    for (int i = 0; i < 3 && i < static_cast<int>(seq.size()); ++i) {
        char c = seq[i];
        Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
        if (c == 'x' || c == 'X') axis = Eigen::Vector3d::UnitX();
        else if (c == 'y' || c == 'Y') axis = Eigen::Vector3d::UnitY();
        else if (c == 'z' || c == 'Z') axis = Eigen::Vector3d::UnitZ();
        R = R * Eigen::AngleAxisd(euler_rad(i), axis).toRotationMatrix();
    }
    return R;
}

static Eigen::Matrix4d parsePoseToMatrix(const tinyxml2::XMLElement* elem, bool use_radian, const std::string& eulerseq = "xyz") {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (!elem) return T;

    Eigen::Vector3d pos = parseVec3(elem->Attribute("pos"), Eigen::Vector3d::Zero());
    T.block<3, 1>(0, 3) = pos;

    if (elem->Attribute("quat")) {
        T.block<3, 3>(0, 0) = parseQuat(elem->Attribute("quat")).toRotationMatrix();
    } else if (elem->Attribute("euler")) {
        Eigen::Vector3d euler_val = parseVec3(elem->Attribute("euler"));
        Eigen::Vector3d euler_rad = use_radian ? euler_val : (euler_val * M_PI / 180.0);
        T.block<3, 3>(0, 0) = parseEulerToRot(euler_rad, eulerseq);
    } else if (elem->Attribute("axisangle")) {
        Eigen::Vector4d aa = parseVec4(elem->Attribute("axisangle"));
        Eigen::Vector3d axis = aa.head<3>();
        if (axis.norm() > 1e-12) axis.normalize();
        double angle = use_radian ? aa(3) : (aa(3) * M_PI / 180.0);
        T.block<3, 3>(0, 0) = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    } else if (elem->Attribute("xyaxes")) {
        std::vector<double> xy = parseVecX(elem->Attribute("xyaxes"));
        if (xy.size() >= 6) {
            Eigen::Vector3d x_axis(xy[0], xy[1], xy[2]);
            Eigen::Vector3d y_axis(xy[3], xy[4], xy[5]);
            x_axis.normalize();
            y_axis = (y_axis - x_axis.dot(y_axis) * x_axis).normalized();
            Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();
            Eigen::Matrix3d R;
            R << x_axis, y_axis, z_axis;
            T.block<3, 3>(0, 0) = R;
        }
    } else if (elem->Attribute("zaxis")) {
        Eigen::Vector3d z_axis = parseVec3(elem->Attribute("zaxis")).normalized();
        T.block<3, 3>(0, 0) = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), z_axis).toRotationMatrix();
    }

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

static Matrix6d transformSpatialInertia(const Matrix6d& I_child, const Eigen::Matrix4d& T_child_to_parent) {
    Eigen::Matrix3d R = T_child_to_parent.block<3,3>(0,0);
    Eigen::Vector3d p = T_child_to_parent.block<3,1>(0,3);
    Eigen::Matrix3d p_skew;
    p_skew <<  0,    -p(2),  p(1),
               p(2),   0,   -p(0),
              -p(1), p(0),   0;

    Matrix6d X_star = Matrix6d::Zero();
    X_star.block<3,3>(0,0) = R;
    X_star.block<3,3>(0,3) = p_skew * R;
    X_star.block<3,3>(3,3) = R;
    return X_star * I_child * X_star.transpose();
}

struct MJCFJointDefault {
    std::string type = "hinge";
    Eigen::Vector3d axis = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
};

static void parseMJCFDefaults(
    const tinyxml2::XMLElement* elem,
    const std::string& current_class,
    MJCFJointDefault current_def,
    std::unordered_map<std::string, MJCFJointDefault>& default_map)
{
    if (!elem) return;
    std::string cls = current_class;
    if (elem->Attribute("class")) {
        cls = elem->Attribute("class");
    }

    const tinyxml2::XMLElement* j_elem = elem->FirstChildElement("joint");
    if (j_elem) {
        if (j_elem->Attribute("type")) current_def.type = j_elem->Attribute("type");
        if (j_elem->Attribute("axis")) current_def.axis = parseVec3(j_elem->Attribute("axis"), current_def.axis);
        if (j_elem->Attribute("pos"))  current_def.pos  = parseVec3(j_elem->Attribute("pos"), current_def.pos);
    }

    default_map[cls] = current_def;

    const tinyxml2::XMLElement* child_def = elem->FirstChildElement("default");
    while (child_def) {
        parseMJCFDefaults(child_def, cls, current_def, default_map);
        child_def = child_def->NextSiblingElement("default");
    }
}

struct MJCFBodyNode {
    std::string name;
    Eigen::Matrix4d T_body_in_parent_body = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_joint_in_body = Eigen::Matrix4d::Identity();
    Matrix6d body_spatial_inertia = Matrix6d::Zero();
    IRJointType joint_type = IRJointType::FIXED;
    std::string joint_name;
    Eigen::Vector3d axis = Eigen::Vector3d(0, 0, 1);
    std::vector<int> children_indices;
};

static Matrix6d extractBodySpatialInertia(const tinyxml2::XMLElement* xml_body, bool use_radian, const std::string& eulerseq) {
    const tinyxml2::XMLElement* xml_inertial = xml_body->FirstChildElement("inertial");
    if (!xml_inertial) return Matrix6d::Zero();

    double mass = 0.0;
    if (xml_inertial->QueryDoubleAttribute("mass", &mass) != tinyxml2::XML_SUCCESS || mass <= 0.0) {
        return Matrix6d::Zero();
    }

    Eigen::Vector3d com = parseVec3(xml_inertial->Attribute("pos"), Eigen::Vector3d::Zero());
    Eigen::Matrix3d I_com = Eigen::Matrix3d::Zero();

    if (xml_inertial->Attribute("fullinertia")) {
        std::stringstream ss(xml_inertial->Attribute("fullinertia"));
        double ixx, iyy, izz, ixy, ixz, iyz;
        if (ss >> ixx >> iyy >> izz >> ixy >> ixz >> iyz) {
            I_com << ixx, ixy, ixz,
                     ixy, iyy, iyz,
                     ixz, iyz, izz;
        }
    } else {
        Eigen::Matrix3d I_diag = Eigen::Matrix3d::Zero();
        if (xml_inertial->Attribute("diaginertia")) {
            I_diag = parseVec3(xml_inertial->Attribute("diaginertia")).asDiagonal();
        } else if (xml_inertial->Attribute("inertia")) {
            I_diag = parseVec3(xml_inertial->Attribute("inertia")).asDiagonal();
        }
        Eigen::Matrix4d T_inertial = parsePoseToMatrix(xml_inertial, use_radian, eulerseq);
        Eigen::Matrix3d R_inertial = T_inertial.block<3,3>(0,0);
        I_com = R_inertial * I_diag * R_inertial.transpose();
    }

    return computeSpatialInertia(mass, com, I_com);
}

static void accumulateFixedSubtreeMJCF(
    int current_idx,
    const std::vector<MJCFBodyNode>& parsed_bodies,
    const Eigen::Matrix4d& T_curr_body_in_target_frame,
    Matrix6d& total_spatial_inertia)
{
    const auto& body = parsed_bodies[current_idx];
    total_spatial_inertia += transformSpatialInertia(body.body_spatial_inertia, T_curr_body_in_target_frame);

    for (int child_idx : body.children_indices) {
        const auto& child_body = parsed_bodies[child_idx];
        if (child_body.joint_type == IRJointType::FIXED) {
            Eigen::Matrix4d T_child_in_target = T_curr_body_in_target_frame * child_body.T_body_in_parent_body;
            accumulateFixedSubtreeMJCF(child_idx, parsed_bodies, T_child_in_target, total_spatial_inertia);
        }
    }
}

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

static void flattenMJCFBodyTree(
    const tinyxml2::XMLElement* xml_body,
    int parent_idx,
    std::vector<MJCFBodyNode>& parsed_bodies,
    bool use_radian,
    const std::string& eulerseq,
    const std::unordered_map<std::string, MJCFJointDefault>& default_map,
    const std::string& inherited_childclass,
    bool is_floating_base)
{
    if (!xml_body) return;

    std::string active_class = inherited_childclass;
    if (xml_body->Attribute("childclass")) {
        active_class = xml_body->Attribute("childclass");
    }

    const char* name_c = xml_body->Attribute("name");
    std::string base_body_name = name_c ? name_c : ("body_" + std::to_string(parsed_bodies.size()));
    Eigen::Matrix4d T_body_raw = parsePoseToMatrix(xml_body, use_radian, eulerseq);
    Matrix6d I_body_raw = extractBodySpatialInertia(xml_body, use_radian, eulerseq);

    struct RawJointSpec {
        std::string name;
        IRJointType type;
        Eigen::Vector3d axis;
        Eigen::Vector3d pos;
    };
    std::vector<RawJointSpec> joints;

    const tinyxml2::XMLElement* xml_freejoint = xml_body->FirstChildElement("freejoint");
    if (xml_freejoint && is_floating_base) {
        RawJointSpec js;
        const char* jname = xml_freejoint->Attribute("name");
        js.name = jname ? jname : (base_body_name + "_floating_joint");
        js.type = IRJointType::FLOATING;
        js.axis = Eigen::Vector3d(0, 0, 1);
        js.pos  = Eigen::Vector3d::Zero();
        joints.push_back(js);
    }

    const tinyxml2::XMLElement* xml_joint = xml_body->FirstChildElement("joint");
    while (xml_joint) {
        std::string j_class = active_class;
        if (xml_joint->Attribute("class")) j_class = xml_joint->Attribute("class");

        MJCFJointDefault j_def;
        auto it = default_map.find(j_class);
        if (it != default_map.end()) j_def = it->second;

        std::string t = xml_joint->Attribute("type") ? xml_joint->Attribute("type") : j_def.type;
        if (t == "free" && !is_floating_base) {
            xml_joint = xml_joint->NextSiblingElement("joint");
            continue;
        }

        RawJointSpec js;
        const char* jname = xml_joint->Attribute("name");
        js.name = jname ? jname : (base_body_name + "_joint_" + std::to_string(joints.size()));

        if (t == "hinge") js.type = IRJointType::REVOLUTE;
        else if (t == "slide") js.type = IRJointType::PRISMATIC;
        else if (t == "free") js.type = IRJointType::FLOATING;
        else js.type = IRJointType::FIXED;

        js.axis = parseVec3(xml_joint->Attribute("axis"), j_def.axis);
        if (js.axis.norm() < 1e-6) js.axis << 0, 0, 1;
        else js.axis.normalize();

        js.pos = parseVec3(xml_joint->Attribute("pos"), j_def.pos);
        if (js.type == IRJointType::FLOATING) js.pos.setZero();

        joints.push_back(js);
        xml_joint = xml_joint->NextSiblingElement("joint");
    }

    int last_created_idx = -1;
    if (joints.empty()) {
        MJCFBodyNode node;
        node.name = base_body_name;
        node.joint_name = base_body_name + "_fixed_joint";
        node.joint_type = IRJointType::FIXED;
        node.T_body_in_parent_body = T_body_raw;
        node.T_joint_in_body = Eigen::Matrix4d::Identity();
        node.body_spatial_inertia = I_body_raw;

        last_created_idx = static_cast<int>(parsed_bodies.size());
        parsed_bodies.push_back(node);
        if (parent_idx >= 0) parsed_bodies[parent_idx].children_indices.push_back(last_created_idx);
    } else {
        int curr_parent = parent_idx;
        for (size_t k = 0; k < joints.size(); ++k) {
            bool is_last = (k + 1 == joints.size());
            MJCFBodyNode node;
            node.name = is_last ? base_body_name : (base_body_name + "_composite_" + std::to_string(k));
            node.joint_name = joints[k].name;
            node.joint_type = joints[k].type;
            node.axis = joints[k].axis;

            if (k == 0) {
                node.T_body_in_parent_body = T_body_raw;
            } else {
                node.T_body_in_parent_body = Eigen::Matrix4d::Identity();
            }

            Eigen::Matrix4d T_j = Eigen::Matrix4d::Identity();
            T_j.block<3,1>(0,3) = joints[k].pos;
            node.T_joint_in_body = T_j;

            node.body_spatial_inertia = is_last ? I_body_raw : Matrix6d::Zero();

            int idx = static_cast<int>(parsed_bodies.size());
            parsed_bodies.push_back(node);
            if (curr_parent >= 0) parsed_bodies[curr_parent].children_indices.push_back(idx);
            curr_parent = idx;
            last_created_idx = idx;
        }
    }

    const tinyxml2::XMLElement* child_xml = xml_body->FirstChildElement("body");
    while (child_xml) {
        flattenMJCFBodyTree(child_xml, last_created_idx, parsed_bodies, use_radian, eulerseq, default_map, active_class, is_floating_base);
        child_xml = child_xml->NextSiblingElement("body");
    }
}

static void parseMJCFBodyDFS(
    int body_idx,
    int current_parent_id,
    const Eigen::Matrix4d& T_parent_body_in_active_joint,
    const Eigen::Matrix3d& parent_R_align,
    const std::vector<MJCFBodyNode>& parsed_bodies,
    std::vector<RobotNodeInfo>& nodes)
{
    if (body_idx < 0 || body_idx >= static_cast<int>(parsed_bodies.size())) return;
    const auto& body = parsed_bodies[body_idx];

    Eigen::Matrix4d T_curr_body_in_active = T_parent_body_in_active_joint * body.T_body_in_parent_body;

    if (body.joint_type == IRJointType::REVOLUTE ||
        body.joint_type == IRJointType::PRISMATIC ||
        body.joint_type == IRJointType::FLOATING)
    {
        Eigen::Matrix4d T_curr_joint_in_active = T_curr_body_in_active * body.T_joint_in_body;

        Eigen::Matrix3d R_align = Eigen::Matrix3d::Identity();
        if (body.joint_type != IRJointType::FLOATING) {
            Eigen::Vector3d axis = body.axis;
            Eigen::Vector3d z_axis(0, 0, 1);
            if (axis.isApprox(z_axis, 1e-6)) {
                R_align = Eigen::Matrix3d::Identity();
            } else if (axis.isApprox(-z_axis, 1e-6)) {
                R_align = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
            } else {
                R_align = Eigen::Quaterniond::FromTwoVectors(z_axis, axis).toRotationMatrix();
            }
        }

        Eigen::Matrix4d T_parent_inv = Eigen::Matrix4d::Identity();
        T_parent_inv.block<3,3>(0,0) = parent_R_align.transpose();

        Eigen::Matrix4d T_child_align = Eigen::Matrix4d::Identity();
        T_child_align.block<3,3>(0,0) = R_align;

        Eigen::Matrix4d final_transform = T_parent_inv * T_curr_joint_in_active * T_child_align;
        if (body.joint_type == IRJointType::FLOATING) {
            final_transform = Eigen::Matrix4d::Identity();
        }

        Eigen::Matrix4d T_body_in_curr_joint = Eigen::Matrix4d::Identity();
        T_body_in_curr_joint.block<3,1>(0,3) = -body.T_joint_in_body.block<3,1>(0,3);

        Matrix6d sub_inertia = Matrix6d::Zero();
        accumulateFixedSubtreeMJCF(body_idx, parsed_bodies, T_body_in_curr_joint, sub_inertia);

        RobotNodeInfo node_info;
        node_info.node_name = body.name;
        node_info.joint_name = body.joint_name;
        node_info.joint_type = body.joint_type;
        node_info.transform = final_transform;
        node_info.parent_id = current_parent_id;
        node_info.accumulated_spatial_inertia = sub_inertia;
        node_info.R_align = R_align;

        int new_node_id = static_cast<int>(nodes.size());
        nodes.push_back(node_info);

        for (int child_idx : body.children_indices) {
            parseMJCFBodyDFS(child_idx, new_node_id, T_body_in_curr_joint, R_align, parsed_bodies, nodes);
        }
    }
    else if (body.joint_type == IRJointType::FIXED) {
        if (hasActiveJointsMJCF(body_idx, parsed_bodies)) {
            for (int child_idx : body.children_indices) {
                parseMJCFBodyDFS(child_idx, current_parent_id, T_curr_body_in_active, parent_R_align, parsed_bodies, nodes);
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

    bool use_radian = false;
    std::string eulerseq = "xyz";
    const tinyxml2::XMLElement* compiler = mujoco->FirstChildElement("compiler");
    if (compiler) {
        if (compiler->Attribute("angle") && std::string(compiler->Attribute("angle")) == "radian") {
            use_radian = true;
        }
        if (compiler->Attribute("eulerseq")) {
            eulerseq = compiler->Attribute("eulerseq");
        }
    }

    std::unordered_map<std::string, MJCFJointDefault> default_map;
    MJCFJointDefault root_def;
    default_map[""] = root_def;
    const tinyxml2::XMLElement* def_elem = mujoco->FirstChildElement("default");
    while (def_elem) {
        parseMJCFDefaults(def_elem, "", root_def, default_map);
        def_elem = def_elem->NextSiblingElement("default");
    }

    const tinyxml2::XMLElement* worldbody = mujoco->FirstChildElement("worldbody");
    if (!worldbody) return false;

    nodes.clear();

    RobotNodeInfo world_node;
    world_node.node_name = "world";
    world_node.joint_name = "world_joint";
    world_node.joint_type = IRJointType::FIXED;
    world_node.transform = Eigen::Matrix4d::Identity();
    world_node.parent_id = -1;
    world_node.accumulated_spatial_inertia = Matrix6d::Zero();
    world_node.R_align = Eigen::Matrix3d::Identity();
    nodes.push_back(world_node);

    std::vector<MJCFBodyNode> parsed_bodies;
    std::vector<int> top_level_indices;
    const tinyxml2::XMLElement* top_body = worldbody->FirstChildElement("body");
    while (top_body) {
        int top_idx = static_cast<int>(parsed_bodies.size());
        top_level_indices.push_back(top_idx);
        flattenMJCFBodyTree(top_body, -1, parsed_bodies, use_radian, eulerseq, default_map, "", is_floating_base);
        top_body = top_body->NextSiblingElement("body");
    }

    if (parsed_bodies.empty()) return false;

    if (is_floating_base) {
        int root_body_idx = 0;
        while (parsed_bodies[root_body_idx].body_spatial_inertia.norm() < 1e-12 &&
               parsed_bodies[root_body_idx].joint_type == IRJointType::FIXED &&
               parsed_bodies[root_body_idx].children_indices.size() == 1)
        {
            int child_idx = parsed_bodies[root_body_idx].children_indices[0];
            if (parsed_bodies[child_idx].joint_type == IRJointType::FLOATING) {
                root_body_idx = child_idx;
            } else {
                break;
            }
        }

        if (parsed_bodies[root_body_idx].joint_type == IRJointType::FLOATING) {
            parseMJCFBodyDFS(root_body_idx, 0, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity(), parsed_bodies, nodes);
        } else {
            RobotNodeInfo base_node;
            base_node.node_name = parsed_bodies[root_body_idx].name;
            base_node.joint_name = parsed_bodies[root_body_idx].name + "_floating_joint";
            base_node.joint_type = IRJointType::FLOATING;
            base_node.transform = Eigen::Matrix4d::Identity();
            base_node.parent_id = 0;
            base_node.R_align = Eigen::Matrix3d::Identity();

            Matrix6d base_inertia = Matrix6d::Zero();
            accumulateFixedSubtreeMJCF(root_body_idx, parsed_bodies, Eigen::Matrix4d::Identity(), base_inertia);
            base_node.accumulated_spatial_inertia = base_inertia;
            nodes.push_back(base_node);

            for (int child_idx : parsed_bodies[root_body_idx].children_indices) {
                parseMJCFBodyDFS(child_idx, 1, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity(), parsed_bodies, nodes);
            }
        }
    } else {
        for (int top_idx : top_level_indices) {
            parseMJCFBodyDFS(top_idx, 0, Eigen::Matrix4d::Identity(), Eigen::Matrix3d::Identity(), parsed_bodies, nodes);
        }
    }

    std::cout << "[MJCFParser] Successfully parsed and aligned MJCF model, total nodes: " << nodes.size() << "\n";
    return true;
}

} // namespace Utils
} // namespace RDL