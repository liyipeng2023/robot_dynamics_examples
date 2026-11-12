// robot_parser_base.h
#ifndef ROBOT_PARSER_BASE_H
#define ROBOT_PARSER_BASE_H

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace RDL {
    namespace Utils {

        using Matrix6d = Eigen::Matrix<double, 6, 6>;

        // 关节类型抽象
        enum class IRJointType {
            FIXED,
            REVOLUTE,
            PRISMATIC,
            FLOATING,
            CONTINUOUS
        };

        // 与文件格式无关的通用节点信息（Intermediate Representation）
        struct RobotNodeInfo {
            std::string node_name;
            std::string joint_name;
            IRJointType joint_type = IRJointType::FIXED;

            int parent_id = -1;
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity(); // 相对父节点的固定位姿 T_fixed
            Matrix6d accumulated_spatial_inertia = Matrix6d::Zero();  // 复合空间惯量矩阵
            Eigen::Matrix3d R_align = Eigen::Matrix3d::Identity();   // 轴线对齐旋转矩阵
        };

        // 抽象解析器接口
        class IRobotParser {
        public:
            virtual ~IRobotParser() = default;

            /**
             * @brief 解析指定路径的机器人描述文件
             * @param file_path 文件路径
             * @param nodes 解析后输出的通用节点列表
             * @param is_floating_base 是否作为浮动基机器人
             * @return true 解析成功，false 解析失败
             */
            virtual bool parse(
                const std::string& file_path,
                std::vector<RobotNodeInfo>& nodes,
                bool is_floating_base
            ) = 0;
        };

    } // namespace Utils
} // namespace RDL

#endif // ROBOT_PARSER_BASE_H