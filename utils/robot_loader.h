// robot_loader.h
#ifndef ROBOT_LOADER_H
#define ROBOT_LOADER_H

#include <string>
#include <memory>
#include <map>
#include <robot_dynamics.h>
#include <graph.h>

namespace RDL {
    namespace Utils {

        enum class RobotFileType {
            URDF,
            MJCF
        };

        /**
         * @brief 统一机器人模型加载函数，支持 URDF、SDF、MJCF、DH 等格式
         */
        std::shared_ptr<RobotDynamics> load_RobotDynamics(
            const std::string& file_path,
            std::map<std::string, int>& joint_map,
            RobotFileType file_type = RobotFileType::URDF,
            bool is_floating_base = false
        );

    } // namespace Utils
} // namespace RDL

#endif // ROBOT_LOADER_H