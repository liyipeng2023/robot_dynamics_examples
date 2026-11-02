//
// Created by liyip on 2026/11/2.
//

#ifndef ROBOT_DYNAMICS_EXAMPLES_URDF_LOADER_H
#define ROBOT_DYNAMICS_EXAMPLES_URDF_LOADER_H

#pragma once

#include <string>
#include <memory>
#include <robot_dynamics.h>
#include <graph.h>

namespace RDL {
    namespace Utils {

        /**
         * @brief 从 URDF 文件中加载机器人模型，并构建 RDL 的图结构与动力学模型
         *
         * @param urdf_file URDF 文件路径
         * @param out_graph 输出的 RDL 图结构智能指针
         * @return std::shared_ptr<RobotDynamics> 初始化并构建完成的动力学求解器实例
         */
        std::shared_ptr<RobotDynamics> loadRobotDynamicsFromURDF(
            const std::string& urdf_file,
            std::shared_ptr<Graph>& out_graph,
            bool is_floating_base = false
        );

    } // namespace Utils
} // namespace RDL

#endif //ROBOT_DYNAMICS_EXAMPLES_URDF_LOADER_H
