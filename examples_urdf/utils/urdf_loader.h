//
// Created by liyip on 2026/11/2.
//

#ifndef ROBOT_DYNAMICS_EXAMPLES_URDF_LOADER_H
#define ROBOT_DYNAMICS_EXAMPLES_URDF_LOADER_H

#pragma once

#include <string>
#include <memory>
#include <map>
#include <robot_dynamics.h>
#include <graph.h>

namespace RDL {
    namespace Utils {

        /**
         * @brief Load model from URDF file, create RDL dynamics and joint name to node ID mapping
         *
         * @param urdf_file URDF path
         * @param joint_map Output mapping from joint name to node ID
         * @param is_floating_base Whether to include a floating base
         * @return std::shared_ptr<RobotDynamics> Return an instance of RobotDynamics
         */
        std::shared_ptr<RobotDynamics> load_RobotDynamics_from_URDF(
            const std::string& urdf_file,
            std::map<std::string, int>& joint_map,
            bool is_floating_base = false
        );

    } // namespace Utils
} // namespace RDL

#endif //ROBOT_DYNAMICS_EXAMPLES_URDF_LOADER_H