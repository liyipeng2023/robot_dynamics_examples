// urdf_parser_impl.h
#ifndef URDF_PARSER_IMPL_H
#define URDF_PARSER_IMPL_H

#include "robot_parser_base.h"

namespace RDL {
    namespace Utils {

        class URDFParser : public IRobotParser {
        public:
            URDFParser() = default;
            ~URDFParser() override = default;

            bool parse(
                const std::string& file_path,
                std::vector<RobotNodeInfo>& nodes,
                bool is_floating_base
            ) override;
        };

    } // namespace Utils
} // namespace RDL

#endif // URDF_PARSER_IMPL_H