// mjcf_parser_impl.h
#ifndef MJCF_PARSER_IMPL_H
#define MJCF_PARSER_IMPL_H

#include "robot_parser_base.h"

namespace RDL {
    namespace Utils {

        class MJCFParser : public IRobotParser {
        public:
            MJCFParser() = default;
            ~MJCFParser() override = default;

            bool parse(
                const std::string& file_path,
                std::vector<RobotNodeInfo>& nodes,
                bool is_floating_base
            ) override;
        };

    } // namespace Utils
} // namespace RDL

#endif // MJCF_PARSER_IMPL_H