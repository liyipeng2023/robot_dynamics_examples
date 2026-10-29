#ifndef MATH_UTILS_H
#define MATH_UTILS_H
#include <dll_export.h>
#include <definitions.h>

namespace RDL{
    /**
     * @brief Calculate the transfomation matrix from modified Denavit-Hartenberg (in accordance with Craig's notation and with Hayati's model) parameters.
     * 
     * @param[in] mdh           Modified Denavit-Hartenberg parameters.
     * @param[in] joint_type    Joint type (only support for revolute and prismatic).
     * @param[in] q             Joint position.
     * @param[out] data         Homogeneous transformation matrix.
     * @param[in] dim           The dimension must be 16.
     */
    RDL_EXPORT void mdh_to_transformation(const MDH& mdh, const JointType& joint_type, double q, double* data, size_t dim=16);

}

#endif