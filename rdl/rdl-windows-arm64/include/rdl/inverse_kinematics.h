// Copyright (C) 2025 Yipeng Li (yipengli.bice@gmail.com)
//
// This file is part of robot dynamics library.
#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <dll_export.h>
#include <graph.h>  
#include <definitions.h>

namespace RDL{
    using namespace std;
    // robot kinematics solver for a robot with a specified topology
    class InverseKinematics{
    public:
        virtual ~InverseKinematics();

        /**
         * @brief Get the result of the inverse kinematics.
         * 
         * @param[out] vec_ The joint positions.
         * @param[in] dim_  The dimension of the joint position.
         */
        virtual void get_joint_pos(double* vec_, size_t dim_) const =0; 

        /**
         * @brief Get the pose of end-effector.
         * 
         * @note This function should be called after forward_kinematics().
         * @param[in] index_ Index of the end-effector.
         * @param[out] pose_ Homogeneous transformation matrix or [angle-axis and position] representation.
         * @param[in] dim_   Dimension must be 6 or 16.
         */
        virtual void get_end_effector_pose(size_t index_, double* pose_, size_t dim_) const =0;

        /**
         * @brief Get the Jacobian matrix of the frame fixed on the end-effector.
         * 
         * @note This function should be called after cal_jacobian().
         * @param[out] J_   Jacobian matrix.
         * @param[in] rows_ Rows of the Jacobian matrix.
         * @param[in] cols_ Columns of the Jacobian matrix.
         */
        virtual void get_jacobian(double* J_, size_t rows_, size_t cols_) const =0; 

        /**
         * @brief Set the initial joint positions for the inverse kinematics.
         * 
         * @param[in] vec_ The initial joint positions.
         * @param[in] dim_ The dimension of the initial joint positions.
         */
        virtual void set_joint_pos(const double* vec_, size_t dim_)=0; 

        /**
         * @brief Set the joint limits of the robot.
         * 
         * @param[in] q_min_ Lower bounds of the joint positions.
         * @param[in] q_max_ Upper bounds of the joint positions.
         * @param[in] dim_   Dimension of the joint positions.
         */
        virtual void set_joint_limits(const double* q_min_, const double* q_max_, size_t dim_)=0; 

        /**
         * @brief Set the LM (Levenberg-Marquardt) parameters.
         * 
         * @param[in] lambda_  The damping factor in LM algorithm.
         * @param[in] max_iterations_ The maximun number of iterations in LM algorithm.
         * @param[in] tolerance_ The tolerance of convergence in LM algorithm.
         * @param[in] gain_ The gain used in the update of joint position.
         */
        virtual void set_LM_params(double lambda_, size_t max_iterations_, double tolerance_, double gain_)=0; 

        // set the MDH parameters
        /**
         * @brief Set the MDH parameters of the robot.
         * 
         * @param[in] index_ Index of the link.
         * @param[in] mdh_ The MDH parameters associated with the transformation from frame index_ to frame index_-1.
         */
        virtual void set_mdh(size_t index_, const MDH& mdh_)=0; 

        /**
         * @brief Set the fixed transformation matrix of the frame fixed on the link.
         * 
         * @param[in] index_ Index of the link.
         * @param[in] T_     Homogeneous transformation matrix or [angle-axis and position] representation.
         * @param[in] dim_   Dimension of T_: 16 or 6.
         */
        virtual void set_fixed_transform(size_t index_, const double* T_, size_t dim_) = 0;

        /**
         * @brief Set the target pose of end-effector.
         * 
         * @param[in] index_ Index of the end-effector
         * @param[in] pose_  Homogeneous transformation matrix or [angle-axis and position] representation.
         * @param[in] dim_   Dimension must be 6 or 16.
         */
        virtual void set_target_end_effector_pose(size_t index_, const double* pose_, size_t dim_)=0;

        /**
         * @brief Set the position of the far-end of the end-effector w.r.t. its frame.
         * 
         * @param[in] index_ Index of the end-effector
         * @param[in] vec_   The position w.r.t. its frame.
         * @param[in] dim_   The dimension must be 3.
         */
        virtual void set_end_link_position(size_t index_, const double* vec_, size_t dim_) = 0;

        /**
         * @brief Set the verbose in the numerical iteration.
         * 
         * @param[in] verbose_ False: the intermediate steps won't display.
         */
        virtual void set_verbose(bool verbose_) = 0; 

        /**
         * @brief Set the joint type.
         * 
         * @param index_    Index of the joint.
         * @param type_     Type of the joint.
         * @param h_        This is only used for helical joint.
         */
        virtual void set_joint_type(size_t index_, JointType type_, double h_) = 0;

        /**
         * @brief Build the model.
         * 
         */
        virtual void build_model() = 0;

        /**
         * @brief Set the regularization gain in the objective functioin. The panelties on joint limits (i.e., 1/2||q-q_lower||^2 and 1/2||q-q_upper||^2) and the path in joint space (i.e., 1/2||q-q_0||^2) are taken into consideration.
         * 
         * @param[in] gain_joint_limit_  The factor of panelty on joint limits.
         * @param[in] gain_shortest_path_ Th factor of panelty on path in joint space.
         */
		virtual void set_regularization_gain(double gain_joint_limit_, double gain_shortest_path_) = 0;

        /**
         * @brief Calculate the foward kinematics.
         * 
         */
        virtual void forward_kinematics() = 0;  

        /**
         * @brief Perform the numerical interations of the inverse kinematics.
         * 
         */
        virtual void inverse_kinematics() = 0; 

        /**
         * @brief Calculate the Jacobian matrix of the end-effectors.
         * 
         * @note This function should be called after forward_kinematics();
         */
        virtual void cal_jacobian() = 0;  
    };
    extern "C" {
        RDL_EXPORT InverseKinematics* createInverseKinematics(Graph* graph);
        RDL_EXPORT void  deleteInverseKinematics(InverseKinematics* ptr);
    }
}

#endif