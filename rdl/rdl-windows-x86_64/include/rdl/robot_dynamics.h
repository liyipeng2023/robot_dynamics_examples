// Copyright (C) 2025 Yipeng Li (yipengli.bice@gmail.com)
//
// This file is part of robot dynamics library.

#ifndef ROBOT_DYNAMICS_H
#define ROBOT_DYNAMICS_H

#include <dll_export.h>
#include <graph.h>
#include <definitions.h>

namespace RDL{
    using namespace std;
    // dynamic model of a robot with a specified topology
    class RobotDynamics{
    public:
        virtual ~RobotDynamics()=default;

        /**
         * @brief Set the mdh associated with the link and joint.
         * 
         * @param[in] index_ Index of the link.
         * @param[in] mdh_   Modified Denavit-Hartenberg parameters (in accordance with Craig's notation).
         */
        virtual void set_mdh(size_t index_, const MDH& mdh_)=0; 

        /**
         * @brief Set the fixed transformation matrix of the frame fixed on the link.
         * 
         * @param[in] index_ Index of the link.
         * @param[in] T_     Homogeneous transformation matrix or [angle-axis and position] representation.
         * @param[in] dim_   Dimension of T_: 16 or 6.
         */
        virtual void set_fixed_transformation(size_t index_, const double* T_, size_t dim_) = 0;

        /**
         * @brief Set the dynamic parameters (i.e., the inertia matrix, mass, and center of mass) of the link.
         * 
         * @param[in] index_    Index of the link.
         * @param[in] dyn_param_ The dynamic parameters of the link.
         */
        virtual void set_dyn_param(size_t index_, const DynamicParam& dyn_param_)=0; 

        /**
         * @brief Set the spatial inertia matrix (in accordance with Featherstone's notation) of the link.
         * 
         * @param[in] index_    Index of the link.
         * @param[in] I_        Spatial inertia matrix.
         * @param[in] dim_      Dimension of I_ must be 10.
         */
        virtual void set_spatial_inertia(size_t index_, const double* I_, size_t dim_) = 0;
        
        /**
         * @brief Set the spatial gravitational acceleration of the system.
         * 
         * @param[in] g_    The gravitational acceleration of the system.
         * @param[in] dim_  Dimension of g_ must be 6.
         */
        virtual void set_spatial_gravitational_acc(const double* g_, size_t dim_) = 0;

        /**
         * @brief Set the spatial velocity and spatial acceleration of the base (in the case of floating base).
         *
         * @param[in] vb_   The spatial velocity.
         * @param[in] va_   The spatial acceleration.
         * @param[in] dim_  Dimension of g_ must be 6.
         */
        //virtual void set_base_spatial_motion(const double* vb_, const double* ab_, size_t dim_) = 0;

        /**
         * @brief Set the external spatial force exerted on the link. Please make sure that the external forcs are updated before calling backward_iterations() and forward_dynamics_aba().
         * 
         * @param[in] index_    Index of the link.
         * @param[in] f_ext_    The external spatial force.
         */
        virtual void set_ext_spatial_force(size_t index_, const SpatialVector& f_ext_) = 0;

        /**
         * @brief Set the joint type.
         * 
         * @param index_    Index of the joint.
         * @param type_     Type of the joint.
         * @param h_        This is only used for helical joint.
         */
        virtual void set_joint_type(size_t index_, JointType type_, double h_)=0;

        /**
         * @brief Build the model of the robot.
         * 
         * @note This function should be called after all parameters of the robot are set.
         */
        virtual void build_model() = 0;

        /**
         * @brief Forward iterations in the recursive Newton-Euler algorithm.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[in] qd_       Generalized velocity with dimension equal to the DOF.
         * @param[in] qdd_      Generalized acceleration with dimension equal to the DOF.
         * @param[in] dof_dyn_  DOF of the robot.
         */
        virtual void forward_iterations(const double* qd_, const double* qdd_, size_t dof_dyn_) = 0;  

        /**
         * @brief Backward iterations in the recursive Newton-Euler algorithm.
         * 
         * @note This function should be called after the forward_iterations().
         * @param[out] tau_     The generalized torque with dimension equal to the DOF.
         * @param[in] dof_dyn_  DOF of the robot.
         */
        virtual void backward_iterations(double* tau_, size_t dof_dyn_) = 0;

        /**
         * @brief Forward kinematics for calculating all the transformation matrices.
         * 
         * @param[in] q_    Generalized position of the robot.
         * @param[in] dim_pos_  Dimension of the generalized position. 
         */
        virtual void forward_kinematics(const double* q_, size_t dim_pos_) = 0; 

        /**
         * @brief The forward dynamics (CRBA in Featherstone's book) for calculating the inertia matrix, the Coriolis and centrifugal matrix, and gravitational torque.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[in] qd_       Generalized velocity of the robot (DOF).
         * @param[out] M_       Inertia matrix (DOFxDOF).
         * @param[out] C_       Coriolis and centrifugal matrix (DOFxDOF).
         * @param[out] g_       Gravitational torque (DOF).
         * @param[in] dof_dyn_  DOF of the robot.
         */
        virtual void forward_dynamics_crba(const double* qd_, double* M_, double* C_, double* g_, size_t dof_dyn_) = 0;  

        /**
         * @brief The forward dynamics (CRBA in Featherstone's book) for calculating the inertia matrix.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[out] M_       Inertia matrix (DOFxDOF).
         * @param[in] dof_dyn_  DOF of the robot.
         */
        virtual void forward_dynamics_crba(double* M_, size_t dof_dyn_) = 0;
        
        /**
         * @brief Calculate the Coriolis and centrifugal matrix.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[in] qd_       Generalized velocity of the robot (DOF).
         * @param[out] C_       Coriolis and centrifugal matrix (DOFxDOF).
         * @param[in] dof_dyn_  DOF of the robot.
         */
        virtual void cal_coriolis_matrix(const double* qd_, double* C_, size_t dof_dyn_) = 0;

        /**
         * @brief Calculate the bias force, i.e, C(q,qd)*qd.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[in] qd_       Generalized velocity of the robot (DOF).
         * @param[out] Cqd_     Bias force (DOF).
         * @param[in] dof_dyn_  DOF of the robot.
         */
        virtual void cal_bias_force(const double* qd_, double* Cqd_, size_t dof_dyn_) = 0;

        /**
         * @brief Calculate the gravitational torque.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[out] g_       Gravitational torque (DOF).
         * @param[in] dof_dyn_  DOF of the robot.
         */
        virtual void cal_gravitational_torque(double* g_, size_t dof_dyn_) = 0;

        /**
         * @brief Calculate the nonlinear effect of the system, i.e., C(q,qd)*qd + g
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[in] qd_       Generalized velocity of the robot (DOF).
         * @param[out] h_       Nonlinear effect (DOF).
         * @param[in] dof_dyn_  DOF of the robot.
         */
        virtual void cal_nonlinear_effect(const double* qd_, double* h_, size_t dof_dyn_) = 0;

        /**
         * @brief Forward dynamics (ABA in Featherstone's book) to obtain the generalized acceleration directly.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[in] qd_   Generalized velocity of the robot (DOF)
         * @param[in] tau_  Generalized torque of the robot (DOF)
         * @param[out] qdd_  Generalized acceleration of the robot (DOF)
         * @param[in] dof_dyn_ DOF of the robot
         * @param[in] using_M_inv_only_ False: the ABA to obtain qdd. True: the ABA to solve the equation \f$M(q)x=b\f$.
         */
        virtual void forward_dynamics_aba(const double* qd_, const double* tau_, double* qdd_, size_t dof_dyn_, bool using_M_inv_only_) = 0;

        /**
         * @brief Calculate the jacobian matrix of the frame fixed on the link.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[in] index_    Index of the link.
         * @param[out] jac_     Jacobian matrix.
         * @param[in] rows_     Rows of jac_.
         * @param[in] cols_     Columns of jac_.
         * @param[in] reference_ The reference frame of jac_.
         */
        virtual void cal_jacobian(size_t index_, double* jac_, size_t rows_, size_t cols_,ReferenceFrame reference_) const = 0;

        /**
         * @brief Calculate all the bias accelerations, i.e., Jd*qd.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[in] qd_       Generalized velocity of the robot (DOF).
         * @param[in] dof_dyn_  DOF of the robot.
         */
        virtual void cal_bias_acceleration(const double* qd_, size_t dof_dyn_) = 0;

        /**
         * @brief Calculate the center of mass of the whole robot (especially for the case of floating base).
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[out] com_ The center of mass.
         * @param[in] dim_  Dimension must be 3.
         */
        virtual void cal_center_of_mass(double* com_, size_t dim_) = 0;

        /**
         * @brief Calculate the velocity of the center of mass.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[in] qd_       Generalized velocity of the robot (DOF).
         * @param[in] dim_qd_   DOF of the robot.
         * @param[out] com_vel_ The velocity of the center of mass.
         * @param[in] dim_vel_  Dimension must be 3.
         */
        virtual void cal_center_of_mass_vel(const double* qd_, size_t dim_qd_, double* com_vel_, size_t dim_vel_) = 0;

        /**
         * @brief Calculate the Jacobian matrix of the center of mass.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[out] jac_com_ The Jacobian matrix of the center of mass.
         * @param[in] rows_     Rows of jac_com_.
         * @param[in] cols_     Columns of jac_com_.
         */
        virtual void cal_jacobian_com(double* jac_com_, size_t rows_, size_t cols_) = 0;


        virtual void cal_momentum(double* m_ang_, double* m_lin_, size_t dim_) = 0;
        /**
         * @brief Get the spatial force exerted on the link
         * 
         * @note This function should be called after backward_iterations().
         * @param[in] index_ Index of the link.
         * @param[out] data_ The spatial force exerted on the link.
         * @param[in] dim_   Dimension of spatial force must be 6.
         */
        virtual void get_spatial_force(size_t index_, double* data_, size_t dim_) const = 0;

        /**
         * @brief Get the pose of the frame fixed on the link.
         * 
         * @note This function should be called after the forward_kinematics().
         * @param[in] index_ Index of the frame.
         * @param[out] data_ The pose of the frame.
         * @param[in] dim_   Dimension of the pose must be 6 or 16.
         */
        virtual void get_pose(size_t index_, double* data_, size_t dim_) const = 0;

        /**
         * @brief Get the bias acceleration.
         * 
         * @note This function should be called after the cal_bias_acceleration().
         * @param[in] index_ Index of the frame.
         * @param[out] data_ The bias acceleration.
         * @param[in] dim_   Dimension of the bias acceleration must be 6.
         * @param[in] reference_ The reference frame of the bias acceleration.
         */
        virtual void get_bias_acceleration(size_t index_, double* data_, size_t dim_, ReferenceFrame reference_) const = 0;
        // return the spatial velocity and acceleration of the joint
        /**
         * @brief Get the spatial velocity and acceleration of the frame.
         * 
         * @param[in] index_ Index of the frame.
         * @param[out] v_    Spatial velocity.
         * @param[out] a_    Spatial acceleration.
         * @param[in] dim_   Dimension of the velocity and acceleration must be 6.
         */
        virtual void get_spatial_motion(size_t index_, double* v_, double* a_, size_t dim_) const = 0;

        /**
         * @brief Return the DOF of the robot
         * 
         * @return size_t 
         */
        [[nodiscard]] virtual size_t DOF() const = 0;
        /**
         * @brief Return the number of the bodies.
         * 
         * @return size_t 
         */
        [[nodiscard]] virtual size_t num_body() const = 0;

        /**
         * @brief Return the dimension of the generalized position.
         * 
         * @return size_t 
         */
        [[nodiscard]] virtual size_t dim_position() const = 0;

        /**
         * @brief Return the type of the joint.
         * 
         * @param[in] index_ Index of the joint associated with the link.
         * @return JointType 
         */
        [[nodiscard]] virtual JointType joint_type(size_t index_) const = 0;

        /**
         * @brief Calculated the derivative of the generalized position.
         * 
         * @param[in] qd_in_    The generalized velocity.
         * @param[in] dof_dyn_  DOF of the system. 
         * @param[in] q_in_     The generalized position.
         * @param[out] qd_out_  The derivative of the generalized position (same dim as q_in_).
         * @param[in] dim_pos_  The dimension of the generalized position.
         */

        virtual void cal_position_derivative(const double* qd_in_, size_t dof_dyn_, const double* q_in_, double* qd_out_, size_t dim_pos_) const = 0;

        /**
         * @brief Update the generalized position. If quaternion is used, the quaternion will be updated using quaternion multiplication.
         * 
         * @param[in] qd_       The generalized velocity.
         * @param[in] dof_dyn_  DOF of the system. 
         * @param[in] q_        The generalized position.
         * @param[out] q_out_   The updated generalized position.
         * @param[in] dim_pos_  The dimension of the generalized position.
         * @param[in] step_     The time step.
         */
        virtual void update_position(const double* qd_, size_t dof_dyn_, const double* q_, double* q_out_, size_t dim_pos_, double step_) const = 0;
    };
    //RDL_EXPORT shared_ptr<RobotDynamics> createRobotDynamics(shared_ptr<Graph> graph); // This will cause the application binary interface issue. The compiler should always be consistant with the one which generated the dynamic linking library to avoid such an issue.
    extern "C" {
        RDL_EXPORT RobotDynamics* createRobotDynamics(Graph* graph);
        RDL_EXPORT void  deleteRobotDynamics(RobotDynamics* ptr);
    }
}
#endif