// Copyright (C) 2025 Yipeng Li (yipengli.bice@gmail.com)
//
// This file is part of robot dynamics library.

#ifndef CONSTRAINED_DYNAMICS_SOLVER_H
#define CONSTRAINED_DYNAMICS_SOLVER_H

#include <dll_export.h>
#include <definitions.h>
#include <robot_dynamics.h>

namespace RDL {
    using namespace std;
    class ConstrainedDynamicsSolver {
    public:
        virtual ~ConstrainedDynamicsSolver() = default;
        
        /**
         * @brief Set the constraints object.
         * 
         * @param[in] constraints_ The loop constraints.
         * @param[in] num_constraints_ The number of loop constraints.
         */
        virtual void set_constraints(LoopConstraint* constraints_, size_t num_constraints_) = 0;

        /**
         * @brief Set the contact constraints object.
         * 
         * @param[in] constraints_ The contact constraints.
         * @param[in] num_constraints_ The number of contact constraints.
         */
        virtual void set_constraints(ContactConstraint* constraints_, size_t num_constraints_) = 0;

        /**
         * @brief Update the constraint parameters.
         * 
         * @param[in] constraint_
         * @param[in] constraint_id_ 
         */
        virtual void update_constraint(const LoopConstraint& constraint_, size_t constraint_id_) = 0;

        /**
         * @brief Update the contact constraint parameters.
         * 
         * @param[in] constraint_ The updated contact constraint.
         * @param[in] constraint_id_ The ID of the constraint to update.
         */
        virtual void update_constraint(const ContactConstraint& constraint_, size_t constraint_id_) = 0;

        /**
         * @brief Set the parameters object.
         * 
         * @param[in] beta_ The stabilization parameter for the constraint velocity.
         * @param[in] alpha_ The stabilization parameter for the constraint position.
         * @param[in] kv_ The gain for the constraint velocity projection.
         * @param[in] kp_ The gain for the constraint position projection.
         * @param[in] damping_factor_ The damping factor.   
         */
        virtual void set_parameters(double beta_, double alpha_, double kv_, double kp_, double damping_factor_) = 0;

        /**
         * @brief Activate or deactivate a constraint.
         * 
         * @param[in] constraint_id_ The ID of the constraint to activate or deactivate.
         * @param[in] is_active_ Whether the constraint should be active.
         */
        virtual void activate_constraint(size_t constraint_id_, bool is_active_) = 0;

        /**
         * @brief Solve the constrained dynamics.
         * 
         * @param[in] robot_dyns_ The robot dynamics associated with the constraints.
         * @param[in] robot_datas_ The robot data associated with the robot dynamics.
         * @param[in] num_robots_   The number of robots.
         */
        virtual void solve(RobotDynamics** robot_dyns_, RobotData** robot_datas_, size_t num_robots_) = 0;

        // this function will be deprecated in the future.
        virtual void project_constraint(RobotDynamics** robot_dyns_, RobotData** robot_datas_, size_t num_robots_) = 0;

        virtual void get_constraint_err(double* constraint_, double* constraint_derivative_, double* constraint_dderivative_, size_t dof_c_) = 0;

        virtual void get_lambda(double* lambda_, size_t dof_c_) = 0;
    };

    // method for manage the virtual class
    extern "C" {
        
        //RDL_EXPORT ConstrainedDynamicsSolver* createConstrainedDynamicsSolver(double beta_, double alpha_, double k2_, double k1_, size_t max_num_constraint_, size_t max_dim_constraint_, size_t total_dof_dyn_);
        RDL_EXPORT ConstrainedDynamicsSolver* createConstrainedDynamicsSolver(size_t num_models_, const size_t* total_dof_dyn_, size_t max_num_constraint_, size_t max_dim_constraint_);
        RDL_EXPORT void  deleteConstrainedDynamicsSolver(ConstrainedDynamicsSolver* ptr);
    }
}

#endif