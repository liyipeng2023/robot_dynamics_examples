// Copyright (C) 2025 Yipeng Li (yipengli.bice@gmail.com)
//
// This file is part of robot dynamics library.
#ifndef DYNAMICS_CALIBRATION_H
#define DYNAMICS_CALIBRATION_H

#include <dll_export.h>
#include <robot_dynamics.h>
#include <definitions.h>

namespace RDL{
    using namespace std;

    class DynamicsCalibration{
    public:
        virtual ~DynamicsCalibration()=default;
        
        // set the joint position, velocity and acceleration for the specified joint index
        /**
         * @brief Add a set of measurements for identifying the dynamic parameters.
         * 
         * @param[in] q_        Generalized position.
         * @param[in] dim_pos_  Dimension of generalized position.
         * @param[in] qd_       Generalized velocity.
         * @param[in] qdd_      Generalized acceleration.
         * @param[in] tau_      Generalized torque.
         * @param[in] dof_dyn_  DOF of the robot.
         */
        virtual void add_measurement(const double* q_, size_t dim_pos_, const double* qd_, const double* qdd_, const double* tau_, size_t dof_dyn_)=0; 

        /**
         * @brief Perform the dynamic parameter calibration.
         * 
         * @note This function should be called after all the measurements are added.
         */
        virtual void calibrate()=0; 

        /**
         * @brief Set the LM parameters for the dynamic parameter calibration.
         * 
         * @param[in] lambda_           The damping factor in LM algorithm.
         * @param[in] max_iterations_   The maximum number of iterations in LM algorithm.
         * @param[in] tolerance_        The tolerance in LM algorithm.
         */
        virtual void set_LM_params(double lambda_, size_t max_iterations_, double tolerance_)=0;

        // get the calibrated dynamic parameters of the specified link index
        /**
         * @brief Get the calibrated dynamic parameters of the link.
         * 
         * @param link_index_ Index of the link.
         * @param data_       The calibrated dynamic parameters (in the form of spatial inertia matrix).
         * @param dim_        Dimension of dynamic parameters must be 10.
         */
        virtual void get_calibrated_param(size_t link_index_, double* data_, size_t dim_)=0;
    };
    extern "C" {
        RDL_EXPORT DynamicsCalibration* createDynamicsCalibration(RobotDynamics* graph);
        RDL_EXPORT void destroyDynamicsCalibration(DynamicsCalibration* instance);
    }
}

#endif