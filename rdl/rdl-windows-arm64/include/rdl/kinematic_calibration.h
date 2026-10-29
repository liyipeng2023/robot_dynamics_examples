// Copyright (C) 2025 Yipeng Li (yipengli.bice@gmail.com)
//
// This file is part of robot dynamics library.
#ifndef KINEMATIC_CALIBRATION_H
#define KINEMATIC_CALIBRATION_H

#include <dll_export.h>
#include <graph.h>
#include <definitions.h>

namespace RDL{
    using namespace std;
    enum class FrozenParameter {
        JOINT_OFFSET=1, // joint offset: theta
        LINK_TWIST=2, // link twist: alpha
        LINK_LENGTH=3,     // link length: a
        LINK_OFFSET=4,    // link offset: d
        BETA = 5,
        ALL=6,   // all the mdh parameters (or joint offset and screw-axies)
        DEVICE_ON_END=7, // the transformation from the device [measurement device(e.g., camera, the end-effector of another robot)/target (e.g., chessboard, the base of another robot)] frame to the end-effector frame
        DEVICE_ON_BASE=8, // the transformation from the device [target/measurement device] frame to the base frame
        // reserved for local poe
        SCREW_AXIS=9,
        SCREW_AXIS_ROT=10,
        SCREW_AXIS_POS=11,
        POSE_OFFSET_ROT=12,
        POSE_OFFSET_POS=13,
    }; // the parameter to be frozen during the calibration process

    enum class CalibrationModel{
        MDH = 1,
        LOCAL_POE = 2,
    };// the kinematic model used in calibration

    enum class PoseErrorReference
    {
        BASE_FRAME = 1,
        TARGET_FRAME = 2,
        END_FRAME = 3,
    };// the reference of pose error in calibration
    // kinematic calibration of a robot with a specified topology
    class KinematicCalibration{
    public:
        virtual ~KinematicCalibration()=default;

        // set the MDH parameters
        /**
         * @brief Set the mdh parameters of the link and joint
         * 
         * @param[in] index_ Index of the link
         * @param[in] mdh_   Modified Denavit-Hartenberg parameters
         */
        virtual void set_mdh(size_t index_, const MDH& mdh_)=0; 

        /**
         * @brief Set the auxiliary frame before the frame fixed to the joint. This is usually used for the dummy frames associated with joints and for calibration devices, e.g., camera and chess board.
         * 
         * @param[in] index_ Index of the joint
         * @param[in] pose_  Homogeneous transformation matrix or [angle-axis and position] representation
         * @param[in] dim_   Dimension of pose_ must be 6 or 16.
         */
        virtual void set_auxiliary_frame(size_t index_, const double* pose_, size_t dim_) = 0;

        // set the pose offset for the specified joint index
        /**
         * @brief Set the pose offset before the frame in which screw axis is defined.
         * 
         * @param[in] index_ Index of the joint.
         * @param[in] axis_  Homogeneous transformation matrix or [angle-axis and position] representation
         * @param[in] dim_   Dimension of axis_ must be 6 or 16.
         */
        virtual void set_pose_offset(size_t index_, const double* axis_, size_t dim_)=0;

        /**
         * @brief Set the screw axis (local case).
         * 
         * @param[in] index_  Index of the joint.
         * @param[in] axis_   Screw axis.
         * @param[in] dim_    Dimension must be 6.
         */
        virtual void set_screw_axis(size_t index_, const double* axis_, size_t dim_)=0;

        // set the joint positions
        /**
         * @brief Add a joint positions.
         * 
         * @param[in] vec_ The joint positions.
         * @param[in] dim_ Dimension of joint positions.
         */
        virtual void add_joint_pos(const double* vec_, int dim_)=0; 

        /**
         * @brief Add a pose measurement for the end-effector
         * 
         * @param[in] index_ Index of the end-effector
         * @param[in] pose_  Homogeneous transformation matrix or [angle-axis and position] representation
         * @param[in] dim_   Dimension of pose_ must be 6 or 16
         */
        virtual void add_measurement(size_t index_, const double* pose_, size_t dim_)=0;

        /**
         * @brief Perform kinematic calibration
         * 
         */
        virtual void calibrate()=0; 

        /**
         * @brief Set the LM parameters.
         * 
         * @param[in] lambda_           The damping factor in LM algorithm.
         * @param[in] max_iterations_   The maximum number of iterations in LM algorithm.
         * @param[in] tolerance_        The tolerance in LM algorithm.
         * @param[in] gamma_rot_        The gain in roation update.
         * @param[in] gamma_trans_      The gain in translation update.
         */
        virtual void set_LM_params(double lambda_, size_t max_iterations_, double tolerance_, double gamma_rot_, double gamma_trans_)=0; 

        /**
         * @brief Freeze the specified kinematic parameters.
         * 
         * @param[in] index_ Index of the joint.
         * @param[in] fp_    The specified parameter to be frozen.
         */
        virtual void freeze_parameters(size_t index_, FrozenParameter fp_)=0; 

        /**
         * @brief Unfreeze the specified kinematic parameters.
         * 
         * @param[in] index_ Index of the joint.
         * @param[in] fp_    The specified parameter to be unfrozen.
         */
        virtual void unfreeze_parameters(size_t index_, FrozenParameter fp_)=0; 

        /**
         * @brief Set the joint type (only revolute or prismatic is supported).
         * 
         * @param[in] index_ Index of the joint.
         * @param[in] type_  Joint type.
         */
        virtual void set_joint_type(size_t index_, JointType type_)=0; 

        /**
         * @brief Set the initial pose of the device (e.g., camera or chessboard) fixed on the end-effector.
         * 
         * @param[in] index_ Index of the end-effector.
         * @param[in] pose_  Homogeneous transformation matrix or [angle-axis and position] representation.
         * @param[in] dim_   Dimension of pose_ must be 6 or 16.
         */
        virtual void set_initial_device2end_pose(size_t index_, const double* pose_, size_t dim_)=0;

        // set the initial guess of the pose of device on the base w.r.t. the base frame
        /**
         * @brief Set the initial pose of the device(e.g., chessboard or camera) fixed to the base frame.
         * 
         * @param[in] pose_ Homogeneous transformation matrix or [angle-axis and position] representation.
         * @param[in] dim_  Dimension of pose_ must be 6 or 16
         */
        virtual void set_initial_device2base_pose(const double* pose_, size_t dim_)=0;

        // set the maximum correction epsilon of the angle and distance associated with the MDH, i.e., the maximum configuration error
        virtual void set_max_param_epsilon(double epsilon_angle_, double epsilon_dist_)=0; 

        virtual void get_final_device2base_pose(double* pose_, size_t dim_)=0;
        virtual void get_final_device2end_pose(size_t index_, double* pose_, size_t dim_)=0;

        // get the final MDH parameters for the specified joint index
        virtual MDH get_final_mdh(size_t index_)=0; 

        // get the final pose of the dummy frame w.r.t. its parent
        virtual void get_final_auxiliary_frame(size_t index_, double* pose_, size_t dim_)=0;

        // get the final pose offset of the specified joint 
        virtual void get_final_pose_offset(size_t index_, double* pose_, size_t dim_)=0;

        // get the final screw axis of the specified joint
        virtual void get_final_screw_axis(size_t index_, double* axis_, size_t dim_)=0;

        // get the final joint offset for the specified joint index
        virtual double get_final_joint_offset(size_t index_)=0; 

        // print the calibration result
        virtual void print_calibration_result() = 0; 

        virtual void get_end_pose(size_t index_, double* pose_, size_t dim_)=0; // get the end-effector pose after calibration
        //virtual void test_jac()=0;
    };
    extern "C" {
        RDL_EXPORT KinematicCalibration* createKinematicCalibration(Graph* graph,CalibrationModel cal_model_,PoseErrorReference err_);
        RDL_EXPORT void  deleteKinematicCalibration(KinematicCalibration* ptr);
    }
}

#endif