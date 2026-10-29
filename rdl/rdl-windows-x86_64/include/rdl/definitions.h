// Copyright (C) 2025 Yipeng Li (yipengli.bice@gmail.com)
//
// This file is part of robot dynamics library.

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <stddef.h>
#include <limits>

namespace RDL {
    // define some commonly used constants
    #define g_acc 9.81

    // modified Denavit-Hartenberg parameters
    struct MDH{
        double alpha; // link twist of ith joint from (i-1)th joint
        double theta; // joint angle offset of ith joint
        double a;     // the distance of ith joint along the (i-1)th x-axis
        double d;     // the link offset between (i-1)th and ith along the ith z-axis (or reused to represent the beta angle associated with the parameter-singularity-free kinematic model proposed by Hayati and Mirmirani)
        double beta; // only used for Hayati's model
        bool using_hayati; // wheather the hayati's model is employed
        MDH(){
            alpha = 0;
            theta = 0;
            a = 0;
            d = 0;
            beta = 0;
            using_hayati = false;
        }
        MDH(double alpha_, double theta_, double a_, double d_, bool using_hayati_ = false, double beta_=0)
        {
            alpha = alpha_;
            theta = theta_;
            a = a_;
            d = d_;
            using_hayati = using_hayati_;
            beta = beta_;
        }
    }; // the modified Denavit-Hartenberg parameters of a link
    
    struct DynamicParam{
        // inerta matrix
        double Ixx;
        double Ixy;
        double Ixz;
        double Iyy;
        double Iyz;
        double Izz;
        // mass
        double m;
        // center-of-mass
        double xc;
        double yc;
        double zc;
        DynamicParam():Ixx(0),Ixy(0),Ixz(0),Iyy(0),Iyz(0),Izz(0),m(0),xc(0),yc(0),zc(0){
        }
        DynamicParam(double Ixx_,double Ixy_,double Ixz_,double Iyy_,double Iyz_,double Izz_, double m_,double xc_,double yc_, double zc_):Ixx(Ixx_),Ixy(Ixy_),Ixz(Ixz_),Iyy(Iyy_),Iyz(Iyz_),Izz(Izz_),m(m_),xc(xc_),yc(yc_),zc(zc_)
        {}
    }; // the dynamic parameters of a link

    enum class FrameDescMethod {
        USING_MDH=1,
        USING_HOMOGENEOUS_MATRIX=2
    }; // the method used to describe the robot's dummy frames

    enum class JointType {
        REVOLUTE=1,     // 1 DOF
        PRISMATIC=2,    // 1 DOF
        FIXED=3,        // 0 DOF
        FLOATING=4,     // 6 DOF
		HELICAL = 5,    // 1 DOF, the screw axis is specified by the user
		CYLINDRICAL = 6,// 2 DOF, the first DOF is the rotation around the joint axis, and the second DOF is the translation along the joint axis
		PLANAR = 7,     // 3 DOF, the first two DOFs are the translation along the plane, and the third DOF is the rotation around the axis perpendicular to the plane
		SPHERICAL = 8   // 3 DOF, the rotation around the x, y, and z axes
    }; // the type of joint

    struct alignas(16) SpatialVector {
        double data[6]{}; // the first three elements represent the angular part, and the last three elements represent the linear part
        SpatialVector() {
            for (double & i : data) {
                i = 0.0;
            }
        }
    };
    
    constexpr size_t ROOT_ID = std::numeric_limits<size_t>::max();

    // frame
    enum class ReferenceFrame
    {
        BODY = 0,
        WORLD = 1,
        BODY_WORLD = 2, // angular part in body frame and linear part in world frame
    };
    // constraint mask
    struct ConstraintMask
    {
        bool rx;
        bool ry;
        bool rz;
        bool x;
        bool y;
        bool z;
        ConstraintMask() : rx(false), ry(false), rz(false), x(false), y(false), z(false) {}
    };
    // structure for loop constraint
    struct LoopConstraint{
        size_t model_A; // the model id of frame A
        size_t frame_A; // the index of frame A in model A
        double R_A[9];  // rotation w.r.t. frame A (column-major)
        double r_A[3];  // translation deviating from frame A
        size_t model_B; // the model id of frame B
        size_t frame_B; // the index of frame B in model B
        double R_B[9];  // rotation w.r.t. frame B (column-major)
        double r_B[3];  // translation deviating from frame B
        ConstraintMask mask;
        //ConstraintType type;
        size_t dof_A;
        size_t dof_B;
        size_t dim_constraint;
    };
    // structure for contact constraint
    struct ContactConstraint {
        size_t model_A;
        size_t frame_A;
        size_t model_B;
        size_t frame_B;
        double position_A[3];   // local position corresponding to frame A
        double normal_A[3];     // the normal vector of the contact surface corresponding to frame A
        double position_B[3];   // local position corresponding to frame B
        double normal_B[3];     // the normal vector of the contact surface corresponding to frame B
        double mu;
        double lambda;
        double penetration;
        size_t dof_A;
        size_t dof_B;
        size_t dim_constraint;
    };
    // robot data
    struct RobotData
    {
        // the state
        double* q = nullptr;        // generalized position
        double* qd = nullptr;       // generalized velocity
        // the acceleration, inertia matrix, Coriolis and Centrifugal matrix, gravitational torque, and input torque
        double* qdd = nullptr;      // acceleration
        double* M = nullptr;        // inertia matrix
        double* C = nullptr;        // Coriolis and Centrifugal matrix
        double* g = nullptr;        // gravitational torque
        double* h = nullptr;        // C*qd+g
        double* tau = nullptr;      // input torque
        double* J = nullptr;        // the Jacobian matrix (tmp variable)

        size_t dim_pos;
        size_t dof_dyn;
    };
 }

#endif