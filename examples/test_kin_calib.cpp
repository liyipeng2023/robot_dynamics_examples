#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32

#include <inverse_kinematics.h>
#include <kinematic_calibration.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <cmath>
#include "./utils/utils.h"

using namespace std;
using namespace RDL;
using namespace Eigen;

double d1 = 0.1273;
double L2 = -0.612;
double L3 = -0.5723;
double d4 = 0.1639;
double d5 = 0.1157;
double d6 = 0.0922;

void forward_kinematics(const Eigen::VectorXd& q, Eigen::Vector3d& t, Eigen::Matrix3d& R)
{
    double q1, q2, q3, q4, q5, q6;
    q1 = q(0); q2 = q(1); q3 = q(2);
    q4 = q(3); q5 = q(4); q6 = q(5);

    R(0, 0) = cos(q6) * (sin(q1) * sin(q5) + cos(q2 + q3 + q4) * cos(q1) * cos(q5)) - sin(q2 + q3 + q4) * cos(q1) * sin(q6);
    R(0, 1) = -sin(q6) * (sin(q1) * sin(q5) + cos(q2 + q3 + q4) * cos(q1) * cos(q5)) - sin(q2 + q3 + q4) * cos(q1) * cos(q6);
    R(0, 2) = cos(q5) * sin(q1) - cos(q2 + q3 + q4) * cos(q1) * sin(q5);

    R(1, 0) = -cos(q6) * (cos(q1) * sin(q5) - cos(q2 + q3 + q4) * cos(q5) * sin(q1)) - sin(q2 + q3 + q4) * sin(q1) * sin(q6);
    R(1, 1) = sin(q6) * (cos(q1) * sin(q5) - cos(q2 + q3 + q4) * cos(q5) * sin(q1)) - sin(q2 + q3 + q4) * cos(q6) * sin(q1);
    R(1, 2) = -cos(q1) * cos(q5) - cos(q2 + q3 + q4) * sin(q1) * sin(q5);

    R(2, 0) = cos(q2 + q3 + q4) * sin(q6) + sin(q2 + q3 + q4) * cos(q5) * cos(q6);
    R(2, 1) = cos(q2 + q3 + q4) * cos(q6) - sin(q2 + q3 + q4) * cos(q5) * sin(q6);
    R(2, 2) = -sin(q2 + q3 + q4) * sin(q5);

    t(0) = d6 * (cos(q5) * sin(q1) - cos(q2 + q3 + q4) * cos(q1) * sin(q5)) + d4 * sin(q1) + L2 * cos(q1) * cos(q2) + d5 * sin(q2 + q3 + q4) * cos(q1) + L3 * cos(q1) * cos(q2) * cos(q3) - L3 * cos(q1) * sin(q2) * sin(q3);
    t(1) = L2 * cos(q2) * sin(q1) - d4 * cos(q1) - d6 * (cos(q1) * cos(q5) + cos(q2 + q3 + q4) * sin(q1) * sin(q5)) + d5 * sin(q2 + q3 + q4) * sin(q1) + L3 * cos(q2) * cos(q3) * sin(q1) - L3 * sin(q1) * sin(q2) * sin(q3);
    t(2) = d1 + L3 * sin(q2 + q3) + L2 * sin(q2) - d5 * (cos(q2 + q3) * cos(q4) - sin(q2 + q3) * sin(q4)) - d6 * sin(q5) * (cos(q2 + q3) * sin(q4) + sin(q2 + q3) * cos(q4));
}

int main()
{
    // Build the six-joint serial-chain topology with link 0 as the fixed base.
    shared_ptr<Graph> graph(createGraph(7), deleteGraph);
    graph->add_edge(0,1); 
    graph->add_edge(1,2); 
    graph->add_edge(2,3);
    graph->add_edge(3,4);
    graph->add_edge(4,5);
    graph->add_edge(5,6);
    
    graph->print();

    // Create the forward/inverse-kinematics model.
    shared_ptr<InverseKinematics> robot_kin(createInverseKinematics(graph.get()),deleteInverseKinematics);
    MDH dh0(0,0,0,0,false);
    MDH dh1(0,0,0,0.1273,false);
    MDH dh2(-M_PI/2,0,0,0,false);
    MDH dh3(0,0,0.612,0,false);
    MDH dh4(0,0,0.5723,0.1639,false);
    MDH dh5(-M_PI/2,0,0,0.1157,false);
    MDH dh6( M_PI/2,0,0,0.0922,false);

    robot_kin->set_mdh(0,dh0);
    robot_kin->set_mdh(1,dh1);
    robot_kin->set_mdh(2,dh2);
    robot_kin->set_mdh(3,dh3);
    robot_kin->set_mdh(4,dh4); 
    robot_kin->set_mdh(5,dh5);
    robot_kin->set_mdh(6,dh6);
    robot_kin->set_joint_type(0,JointType::FIXED, 0);
    robot_kin->set_joint_type(1,JointType::REVOLUTE, 0);
    robot_kin->set_joint_type(2,JointType::REVOLUTE, 0);
    robot_kin->set_joint_type(3,JointType::REVOLUTE, 0);
    robot_kin->set_joint_type(4,JointType::REVOLUTE, 0);
    robot_kin->set_joint_type(5,JointType::REVOLUTE, 0);
    robot_kin->set_joint_type(6,JointType::REVOLUTE, 0);
    Vector3d r_end;
    r_end<<0,0,0;
    robot_kin->set_end_link_position(6, r_end.data(), r_end.size());
    robot_kin->build_model();

    // Define the calibration-board pose in the robot base frame.
    Matrix4d T_board2base;
    T_board2base.setIdentity();
    AngleAxisd aa_board(M_PI/6, Vector3d::UnitY());
    T_board2base.block<3,3>(0,0) = aa_board.toRotationMatrix();
    T_board2base(0,3) = 0.9;
    T_board2base(1,3) = 0.9;
    T_board2base(2,3) = -0.2;
    // Define the camera pose relative to the end effector.
    Matrix4d T_camera2end;
    VectorXd axis = M_PI/2*Vector3d::UnitY() + 0.2*Vector3d::UnitX() + 0.0*Vector3d::UnitZ();
    AngleAxisd aa_camera(axis.norm(), axis.normalized());
    T_camera2end.setIdentity();
    T_camera2end.block<3,3>(0,0) = aa_camera.toRotationMatrix();
    T_camera2end(0,3) = 0.1;
    T_camera2end(1,3) = 0.1;
    T_camera2end(2,3) = 0.2;

    shared_ptr<KinematicCalibration> kin_calib(createKinematicCalibration(graph.get(), CalibrationModel::MDH, PoseErrorReference::BASE_FRAME), deleteKinematicCalibration);

    // Supply perturbed MDH parameters as the calibration initial estimate.
    MDH dh1_(0,0,0,0.1273,false);
    MDH dh2_(-M_PI/2.05,0,0,0,false);
    MDH dh3_(0,0,0.6,0,false);
    MDH dh4_(0,0,0.51, 0.15, false);
    MDH dh5_(-M_PI/1.95,0,0,0.11,false);
    MDH dh6_( M_PI/2.1,0,0,0.0922,false);

    kin_calib->set_mdh(0,dh1_);
    kin_calib->set_mdh(1,dh2_);
    kin_calib->set_mdh(2,dh3_);
    kin_calib->set_mdh(3,dh4_);
    kin_calib->set_mdh(4,dh5_);
    kin_calib->set_mdh(5,dh6_);

    kin_calib->set_joint_type(0, JointType::REVOLUTE);
    kin_calib->set_joint_type(1, JointType::REVOLUTE);
    kin_calib->set_joint_type(2, JointType::REVOLUTE);
    kin_calib->set_joint_type(3, JointType::REVOLUTE);
    kin_calib->set_joint_type(4, JointType::REVOLUTE);
    kin_calib->set_joint_type(5, JointType::REVOLUTE);


    // Define initial estimates for the auxiliary device-to-robot transforms.
	Matrix4d T_err1, T_err2;
    Vector3d a_err1, a_err2;
    a_err1 << -0.31, 0.52, -0.43;
    a_err2 << 0.15, -0.17, 0.23;
	T_err1.setIdentity();
	T_err1.block<3, 3>(0, 0) = AngleAxisd(a_err1.norm(),a_err1.normalized()).toRotationMatrix();
	T_err1(0, 3) = 0.35;
	T_err1(1, 3) = -0.13;
	T_err1(2, 3) = 0.22;
	T_err2.setIdentity();
	T_err2.block<3, 3>(0, 0) = AngleAxisd(a_err2.norm(), a_err2.normalized()).toRotationMatrix();
	T_err2(0, 3) = -0.04;
	T_err2(1, 3) = 0.07;
	T_err2(2, 3) = -0.05;


	Matrix4d T_temp1 = T_board2base* T_err1;
	Matrix4d T_temp2 = T_camera2end*T_err2;

    kin_calib->set_initial_device2end_pose(5, T_temp2.data(), 16);
    kin_calib->set_initial_device2base_pose(T_temp1.data(), 16);
    print("Reference calibration-board pose in the robot base frame:", T_board2base);
    print("Reference camera pose relative to the end effector:", T_camera2end);

    // Define the robot configurations used to generate calibration measurements.
    vector<Vector<double,6>> qs;
    qs.resize(12);
    qs[0] << 0.0000, -1.5708, 0.0000, -1.5708, 1.5708, 0.2000;
    qs[1] << 0.7854, -1.2217, -0.7854, 0.3491, -2.5708, 0.7854;
    qs[2] << -0.7854, -1.9199, 0.8727, -0.5236, 1.1708, -0.7854;
    qs[3] << 1.5708, -1.5708, -1.5708, -1.5708, -1.5708, 1.5708;
    qs[4] << 1.2217, -1.0472, 1.2217, -1.5708, 0.7854, 0.7000;
    qs[5] << 2.3562, -2.0944, -1.0472, 1.5708, 0.7854, 3.1416;
    qs[6] << -1.2217, -2.0944, 2.0944, -1.5708 / 4, -0.7854, -0.9000;
    qs[7] << 0.3491, -0.7854, -0.3491, -0.8000, 1.9199, 0.5236;
    qs[8] << 0.0000, -1.2217, 1.5708, -1.9199 / 4, -1.5708, 1.2000;
    qs[9] << 3.1416, -1.5708, -1.9199, -0.3491, -0.5708, -1.2000;
    qs[10] << 1.0472, -1.7453, -0.1745, -2.0944, 1.0472, 2.0944;
    qs[11] << 0.0000, -0.7854, 0.3491, -1.1345, 0.1708, -2.4000;

    size_t N=12;
    for (size_t i=0; i<N; i++)
    {
        kin_calib->add_joint_pos(qs[i].data(), qs[i].size());
    }
    for (size_t i=0; i<N; i++)
    {
        VectorXd q(6);
        q = qs[i];
        Matrix4d T_end2base, T_camera2board;
        // Compute the camera-to-board pose measurement for this joint configuration.
        robot_kin->set_joint_pos(q.data(), q.size());
        robot_kin->forward_kinematics();
        robot_kin->get_end_effector_pose(6, T_end2base.data(), 16);
        T_camera2board = T_board2base.inverse() * T_end2base * T_camera2end;
        print("End-effector pose in the base frame:", T_end2base);

        kin_calib->add_measurement(5, T_camera2board.data(), 16);
    }
	kin_calib->set_LM_params(0.3, 100, 1e-8, 1, 1);
    kin_calib->set_max_param_epsilon(1.5,1.3);
    // First hold all MDH parameters fixed while estimating the auxiliary frames.
    kin_calib->freeze_parameters(0,FrozenParameter::ALL);
    kin_calib->freeze_parameters(1,FrozenParameter::ALL);
    kin_calib->freeze_parameters(2,FrozenParameter::ALL);
    kin_calib->freeze_parameters(3,FrozenParameter::ALL);
    kin_calib->freeze_parameters(4,FrozenParameter::ALL);
    kin_calib->freeze_parameters(5,FrozenParameter::ALL);
   
    kin_calib->calibrate();
    kin_calib->print_calibration_result();


    // Refine selected MDH parameters while keeping link offsets constrained.
    kin_calib->unfreeze_parameters(1, FrozenParameter::ALL);
    kin_calib->freeze_parameters(1,FrozenParameter::LINK_OFFSET);
    kin_calib->unfreeze_parameters(2, FrozenParameter::ALL);
    kin_calib->freeze_parameters(2, FrozenParameter::LINK_OFFSET);
    kin_calib->unfreeze_parameters(3, FrozenParameter::ALL);
    kin_calib->unfreeze_parameters(4, FrozenParameter::ALL);
    kin_calib->unfreeze_parameters(5, FrozenParameter::ALL);
    kin_calib->freeze_parameters(5, FrozenParameter::LINK_OFFSET);
    
    kin_calib->set_LM_params(0.1, 100, 1e-10, 1, 1);
    
    kin_calib->calibrate();
	kin_calib->print_calibration_result();

    // Compare the calibrated model with the analytical forward-kinematics implementation.
    size_t M = 20;
    std::vector<VectorXd> q_test;
    VectorXd q0(6);
    VectorXd q_step(6);
    q_step << 0.1,0.1,0.1,0.1,-0.1,-0.1;
    q0.setZero();
    for (size_t i = 0; i < M; i++)
    {
        VectorXd q_ = q0 + ((double)i)*q_step;
        q_test.push_back(q_);
    }
    for (size_t i = 0; i < 6; i++)
    {
        MDH mdh_ = kin_calib->get_final_mdh(i);
        robot_kin->set_mdh(i+1, mdh_);
    }
    robot_kin->build_model();
    for (size_t i = 0; i < M; i++)
    {
        robot_kin->set_joint_pos(q_test[i].data(), q_test[i].size());
        robot_kin->forward_kinematics();
        Matrix4d T;
        robot_kin->get_end_effector_pose(6, T.data(), 16);
        Matrix3d R_;
        Vector3d t_;
        forward_kinematics(q_test[i], t_, R_);
        Matrix4d T_;
        T_.setIdentity();
        T_.block<3,3>(0,0) = R_;
        T_.block<3,1>(0,3) = t_;

        print("Calibrated-model end-effector pose:", T);
        print("Analytical end-effector pose:", T_);
    }
    return 0;
}
