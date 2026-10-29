#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32

#include <graph.h>
#include <robot_dynamics.h>
#include <constrained_dynamics_solver.h>
#include <iostream>
//#define EIGEN_DONT_PARALLELIZE
#include <Eigen/Dense>
#define EIGEN_USE_THREADS
#include <Eigen/Core>
#include <cmath>
#include <math_utils.h>
#include "./utils/utils.h"
#include "./utils/robot_data_allocator.h"
#include "./utils/robot_dynamics_integrator.h"
#include "./utils//data_recorder.h"

using namespace std;
using namespace RDL;
using namespace Eigen;

int main()
{
    setNbThreads(1);
    //#ifdef _WIN32
    //SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    //#endif
    // create a two-DOF robot manipulator, 0 represents the base frame
    int num = 8;
    shared_ptr<Graph> graph(createGraph(num), deleteGraph);
    // 0->1->2->3->4->5->6
    graph->add_edge(0,1); 
    graph->add_edge(1,2); 
    graph->add_edge(2,3);
    graph->add_edge(3,4);
    graph->add_edge(4,5);
    graph->add_edge(5,6);
    graph->add_edge(6,7);
    
    graph->print();

    double pi = 3.141592653589793238462643383279502884;

    // create the dynamics
    shared_ptr<RobotDynamics> robot_dyn_1(createRobotDynamics(graph.get()),deleteRobotDynamics);
    shared_ptr<RobotDynamics> robot_dyn_2(createRobotDynamics(graph.get()),deleteRobotDynamics);

    // alpha theta a d
	MDH dh0(0, 0, 0, 0, false);
    MDH dh1(0,0,0,0.1273,false);
    MDH dh2(-pi/2,0,0,0,false);
    MDH dh3(0,0,0.612,0,false);
    MDH dh4(0,0,0.5723,0.1639,false);
    MDH dh5(-pi/2,0,0,0.1157,false);
    MDH dh6( pi/2,0,0,0.0922,false);

    vector<Matrix4d> T;
    T.resize(8);
    mdh_to_transformation(dh0, JointType::REVOLUTE, 0, T[1].data(), 16);
    mdh_to_transformation(dh1, JointType::REVOLUTE, 0, T[2].data(), 16);
    mdh_to_transformation(dh2, JointType::REVOLUTE, 0, T[3].data(), 16);
    mdh_to_transformation(dh3, JointType::REVOLUTE, 0, T[4].data(), 16);
    mdh_to_transformation(dh4, JointType::REVOLUTE, 0, T[5].data(), 16);
    mdh_to_transformation(dh5, JointType::REVOLUTE, 0, T[6].data(), 16);
    mdh_to_transformation(dh6, JointType::REVOLUTE, 0, T[7].data(), 16);

    T[2](1,3) += 0.2;
    T[2](2,3) += 0.4;
    robot_dyn_1->set_fixed_transformation(1, T[1].data(), 16);
    robot_dyn_1->set_fixed_transformation(2, T[2].data(), 16);
    robot_dyn_1->set_fixed_transformation(3, T[3].data(), 16);
    robot_dyn_1->set_fixed_transformation(4, T[4].data(), 16);
    robot_dyn_1->set_fixed_transformation(5, T[5].data(), 16);
    robot_dyn_1->set_fixed_transformation(6, T[6].data(), 16);
    robot_dyn_1->set_fixed_transformation(7, T[7].data(), 16);

    robot_dyn_2->set_fixed_transformation(1, T[1].data(), 16);
    robot_dyn_2->set_fixed_transformation(2, T[2].data(), 16);
    robot_dyn_2->set_fixed_transformation(3, T[3].data(), 16);
    robot_dyn_2->set_fixed_transformation(4, T[4].data(), 16);
    robot_dyn_2->set_fixed_transformation(5, T[5].data(), 16);
    robot_dyn_2->set_fixed_transformation(6, T[6].data(), 16);
    robot_dyn_2->set_fixed_transformation(7, T[7].data(), 16);

    //
    robot_dyn_1->set_joint_type(1,JointType::FLOATING,0);
    robot_dyn_1->set_joint_type(2,JointType::REVOLUTE,0);
    robot_dyn_1->set_joint_type(3,JointType::REVOLUTE,0);
    robot_dyn_1->set_joint_type(4,JointType::REVOLUTE,0);
    robot_dyn_1->set_joint_type(5,JointType::REVOLUTE,0);
    robot_dyn_1->set_joint_type(6,JointType::REVOLUTE,0);
    robot_dyn_1->set_joint_type(7, JointType::REVOLUTE,0);

    robot_dyn_2->set_joint_type(1,JointType::FLOATING,0);
    robot_dyn_2->set_joint_type(2,JointType::REVOLUTE,0);
    robot_dyn_2->set_joint_type(3,JointType::REVOLUTE,0);
    robot_dyn_2->set_joint_type(4,JointType::REVOLUTE,0);
    robot_dyn_2->set_joint_type(5,JointType::REVOLUTE,0);
    robot_dyn_2->set_joint_type(6,JointType::REVOLUTE,0);
    robot_dyn_2->set_joint_type(7, JointType::REVOLUTE,0);

    std::cout << "The joint types have been set." << std::endl;
    // dynamic parameters of robot_dyn_1
    DynamicParam link0_param_1(29.053, 0.0, 0.0, 29.053, 0.0, 200.43, 777.8, -0.08, 0.33, 0.037);
    DynamicParam link1_param_1(0.029053, 0.0, 0.0, 0.029053, 0.0, 3.0243, 7.778, 0.00008, 0.00244, -0.037);   // shoulder_link
    DynamicParam link2_param_1(0.023247, 0.0, 0.0, 1.157850, 0.0, 5.157850, 12.930, 0.00001, 0.15061, 0.38757);  // upper_arm_link
    DynamicParam link3_param_1(0.012586, 0.0, 0.0, 0.408420, 0.0, 2.408420, 3.870, -0.00012, 0.06112, 0.1984);   // forearm_link
    DynamicParam link4_param_1(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.25625, 1.960, -0.00021, -0.00112, 0.02269);  // wrist_1_link
    DynamicParam link5_param_1(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.25625, 1.960, -0.00021, 0.00112, 0.002269); // wrist_2_link
    DynamicParam link6_param_1(0.000405, 0.0, 0.0, 0.000405, 0.0, 0.1577, 0.202, 0.0, -0.001156, -0.00149);  // wrist_3_link

    robot_dyn_1->set_dyn_param(1,link0_param_1);
    robot_dyn_1->set_dyn_param(2,link1_param_1);
    robot_dyn_1->set_dyn_param(3,link2_param_1);
    robot_dyn_1->set_dyn_param(4,link3_param_1);
    robot_dyn_1->set_dyn_param(5,link4_param_1);
    robot_dyn_1->set_dyn_param(6,link5_param_1);
	robot_dyn_1->set_dyn_param(7,link6_param_1);

    robot_dyn_1->build_model();

    // dynamic parameters of robot_dyn_2
    DynamicParam link0_param_2(50.3, 0.0, 0.0, 48.03, 0.0, 240.43, 877.8, 0.08, 0.244, -0.37);
    DynamicParam link1_param_2(0.029053, 0.0, 0.0, 0.029053, 0.0, 3.0243, 7.778, 0.00008, 0.00244, -0.037);   // shoulder_link
    DynamicParam link2_param_2(0.023247, 0.0, 0.0, 1.157850, 0.0, 5.157850, 12.930, 0.00001, 0.15061, 0.38757);  // upper_arm_link
    DynamicParam link3_param_2(0.012586, 0.0, 0.0, 0.408420, 0.0, 2.408420, 3.870, -0.00012, 0.06112, 0.1984);   // forearm_link
    DynamicParam link4_param_2(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.25625, 1.960, -0.00021, -0.00112, 0.02269);  // wrist_1_link
    DynamicParam link5_param_2(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.25625, 1.960, -0.00021, 0.00112, 0.002269); // wrist_2_link
    DynamicParam link6_param_2(0.000405, 0.0, 0.0, 0.000405, 0.0, 0.1577, 0.202, 0.0, -0.001156, -0.00149);  // wrist_3_link

    robot_dyn_2->set_dyn_param(1,link0_param_2);
    robot_dyn_2->set_dyn_param(2,link1_param_2);
    robot_dyn_2->set_dyn_param(3,link2_param_2);
    robot_dyn_2->set_dyn_param(4,link3_param_2);
    robot_dyn_2->set_dyn_param(5,link4_param_2);
    robot_dyn_2->set_dyn_param(6,link5_param_2);
	robot_dyn_2->set_dyn_param(7,link6_param_2);

    robot_dyn_2->build_model();

    // create a free-floating object
    shared_ptr<Graph> graph_obj(createGraph(2), deleteGraph);
    // 0->1->2->3->4->5->6
    graph_obj->add_edge(0,1); 
    graph_obj->print();
    
    DynamicParam obj_param(0.34, 0.0, 0.0, 0.53, 0.0, 1.43, 8.8, 0.08, 0.0144, 0.027);
    shared_ptr<RobotDynamics> robot_dyn_obj(createRobotDynamics(graph_obj.get()),deleteRobotDynamics);
    Matrix4d T_obj_offset = Matrix4d::Identity();

    robot_dyn_obj->set_fixed_transformation(1, T_obj_offset.data(), 16);
    robot_dyn_obj->set_dyn_param(1, obj_param);
    robot_dyn_obj->set_joint_type(1,JointType::FLOATING, 0);
    robot_dyn_obj->build_model();

    
    RDL_UTILS::RobotDataAllocator data_allocator_1(13,12), data_allacator_2(13,12), data_allocator_obj(7,6);
    RobotData robot_data_1 = data_allocator_1.get_robot_data();
    RobotData robot_data_2 = data_allacator_2.get_robot_data();
    RobotData robot_data_obj = data_allocator_obj.get_robot_data();

    Vector3d axis_1(0, 0, 1);
    axis_1.normalize();
    AngleAxisd aa_1(0, axis_1);
    Quaterniond quat1(aa_1);

    //VectorXd q_1(13), qd_1(12), qdd_1(12);
    Map<VectorXd> q_1(robot_data_1.q, 13), qd_1(robot_data_1.qd, 12), qdd_1(robot_data_1.qdd, 12);
    Map<MatrixXd> M_1(robot_data_1.M, 12, 12), C_1(robot_data_1.C, 12, 12);
    Map<VectorXd> tau_1(robot_data_1.tau, 12), g_1(robot_data_1.g, 12), h_1(robot_data_1.h, 12);
	q_1 <<  quat1.x(), quat1.y(), quat1.z(), quat1.w(), 0, -0.8, 0.0, pi/2, -pi / 4, pi / 2, -pi, -pi / 2, 0;
    qd_1 << 0.003, 0.001, 0.002, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0;
    qdd_1.setZero();
    
    Vector3d axis_2(0, 0, 1);
    axis_2.normalize();
    AngleAxisd aa_2(pi, axis_2);
    Quaterniond quat2(aa_2);
    print("quat2",quat2.vec().transpose());

    Map<VectorXd> q_2(robot_data_2.q, 13), qd_2(robot_data_2.qd, 12), qdd_2(robot_data_2.qdd, 12);
    Map<MatrixXd> M_2(robot_data_2.M, 12, 12), C_2(robot_data_2.C, 12, 12);
    Map<VectorXd> tau_2(robot_data_2.tau, 12), g_2(robot_data_2.g, 12), h_2(robot_data_2.h, 12);
	q_2 <<  quat2.x(), quat2.y(), quat2.z(), quat2.w(), 0, 0.8, 0.0, 3*pi/2, pi / 4, -pi / 2, -pi, pi / 2, 0;
    qd_2 << -0.003, -0.001, 0.002, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0;
    qdd_2.setZero();

    Map<VectorXd> q_obj(robot_data_obj.q, 7), qd_obj(robot_data_obj.qd, 6), qdd_obj(robot_data_obj.qdd, 6);
    Map<MatrixXd> M_obj(robot_data_obj.M, 6, 6), C_obj(robot_data_obj.C, 6, 6);
    Map<VectorXd> tau_obj(robot_data_obj.tau, 6), g_obj(robot_data_obj.g, 6), h_obj(robot_data_obj.h, 6);
    q_obj << 0, 0, 0, 1, 0.0, 0, 0.4;
    qd_obj << 0.003, 0.001, 0.002, 0, 0, 0;
    qdd_obj.setZero();

    
    robot_dyn_1->forward_kinematics(robot_data_1.q, robot_data_1.dim_pos);
    robot_dyn_1->forward_dynamics_crba(robot_data_1.M, robot_data_1.dof_dyn);

    robot_dyn_2->forward_kinematics(robot_data_2.q, robot_data_2.dim_pos);
    robot_dyn_2->forward_dynamics_crba(robot_data_2.M, robot_data_2.dof_dyn);

    robot_dyn_obj->forward_kinematics(robot_data_obj.q, robot_data_obj.dim_pos);
    robot_dyn_obj->forward_dynamics_crba(robot_data_obj.M, robot_data_obj.dof_dyn);

    print("M1_bb", M_1.block<6,6>(0,0));
    print("M1_mm", M_1.block<6,6>(6,6));
    print("M1_bm", M_1.block<6,6>(0,6));
    print("M2_bb", M_2.block<6,6>(0,0));
    print("M2_mm", M_2.block<6,6>(6,6));
    print("M2_bm", M_2.block<6,6>(0,6));
    print("Mobj", M_obj);

    // construct a constraint
    robot_dyn_1->forward_kinematics(robot_data_1.q, robot_data_1.dim_pos);
    robot_dyn_2->forward_kinematics(robot_data_2.q, robot_data_2.dim_pos);
    robot_dyn_obj->forward_kinematics(robot_data_obj.q, robot_data_obj.dim_pos);
    Matrix4d T_end_1, T_end_2, T_base_1, T_base_2, T_obj;
    size_t end_frame_id = 7;
    robot_dyn_1->get_pose(end_frame_id, T_end_1.data(), 16);
    robot_dyn_2->get_pose(end_frame_id, T_end_2.data(), 16);
    robot_dyn_obj->get_pose(1, T_obj.data(), 16);
    robot_dyn_1->get_pose(1, T_base_1.data(), 16);
    robot_dyn_2->get_pose(1, T_base_2.data(), 16);

    print("T_end_1",T_end_1);
    print("T_end_2",T_end_2);
    print("T_obj",T_obj);
    Vector3d del_r = T_end_1.block<3,1>(0,3) - T_end_2.block<3,1>(0,3);
    Matrix3d del_R = T_end_2.block<3,3>(0,0).transpose()*T_end_1.block<3,3>(0,0);
    print("del_r",del_r.transpose());

    Matrix3d R_constraint_1 = T_end_1.block<3, 3>(0, 0);
    Vector3d r_constraint_1 = T_end_1.block<3, 1>(0, 3) - T_obj.block<3, 1>(0, 3);
    Matrix3d R_constraint_2 = T_end_2.block<3, 3>(0, 0);
    Vector3d r_constraint_2 = T_end_2.block<3, 1>(0, 3) - T_obj.block<3, 1>(0, 3);
    print("r_constraint_1", r_constraint_1.transpose());
    print("r_constraint_2", r_constraint_2.transpose());
    print("R_constraint_1", R_constraint_1);
    print("R_constraint_2", R_constraint_2);

    //print("del_r_base",T_base_1.block<3,1>(0,3) - T_base_2.block<3,1>(0,3));

    LoopConstraint constraint_1;
    constraint_1.model_A = 0;
    constraint_1.frame_A = end_frame_id;
    constraint_1.model_B = 2;
    constraint_1.frame_B = 1;
    matrix3d_to_array(Matrix3d::Identity(), constraint_1.R_A);
    vector3d_to_array(Vector3d::Zero(), constraint_1.r_A);
    matrix3d_to_array(R_constraint_1, constraint_1.R_B);
    vector3d_to_array(r_constraint_1, constraint_1.r_B);
    constraint_1.dof_A = 12;
    constraint_1.dof_B = 6;
    constraint_1.mask.rx = true;
    constraint_1.mask.ry = true;
    constraint_1.mask.rz = true;
    constraint_1.mask.x = true;
    constraint_1.mask.y = true;
    constraint_1.mask.z = true;
    //constraint_1.type = ConstraintType::CLOSED_LOOP;

    LoopConstraint constraint_2;
    constraint_2.model_A = 1;
    constraint_2.frame_A = end_frame_id;
    constraint_2.model_B = 2;
    constraint_2.frame_B = 1;
    matrix3d_to_array(Matrix3d::Identity(), constraint_2.R_A);
    vector3d_to_array(Vector3d::Zero(), constraint_2.r_A);
    matrix3d_to_array(R_constraint_2, constraint_2.R_B);
    vector3d_to_array(r_constraint_2, constraint_2.r_B);

    constraint_2.dof_A = 12;
    constraint_2.dof_B = 6;
    constraint_2.mask.rx = true;
    constraint_2.mask.ry = true;
    constraint_2.mask.rz = true;
    constraint_2.mask.x = true;
    constraint_2.mask.y = true;
    constraint_2.mask.z = true;
    //constraint_2.type = ConstraintType::CLOSED_LOOP;

    Matrix<double, 6, 1> acc;
    acc.setZero();
    robot_dyn_1->set_spatial_gravitational_acc(acc.data(), 6);
    robot_dyn_2->set_spatial_gravitational_acc(acc.data(), 6);
    robot_dyn_obj->set_spatial_gravitational_acc(acc.data(), 6);   

    vector<LoopConstraint> constraints = {constraint_1,constraint_2};
    vector<RobotDynamics*> robots = {robot_dyn_1.get(), robot_dyn_2.get(), robot_dyn_obj.get()};
    vector<RobotData*> datas = {&robot_data_1, &robot_data_2, &robot_data_obj};

    vector<size_t> dofs = {12, 12, 6};
    shared_ptr<ConstrainedDynamicsSolver> solver(createConstrainedDynamicsSolver(3, dofs.data(), 2, 12),  deleteConstrainedDynamicsSolver);
    solver->set_constraints(constraints.data(), constraints.size());
    solver->set_parameters(40.0,400.0,0.0,0.0,0.0);
    solver->solve(robots.data(), datas.data(), 3);

    print("qdd_1",qdd_1.transpose());
    print("qdd_2",qdd_2.transpose());
    print("qdd_obj",qdd_obj.transpose());

    Matrix<double, 6, 12> J_1, J_2;
    robot_dyn_1->forward_kinematics(robot_data_1.q, 13);
    robot_dyn_2->forward_kinematics(robot_data_2.q, 13);
    robot_dyn_1->cal_jacobian(end_frame_id, J_1.data(), 6, 12, ReferenceFrame::BODY);
    robot_dyn_2->cal_jacobian(end_frame_id, J_2.data(), 6, 12, ReferenceFrame::BODY);
    print("J_1",J_1);
    print("J_2",J_2);

    double dt = 0.0005;
    size_t N = 100/dt;
    RDL_UTILS::RobotDynamicsIntegrator integrator(dt,RDL_UTILS::IntegrationMethod::SEMI_IMPLICIT_EULER);
    RDL_UTILS::DataRecorder data_recorder({"time", "quat-1", "w-1", "quat-2", "w-2", "quat-3", "w-3", "tau-1", "tau-2","q-1","q-2","pos-1","pos-2","pos-obj", "c-err", "c-err-d", "c-err-dd", "lambda-1","lambda-2","r-end-1","r-end-2","del-r-obj"});

    Matrix<double,6,12> J1, J2;
    Matrix<double,6,1> gamma1, gamma2;

    Matrix<double,6,1> q_1_0, q_2_0;
    q_1_0 = q_1.segment<6>(7);
    q_2_0 = q_2.segment<6>(7);

    qd_1.setZero();
    qd_2.setZero();
    qd_obj.setZero();
    tau_1.setZero();
    tau_2.setZero();

    Eigen::Matrix<double,6,6> Kd, Kp;
    Kd.setIdentity();
    Kp.setIdentity();
    Kp(0, 0) = 600;
    Kp(1, 1) = 600;
    Kp(2, 2) = 800;
    Kp(3, 3) = 800;
    Kp(4, 4) = 600;
    Kp(5, 5) = 600;
    Kp = Kp*4;
    Kd = Kp/10;

    Eigen::VectorXd q_des_1, qd_des_1, qdd_des_1,q_des_2, qd_des_2, qdd_des_2;
    q_des_1.resize(6);
    qd_des_1.resize(6); qd_des_1.setZero();
    qdd_des_1.resize(6); qdd_des_1.setZero();
    q_des_2.resize(6);
    qd_des_2.resize(6); qd_des_2.setZero();
    qdd_des_2.resize(6); qdd_des_2.setZero();


    Eigen::Matrix<double,9,1> c_err, c_err_d, c_err_dd;
    Eigen::Matrix<double,6,1> h_bar_1, h_bar_2;
    Eigen::Matrix<double,6,6> M_bar_1, M_bar_2, J_bar_1, J_bar_2;
    Eigen::Matrix<double,12,1> lambda;

    q_des_1 = q_1_0;
    q_des_2 = q_2_0;
    Eigen::Vector3d del_r_12 = T_end_1.block<3,1>(0,3) - T_end_2.block<3,1>(0,3);

    Eigen::Vector3d x_des_obj, xd_des_obj, xdd_des_obj;
    x_des_obj = T_obj.block<3,1>(0,3);
    x_des_obj(0) += 0.1;
    x_des_obj(1) -= 0.1;
    x_des_obj(2) += 0.05;
    xd_des_obj.setZero();
    xdd_des_obj.setZero();
    Eigen::Matrix<double,3,12> J_bar;
    Eigen::Matrix<double,6,12>J_bar_obj;
    Eigen::AngleAxisd aa(-5/57.3,Eigen::Vector3d::UnitX());
    Eigen::Matrix3d R_des_obj = aa.toRotationMatrix();
    
    for (size_t i = 0; i < N; i++)
    {
        robot_data_1.qdd = qdd_1.data();
        robot_data_2.qdd = qdd_2.data();
        robot_data_obj.qdd = qdd_obj.data();
        robot_dyn_1->forward_kinematics(robot_data_1.q, 13);
        robot_dyn_1->forward_dynamics_crba(M_1.data(), robot_data_1.dof_dyn);
        robot_dyn_1->cal_nonlinear_effect(robot_data_1.qd, h_1.data(), h_1.size());
        robot_dyn_1->cal_jacobian(end_frame_id,J_1.data(), 6, 12, ReferenceFrame::BODY_WORLD);
        M_bar_1 = M_1.block<6,6>(6,6) - M_1.block<6,6>(6,0)*M_1.block<6,6>(0,0).inverse()*M_1.block<6,6>(0,6);
        h_bar_1 = h_1.segment<6>(6) - M_1.block<6,6>(6,0)*M_1.block<6,6>(0,0).inverse()*h_1.segment<6>(0);
        J_bar_1 = J_1.block<6,6>(0,6) - J_1.block<6,6>(0,0)*M_1.block<6,6>(0,0).inverse()*M_1.block<6,6>(0,6);
        robot_dyn_1->get_pose(end_frame_id, T_end_1.data(), 16);
        Eigen::Matrix3d R_end_1 = T_end_1.block<3,3>(0,0);
        Eigen::VectorXd v_1 = J_1*qd_1;
        Eigen::Vector3d r_end_1 = T_end_1.block<3,1>(0,3);
        Eigen::Vector3d v_end_1 = v_1.tail<3>();

        //print("M_bar_1", M_bar_1);
        //print("h_bar_1", h_bar_1.transpose());
        robot_dyn_2->forward_kinematics(robot_data_2.q, 13);
        robot_dyn_2->forward_dynamics_crba(M_2.data(), robot_data_2.dof_dyn);
        robot_dyn_2->cal_nonlinear_effect(robot_data_2.qd, h_2.data(), h_2.size());
        robot_dyn_2->cal_jacobian(end_frame_id,J_2.data(), 6, 12, ReferenceFrame::BODY_WORLD);
        M_bar_2 = M_2.block<6,6>(6,6) - M_2.block<6,6>(6,0)*M_2.block<6,6>(0,0).inverse()*M_2.block<6,6>(0,6);
        h_bar_2 = h_2.segment<6>(6) - M_2.block<6,6>(6,0)*M_2.block<6,6>(0,0).inverse()*h_2.segment<6>(0);
        J_bar_2 = J_2.block<6,6>(0,6) - J_2.block<6,6>(0,0)*M_2.block<6,6>(0,0).inverse()*M_2.block<6,6>(0,6);
        robot_dyn_2->get_pose(end_frame_id, T_end_2.data(), 16);
        Eigen::VectorXd v_2 = J_2*qd_2;
        Eigen::Matrix3d R_end_2 = T_end_2.block<3,3>(0,0);
        Eigen::Vector3d r_end_2 = T_end_2.block<3,1>(0,3);
        Eigen::Vector3d v_end_2 = v_2.tail<3>();

        // update position of obj
        robot_dyn_obj->forward_kinematics(robot_data_obj.q, 7);
        robot_dyn_obj->get_pose(1, T_obj.data(), 16);
        Eigen::Matrix3d R_obj = T_obj.block<3,3>(0,0);
        Eigen::Vector3d r_end_obj = T_obj.block<3,1>(0,3);

        J_bar_obj.block<3,6>(0,0) = R_obj.transpose()*R_end_1*J_bar_1.block<3,6>(0,0);
        Eigen::Matrix3d r_cross;
        r_cross=cross_3d(-R_obj*r_constraint_1);
        J_bar_obj.block<3,6>(3,0) = J_bar_1.block<3,6>(3,0) - r_cross*R_end_1*J_bar_1.block<3,6>(0,0);

        J_bar_obj.block<3,6>(0,6) = R_obj.transpose()*R_end_2*J_bar_2.block<3,6>(0,0);
        r_cross=cross_3d(-R_obj*r_constraint_2);
        J_bar_obj.block<3,6>(3,6) = J_bar_2.block<3,6>(3,0) - r_cross*R_end_2*J_bar_2.block<3,6>(0,0);

        J_bar_obj.block<6,6>(0,0) = 0.5*J_bar_obj.block<6,6>(0,0);
        J_bar_obj.block<6,6>(0,6) = 0.5*J_bar_obj.block<6,6>(0,6);

        Eigen::Quaterniond del_quat_obj(R_des_obj.transpose()*R_obj);
        Eigen::AngleAxisd del_aa_obj(del_quat_obj);
        Eigen::Matrix<double,6,1> u_obj;
        Eigen::Vector3d del_r_obj = r_end_obj - x_des_obj;
        Eigen::Vector3d del_w_obj = del_quat_obj.vec();//del_aa_obj.axis() * del_aa_obj.angle();
        u_obj.head<3>() = -0.4*del_w_obj;
        u_obj.tail<3>() = -0.4*del_r_obj;

        Eigen::Matrix<double,6,6> JJT_obj = J_bar_obj*J_bar_obj.transpose();
        Eigen::Matrix<double,12,1> qd_des_all = J_bar_obj.transpose()*JJT_obj.inverse()*u_obj;
        qd_des_1 = qd_des_all.segment<6>(0);
        qdd_des_1.setZero();
        q_des_1 += dt*qd_des_1;
        qd_des_2 = qd_des_all.segment<6>(6);
        qdd_des_2.setZero();
        q_des_2 += dt*qd_des_2;
        /*
        Eigen::Quaterniond del_quat_1(R_des_1.transpose()*T_end_1.block<3,3>(0,0));
        Eigen::AngleAxisd del_aa(del_quat_1);
        Eigen::Matrix<double,6,1> u_1;
		Eigen::Vector3d del_r_1 = r_end_1 - x_des_1;
		Eigen::Vector3d del_w_1 = del_aa.axis() * del_aa.angle();
		//if (del_r_1.norm() < 2e-3) del_r_1.setZero();
		//if (del_w_1.norm() < 2e-3) del_w_1.setZero();
        u_1.head<3>() = -1.5*del_w_1;
        u_1.tail<3>() = -1.5*del_r_1;// - 0.1*(r_end_1-r_end_2 - del_r_12);
        //Eigen::Matrix3d JJT_1 = J_bar_1.block<3,6>(3,0)*J_bar_1.block<3,6>(3,0).transpose();
        qd_des_1 = J_bar_1.inverse()*u_1 ;
        qdd_des_1.setZero();
        q_des_1 += dt*qd_des_1;

        Eigen::Quaterniond del_quat_2(R_des_2.transpose()*T_end_2.block<3,3>(0,0));
        Eigen::Matrix<double,6,1> u_2;
		Eigen::Vector3d del_r_2 = r_end_2 - x_des_2;
		//if (del_r_2.norm() < 1e-4) del_r_2.setZero();
        u_2.head<3>() = -0.0*del_quat_2.vec();
        u_2.tail<3>() = -0.0*del_r_2 - 0.15* (v_end_2- v_end_1) - 2.5*(r_end_2-r_end_1 + T_obj.block<3,3>(0,0)*del_r_12);
        Eigen::Matrix3d JJT_2 = J_bar_2.block<3,6>(3,0)*J_bar_2.block<3,6>(3,0).transpose();
        qd_des_2 = J_bar_2.block<3, 6>(3, 0).transpose()* JJT_2.inverse()*u_2.tail<3>(); //J_bar_2.inverse() * u_2; 
        qdd_des_2.setZero();
        q_des_2 += dt*qd_des_2;
        */
        
        //print("M_bar_2", M_bar_2);
        //print("h_bar_2", h_bar_2.transpose());
        tau_1.segment<6>(6) = M_bar_1*(qdd_des_1  -Kp*(q_1.segment<6>(7) - q_des_1) - Kd*(qd_1.segment<6>(6)-qd_des_1)) + h_bar_1;
        //tau_1.setZero();
		//tau_1 = J_bar_1.inverse() * u_1 + h_bar_1;
        tau_2.segment<6>(6) =  M_bar_2*(qdd_des_2 -Kp*(q_2.segment<6>(7) - q_des_2) - Kd*(qd_2.segment<6>(6)) - qd_des_2) + h_bar_2;
        tau_obj.setZero();
        //print("tau_1", tau_1.transpose());
        //print("tau_2", tau_2.transpose());

        // test taskspace control
        /*robot_dyn_1->get_pose(end_frame_id, T_end_1.data(), 16);
        Eigen::Vector3d r_end_1 = T_end_1.block<3,1>(0,3);
        Eigen::Matrix<double,6,6> M_tmp = J_bar_1.transpose()*(J_bar_1*M_1.block<6,6>(6,6).inverse()*J_bar_1.transpose()).inverse();
        tau_1.segment<6>(6) = -M_tmp.block<3,6>(3,0).transpose()*0.3*(r_end_1 - x_des_1);

        robot_dyn_2->get_pose(end_frame_id, T_end_2.data(), 16);
        Eigen::Vector3d r_end_2 = T_end_2.block<3,1>(0,3);
        Eigen::Matrix<double,6,6> M_tmp_2 = J_bar_2.transpose()*(J_bar_2*M_2.block<6,6>(6,6).inverse()*J_bar_2.transpose()).inverse();
        tau_2.segment<6>(6) = -M_tmp_2.block<3,6>(3,0).transpose()*0.3*(r_end_2 - x_des_2);
        */
        solver->get_constraint_err(c_err.data(), c_err_d.data(), c_err_dd.data(), 6);
        solver->get_lambda(lambda.data(), 12);
        
        if (i%50 == 0)
            data_recorder.record_frame(i*dt, q_1.segment<4>(0), qd_1.segment<3>(0), q_2.segment<4>(0), qd_2.segment<3>(0), q_obj.segment<4>(0), qd_obj.segment<3>(0),tau_1.segment<6>(6),tau_2.segment<6>(6),q_1.segment<6>(7),q_2.segment<6>(7),q_1.segment<3>(4),q_2.segment<3>(4),r_end_obj, c_err, c_err_d, c_err_dd, lambda.head<6>(), lambda.tail<6>(), r_end_1, r_end_2,del_r_obj);
        integrator.integrate(robots, datas, solver.get());
    }
    data_recorder.save("dual_floating_robots.txt");
   
    return 0;
}