#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32

#include <graph.h>
#include <robot_dynamics.h>
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
    Eigen::setNbThreads(1);
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
    shared_ptr<RobotDynamics> robot_dyn(createRobotDynamics(graph.get()),deleteRobotDynamics);
    // d theta a alpha
    // alpha theta a d
    MDH dh_1(0,0,0,0,false);
	MDH dh0(0, 0, 0, 0, false);
    MDH dh1(0,0,0,0.1273,false);
    MDH dh2(-pi/2,0,0,0,false);
    MDH dh3(0,0,0.612,0,false);
    MDH dh4(0,0,0.5723,0.1639,false);
    MDH dh5(-pi/2,0,0,0.1157,false);
    MDH dh6( pi/2,0,0,0.0922,false);

    vector<Eigen::Matrix4d> T;
    T.resize(8);
    mdh_to_transformation(dh_1,JointType::REVOLUTE, 0, T[0].data(), 16);
    mdh_to_transformation(dh0, JointType::REVOLUTE, 0, T[1].data(), 16);
    mdh_to_transformation(dh1, JointType::REVOLUTE, 0, T[2].data(), 16);
    mdh_to_transformation(dh2, JointType::REVOLUTE, 0, T[3].data(), 16);
    mdh_to_transformation(dh3, JointType::REVOLUTE, 0, T[4].data(), 16);
    mdh_to_transformation(dh4, JointType::REVOLUTE, 0, T[5].data(), 16);
    mdh_to_transformation(dh5, JointType::REVOLUTE, 0, T[6].data(), 16);
    mdh_to_transformation(dh6, JointType::REVOLUTE, 0, T[7].data(), 16);

    //T[2].block<3, 1>(0, 3) << 0.1, -0.2, 0.4;
    //robot_dyn->set_fixed_transformation(0, T[0].data(), 16);
    robot_dyn->set_fixed_transformation(1, T[1].data(), 16);
    robot_dyn->set_fixed_transformation(2, T[2].data(), 16);
    robot_dyn->set_fixed_transformation(3, T[3].data(), 16);
    robot_dyn->set_fixed_transformation(4, T[4].data(), 16);
    robot_dyn->set_fixed_transformation(5, T[5].data(), 16);
    robot_dyn->set_fixed_transformation(6, T[6].data(), 16);
    robot_dyn->set_fixed_transformation(7, T[7].data(), 16);
    //
    std::cout << "The MDH parameters have been set." << std::endl;
    //robot_dyn->set_joint_type(0,JointType::FIXED);
    robot_dyn->set_joint_type(1,JointType::FLOATING, 0);
    robot_dyn->set_joint_type(2,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(3,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(4,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(5,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(6,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(7, JointType::REVOLUTE, 0);
    std::cout << "The joint types have been set." << std::endl;

    DynamicParam link0_param(29.053, 0.0, 0.0, 29.053, 0.0, 200.43, 777.8, 0.00008, 0.00244, -0.037);
    /*DynamicParam link1_param(1, 0.0, 0.0, 1, 0.0, 1, 7.778, 0.00008, 0.00244, -0.037);   // shoulder_link
    DynamicParam link2_param(0.5, 0.0, 0.0, 0.5, 0.0, 0.5, 12.930, 0.00001,  0.15061,  0.38757);  // upper_arm_link
    DynamicParam link3_param(0.5, 0.0, 0.0, 0.5, 0.0, 0.5,  3.870, -0.00012, 0.06112,  0.1984);   // forearm_link
    DynamicParam link4_param(0.1, 0.0, 0.0, 0.1, 0.0, 0.1,  1.960, -0.00021, -0.00112, 0.02269);  // wrist_1_link
    DynamicParam link5_param(0.1, 0.0, 0.0, 0.1, 0.0, 0.1,  1.960, -0.00021, 0.00112,  0.002269); // wrist_2_link
    DynamicParam link6_param(0.1, 0.0, 0.0, 0.1, 0.0, 0.1,  0.202,  0.0,     -0.001156,-0.00149);  // wrist_3_link
    */
    DynamicParam link1_param(0.029053, 0.0, 0.0, 0.029053, 0.0, 3.0243, 7.778, 0.00008, 0.00244, -0.037);   // shoulder_link
    DynamicParam link2_param(0.023247, 0.0, 0.0, 1.157850, 0.0, 5.157850, 12.930, 0.00001, 0.15061, 0.38757);  // upper_arm_link
    DynamicParam link3_param(00.012586, 0.0, 0.0, 0.408420, 0.0, 2.408420, 3.870, -0.00012, 0.06112, 0.1984);   // forearm_link
    DynamicParam link4_param(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.25625, 1.960, -0.00021, -0.00112, 0.02269);  // wrist_1_link
    DynamicParam link5_param(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.25625, 1.960, -0.00021, 0.00112, 0.002269); // wrist_2_link
    DynamicParam link6_param(0.000405, 0.0, 0.0, 0.000405, 0.0, 0.1577, 0.202, 0.0, -0.001156, -0.00149);  // wrist_3_link

    //robot_dyn->set_dyn_param(0,link0_param);
    robot_dyn->set_dyn_param(1, link0_param);
    robot_dyn->set_dyn_param(2,link1_param);
    robot_dyn->set_dyn_param(3,link2_param);
    robot_dyn->set_dyn_param(4,link3_param);
    robot_dyn->set_dyn_param(5,link4_param);
    robot_dyn->set_dyn_param(6,link5_param);
	robot_dyn->set_dyn_param(7,link6_param);

    robot_dyn->build_model();
     // set the joint positions, velocities, and accelerations  
    //double q[6] = {0, -pi/4, 0, -pi/2, 0, pi/3};
    //double qd[6] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    //double qdd[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    Eigen::Vector3d axis(1, -1, 2);
    axis.normalize();
    Eigen::AngleAxisd aa1(pi / 4*0, axis);
    Eigen::Quaterniond q1(aa1);

    Eigen::VectorXd q(13), qd(12), qdd(12);
	//q <<  q1.x(), q1.y(), q1.z(), q1.w(), 0, 0, 0, 0, -pi / 4, pi / 2, -pi / 2, -pi / 3, pi / 3;
	q << 0, 0, 0, 1, 0, 0, 0, 0, -pi / 4, pi / 2, -pi / 2, -pi / 3, pi / 3;
	//qd << 0.21, -0.33, 0.5022, 0.0, 0.0, 0.0, 0.0, -0.5, -0.2, 0.7, -1.3, 0.2;
    //qd << 0,0,0,0,0,0, 0.27, -0.5, -0.2, 0.7, -1.3, 0.2;
	//qdd << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    qd << 0.01, -0.03, 0.05, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0;
    qdd.setZero();
     

    //robot_dyn->set_joint_pos(q.data(), 6);
    //robot_dyn->set_joint_vel(qd.data(), 6);
    //robot_dyn->set_joint_acc(qdd.data(), 6);
    //printf("1\n");
    Eigen::VectorXd a0(6); 
    a0.setZero();	
    robot_dyn->forward_kinematics(q.data(),q.size());
	Eigen::Matrix4d T0;
	robot_dyn->get_pose(1, T0.data(), 16);
	//qdd.segment<3>(3) = -T0.block<3, 3>(0, 0).transpose() * a0.segment<3>(3);
    //robot_dyn->set_base_spatial_motion(a0.data(), a0.data(), 6);
    Eigen::MatrixXd M(12,12);// = get_M(robot_dyn);
    Eigen::MatrixXd C(12,12);// = get_C(robot_dyn);
    Eigen::VectorXd g(12);// = get_g(robot_dyn);
	auto start = Clock::now();
	auto end = Clock::now();
    for (size_t i=0; i<20000; i++)
    {
        robot_dyn->forward_kinematics(q.data(),q.size());
        //robot_dyn->forward_iterations();
        //robot_dyn->forward_dynamics(qd.data(),M.data(),C.data(),g.data(),qd.size());
        robot_dyn->forward_dynamics_crba(M.data(),qd.size());
        //printf("3\n");
    }
    end = Clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "cal inertia, coriolis, gravitational time: " << duration.count()/20000 << " ns" << endl;
	robot_dyn->cal_coriolis_matrix(qd.data(), C.data(), qd.size());
	robot_dyn->cal_gravitational_torque(g.data(), g.size());
    Eigen::VectorXd tau = M*qdd +C*qd+ g;


    print("Mqq", M.block<6,6>(0,0));
    print("Mbb", M.block<6,6>(6,6));
    print("Mqb", M.block<6,6>(0,6));
	//print("C", C);
    //print("Cbb", C.block<6,6>(6,6));
	print("g", g.transpose());
    print("tau", tau.transpose());
    
    start = Clock::now();
    size_t N = 20000;
    for (size_t i = 0; i < N; i++)
    {   
        
        robot_dyn->forward_kinematics(q.data(),q.size());
        robot_dyn->forward_iterations(qd.data(),qdd.data(),qd.size());
        robot_dyn->backward_iterations(tau.data(),tau.size());
        //cout << "backward iterations time: " << duration_.count() << " us" << endl;
        //printf("elapsed time: %lld \n", duration_.count());
        //times.push_back(duration_.count());
    }
    end = Clock::now();
    auto duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "average iterations time: " << duration_.count()/N << " ns" << endl;
    print("tau_", tau.transpose());
    cout << "test aba: " << endl;

	Eigen::VectorXd qdd_aba(12);
    Eigen::Matrix<double, 12, 1> zero_tau;
	zero_tau.setZero();
    start = Clock::now();
    for (size_t i = 0; i < N; i++)
	{
        robot_dyn->forward_kinematics(q.data(),q.size());
        robot_dyn->forward_dynamics_aba(qd.data(),zero_tau.data(), qdd_aba.data(), 12, false);
    }
    end = Clock::now();
    duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "aba average iterations time: " << duration_.count() / N << " ns" << endl;
	print("qdd_aba", qdd_aba.transpose());
	print("qdd    ", -1.0*(M.inverse()*(C * qd + g)).transpose());

    robot_dyn->forward_iterations(qd.data(),qdd_aba.data(), qd.size());
	robot_dyn->backward_iterations(tau.data(), tau.size());
	print("tau_aba", tau.transpose());

    // center of mass
    Eigen::Vector3d com, com_vel;
    robot_dyn->cal_center_of_mass(com.data(),com.size());
    robot_dyn->cal_center_of_mass_vel(qd.data(), qd.size(), com_vel.data(), com_vel.size());
    print("com", com.transpose());
    print("com_vel", com_vel.transpose());
    Eigen::Matrix<double, 3, 12> J_com;
    start = Clock::now();
    for (size_t i = 0; i < N; i++)
    {
        robot_dyn->cal_jacobian_com(J_com.data(), J_com.rows(), J_com.cols());
    }
    end = Clock::now();
    duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "J_com average iterations time: " << duration_.count() / N << " ns" << endl;

    print("Jcom", J_com);
    print("com_vel_",(J_com*qd).transpose());

    Eigen::Matrix4d T1, T2, T3, T4, T5, T6;
    robot_dyn->get_pose(1, T1.data(), 16);
    robot_dyn->get_pose(2, T2.data(), 16);
    robot_dyn->get_pose(3, T3.data(), 16);
    robot_dyn->get_pose(4, T4.data(), 16);
    robot_dyn->get_pose(5, T5.data(), 16);
    robot_dyn->get_pose(6, T6.data(), 16);
    
    //print("T1", T1);
    //print("T2", T2);
    //print("T3", T3);
    //print("T4", T4);
    //print("T5", T5);
    //print("T6", T6);

    double dt = 0.001;
   N = 400/dt;
   RDL_UTILS::RobotDynamicsIntegrator integrator(dt);
   //robot_data_eigen.q.setZero();
   //robot_data_eigen.qd.setZero();
   
   double mag = 0.2;
   double omega = M_PI/2.0;
   Eigen::Matrix<double,6,6> Kd, Kp;
   Kd.setIdentity();
   Kp.setIdentity();
   Kp(0, 0) = 400;
   Kp(1, 1) = 400;
   Kp(2, 2) = 400;
   Kp(3, 3) = 400;
   Kp(4, 4) = 400;
   Kp(5, 5) = 400;
   Kd = Kp / 10;

   RDL_UTILS::RobotDataAllocator robot_data_allocator(13,12);
   RDL::RobotData robot_data = robot_data_allocator.get_robot_data();
   for (size_t i = 0; i < 13; i++)
   {
       robot_data.q[i] = q(i);
   }
   for (size_t i = 0; i < 12; i++)
   {
       robot_data.qd[i] = qd(i);
   }
   Eigen::Map<Eigen::VectorXd> q_(robot_data.q,13);
   Eigen::Map<Eigen::VectorXd> qd_(robot_data.qd,12);
   Eigen::Map<Eigen::VectorXd> qdd_(robot_data.qdd,12);
   Eigen::Map<Eigen::VectorXd> tau_cal(robot_data.tau,12);
   Eigen::Map<Eigen::VectorXd> g_(robot_data.g, 12);
   Eigen::Map<Eigen::VectorXd> h_(robot_data.h, 12);
   Eigen::VectorXd q0 = q_;
   Eigen::Vector3d m_ang, m_lin;
   RDL_UTILS::DataRecorder data_recorder({"time", "quat-b", "w-b", "pos-b", "v-b","pos-r", "vel-r", "torque", "traking-error", "m-ang", "m-lin"});

   Eigen::VectorXd v = qd_.segment<6>(0); //Eigen::VectorXd::Zero(6);
   Eigen::VectorXd a = Eigen::VectorXd::Zero(6);
   //robot_dyn->set_base_spatial_motion(v.data(), a.data(), 6);
   robot_dyn->set_spatial_gravitational_acc(a.data(), 6);

   tau_cal.setZero();
   Eigen::Vector3d pos;
   pos.setZero();
   //Eigen::Map<Eigen::Matrix> M(robot_data.M, 12, 12);
   Eigen::Matrix<double,6,6> M_bar;
   Eigen::Matrix<double,6,1> h_bar;

   std::vector<RDL::RobotDynamics*> robot_dynamics_list = {robot_dyn.get()};
   std::vector<RDL::RobotData*> robot_data_list = {&robot_data};
   for (size_t i = 0; i < N; i++)
   {
        Eigen::VectorXd q_des = q0.segment<6>(7) + mag*sin(omega*i*dt)* Eigen::VectorXd::Ones(6);      
        Eigen::VectorXd qd_des = omega*mag*cos(omega*i*dt)* Eigen::VectorXd::Ones(6);
        Eigen::VectorXd qdd_des = -omega*omega*mag*sin(omega*i*dt)* Eigen::VectorXd::Ones(6);
        
        //q_des = q0;
        //qd_des.setZero();
        //q_des(5) = mag * sin(omega * i * dt);
        //qd_des(5) = omega * mag * cos(omega * i * dt);
        //robot_dyn->get_spatial_motion(1, v.data(), a.data(), 6);
        //std::cout << "v & a: " << v.transpose() << ", " << a.transpose() << std::endl;
        robot_dyn->forward_kinematics(q_.data(), q_.size());
        robot_dyn->forward_dynamics_crba(M.data(), robot_data.dof_dyn);
        robot_dyn->cal_nonlinear_effect(robot_data.qd, h_.data(), h_.size());
        M_bar = M.block<6,6>(6,6) - M.block<6,6>(6,0)*M.block<6,6>(0,0).inverse()*M.block<6,6>(0,6);
        h_bar = h_.segment<6>(6) - M.block<6,6>(6,0)*M.block<6,6>(0,0).inverse()*h_.segment<6>(0);
        //robot_dyn->cal_gravitational_torque(g_.data(), 6);
        tau_cal.segment<6>(6) = M_bar*(qdd_des-Kp * (q_.segment<6>(7) - q_des) - Kd*(qd_.segment<6>(6) - qd_des)) + h_bar;

        //tau_cal.segment<6>(6) = -Kp * (q_.segment<6>(7) - q_des) - Kd*(qd_.segment<6>(6) - qd_des);
        if (i==0)
        {
            print("tau",tau.transpose());
            print("h_bar", h_bar.transpose());
            print("Mbb", M.block<6,6>(0,0).transpose());
            print("Mmb", M.block<6,6>(0,6).transpose());
            print("Mmm", M.block<6,6>(6,6).transpose());
        }
        
        robot_dyn->forward_iterations(qd_.data(),qdd_.data(), 12);
        robot_dyn->cal_momentum(m_ang.data(), m_lin.data(), 3);
        Eigen::Matrix4d T1;
        robot_dyn->get_pose(1, T1.data(), 16);
        pos += T1.block<3,3>(0,0)*qd_.segment<3>(3)*dt;
        //if (i%4==0)
            data_recorder.record_frame(i*dt, q_.segment<4>(0), qd_.segment<3>(0), q_.segment<3>(4), qd_.segment<3>(3), q_.segment<6>(7), qd_.segment<6>(6), tau_cal, q_.segment<6>(7) - q_des, m_ang, m_lin);
        
        integrator.integrate(robot_dynamics_list, robot_data_list);

        
        //std::cout << i << "-th del q: " << (q_ - q_des).transpose() << std::endl;
   }
   data_recorder.save("data_floating.txt");

    return 0;
}