#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32

#include <graph.h>
#include <robot_dynamics.h>
#include <iostream>
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
    // Build a floating-base serial manipulator with six revolute joints.
    int num = 8;
    shared_ptr<Graph> graph(createGraph(num), deleteGraph);
    graph->add_edge(0,1); 
    graph->add_edge(1,2); 
    graph->add_edge(2,3);
    graph->add_edge(3,4);
    graph->add_edge(4,5);
    graph->add_edge(5,6);
    graph->add_edge(6,7);
    
    graph->print();

    double pi = 3.141592653589793238462643383279502884;

    // Create the robot dynamics model and configure its link transforms.
    shared_ptr<RobotDynamics> robot_dyn(createRobotDynamics(graph.get()),deleteRobotDynamics);
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

    robot_dyn->set_fixed_transformation(1, T[1].data(), 16);
    robot_dyn->set_fixed_transformation(2, T[2].data(), 16);
    robot_dyn->set_fixed_transformation(3, T[3].data(), 16);
    robot_dyn->set_fixed_transformation(4, T[4].data(), 16);
    robot_dyn->set_fixed_transformation(5, T[5].data(), 16);
    robot_dyn->set_fixed_transformation(6, T[6].data(), 16);
    robot_dyn->set_fixed_transformation(7, T[7].data(), 16);
    std::cout << "Robot model: modified Denavit-Hartenberg transforms configured." << std::endl;
    robot_dyn->set_joint_type(1,JointType::FLOATING, 0);
    robot_dyn->set_joint_type(2,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(3,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(4,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(5,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(6,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(7, JointType::REVOLUTE, 0);
    std::cout << "Robot model: floating base and six revolute joints configured." << std::endl;

    DynamicParam link0_param(29.053, 0.0, 0.0, 29.053, 0.0, 200.43, 777.8, 0.00008, 0.00244, -0.037);
    // Set inertial parameters for the floating base and six arm links.
    DynamicParam link1_param(0.029053, 0.0, 0.0, 0.029053, 0.0, 3.0243, 7.778, 0.00008, 0.00244, -0.037);   // Shoulder link.
    DynamicParam link2_param(0.023247, 0.0, 0.0, 1.157850, 0.0, 5.157850, 12.930, 0.00001, 0.15061, 0.38757);  // Upper-arm link.
    DynamicParam link3_param(0.012586, 0.0, 0.0, 0.408420, 0.0, 2.408420, 3.870, -0.00012, 0.06112, 0.1984);   // Forearm link.
    DynamicParam link4_param(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.25625, 1.960, -0.00021, -0.00112, 0.02269);  // First wrist link.
    DynamicParam link5_param(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.25625, 1.960, -0.00021, 0.00112, 0.002269); // Second wrist link.
    DynamicParam link6_param(0.000405, 0.0, 0.0, 0.000405, 0.0, 0.1577, 0.202, 0.0, -0.001156, -0.00149);  // Third wrist link.

    robot_dyn->set_dyn_param(1, link0_param);
    robot_dyn->set_dyn_param(2,link1_param);
    robot_dyn->set_dyn_param(3,link2_param);
    robot_dyn->set_dyn_param(4,link3_param);
    robot_dyn->set_dyn_param(5,link4_param);
    robot_dyn->set_dyn_param(6,link5_param);
	robot_dyn->set_dyn_param(7,link6_param);

    robot_dyn->build_model();
    // Initialize the floating-base pose and generalized velocities/accelerations.
    Eigen::VectorXd q(13), qd(12), qdd(12);
	q << 0, 0, 0, 1, 0, 0, 0, 0, -pi / 4, pi / 2, -pi / 2, -pi / 3, pi / 3;
    qd << 0.01, -0.03, 0.05, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0;
    qdd.setZero();
     

    Eigen::VectorXd a0(6); 
    a0.setZero();	
    robot_dyn->forward_kinematics(q.data(),q.size());
    Eigen::MatrixXd M(12,12);
    Eigen::MatrixXd C(12,12);
    Eigen::VectorXd g(12);
	auto start = Clock::now();
	auto end = Clock::now();
    for (size_t i=0; i<20000; i++)
    {
        robot_dyn->forward_kinematics(q.data(),q.size());
        robot_dyn->forward_dynamics_crba(M.data(),qd.size());
    }
    end = Clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Average floating-base CRBA calculation time: "
         << duration.count()/20000 << " ns/call" << endl;
	robot_dyn->cal_coriolis_matrix(qd.data(), C.data(), qd.size());
	robot_dyn->cal_gravitational_torque(g.data(), g.size());
    Eigen::VectorXd tau = M*qdd +C*qd+ g;


    print("Floating-base inertia matrix block:", M.block<6,6>(0,0));
    print("Joint-space inertia matrix block:", M.block<6,6>(6,6));
    print("Base-to-joint inertia matrix block:", M.block<6,6>(0,6));
	print("Generalized gravity force vector:", g.transpose());
    print("Generalized inverse-dynamics force vector:", tau.transpose());
    
    start = Clock::now();
    size_t N = 20000;
    for (size_t i = 0; i < N; i++)
    {   
        
        robot_dyn->forward_kinematics(q.data(),q.size());
        robot_dyn->forward_iterations(qd.data(),qdd.data(),qd.size());
        robot_dyn->backward_iterations(tau.data(),tau.size());
    }
    end = Clock::now();
    auto duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Average recursive inverse-dynamics evaluation time: "
         << duration_.count()/N << " ns/call" << endl;
    print("Generalized force from inverse dynamics:", tau.transpose());
    cout << "Forward-dynamics verification using the articulated-body algorithm (ABA):" << endl;

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
    cout << "Average articulated-body forward-dynamics evaluation time: "
         << duration_.count() / N << " ns/call" << endl;
	print("Generalized acceleration from ABA:", qdd_aba.transpose());
	print("Generalized acceleration from the mass-matrix solution:", -1.0*(M.inverse()*(C * qd + g)).transpose());

    robot_dyn->forward_iterations(qd.data(),qdd_aba.data(), qd.size());
	robot_dyn->backward_iterations(tau.data(), tau.size());
	print("Generalized force reconstructed from ABA acceleration:", tau.transpose());

    // Evaluate the center-of-mass state and Jacobian.
    Eigen::Vector3d com, com_vel;
    robot_dyn->cal_center_of_mass(com.data(),com.size());
    robot_dyn->cal_center_of_mass_vel(qd.data(), qd.size(), com_vel.data(), com_vel.size());
    print("Center-of-mass position [m]:", com.transpose());
    print("Center-of-mass velocity [m/s]:", com_vel.transpose());
    Eigen::Matrix<double, 3, 12> J_com;
    start = Clock::now();
    for (size_t i = 0; i < N; i++)
    {
        robot_dyn->cal_jacobian_com(J_com.data(), J_com.rows(), J_com.cols());
    }
    end = Clock::now();
    duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Average center-of-mass Jacobian calculation time: "
         << duration_.count() / N << " ns/call" << endl;

    print("Center-of-mass Jacobian:", J_com);
    print("Center-of-mass velocity from the Jacobian:", (J_com*qd).transpose());

    double dt = 0.001;
   N = 400/dt;
   RDL_UTILS::RobotDynamicsIntegrator integrator(dt);
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

   Eigen::VectorXd a = Eigen::VectorXd::Zero(6);
   robot_dyn->set_spatial_gravitational_acc(a.data(), 6);

   tau_cal.setZero();
   Eigen::Matrix<double,6,6> M_bar;
   Eigen::Matrix<double,6,1> h_bar;

   std::vector<RDL::RobotDynamics*> robot_dynamics_list = {robot_dyn.get()};
   std::vector<RDL::RobotData*> robot_data_list = {&robot_data};
   for (size_t i = 0; i < N; i++)
   {
        Eigen::VectorXd q_des = q0.segment<6>(7) + mag*sin(omega*i*dt)* Eigen::VectorXd::Ones(6);      
        Eigen::VectorXd qd_des = omega*mag*cos(omega*i*dt)* Eigen::VectorXd::Ones(6);
        Eigen::VectorXd qdd_des = -omega*omega*mag*sin(omega*i*dt)* Eigen::VectorXd::Ones(6);
        
        robot_dyn->forward_kinematics(q_.data(), q_.size());
        robot_dyn->forward_dynamics_crba(M.data(), robot_data.dof_dyn);
        robot_dyn->cal_nonlinear_effect(robot_data.qd, h_.data(), h_.size());
        M_bar = M.block<6,6>(6,6) - M.block<6,6>(6,0)*M.block<6,6>(0,0).inverse()*M.block<6,6>(0,6);
        h_bar = h_.segment<6>(6) - M.block<6,6>(6,0)*M.block<6,6>(0,0).inverse()*h_.segment<6>(0);
        tau_cal.segment<6>(6) = M_bar*(qdd_des-Kp * (q_.segment<6>(7) - q_des) - Kd*(qd_.segment<6>(6) - qd_des)) + h_bar;

        if (i==0)
        {
            print("Initial generalized force vector:", tau.transpose());
            print("Initial reduced nonlinear-effects vector:", h_bar.transpose());
            print("Initial floating-base inertia block:", M.block<6,6>(0,0).transpose());
            print("Initial base-to-joint inertia block:", M.block<6,6>(0,6).transpose());
            print("Initial joint-space inertia block:", M.block<6,6>(6,6).transpose());
        }
        
        robot_dyn->forward_iterations(qd_.data(),qdd_.data(), 12);
        robot_dyn->cal_momentum(m_ang.data(), m_lin.data(), 3);
        data_recorder.record_frame(i*dt, q_.segment<4>(0), qd_.segment<3>(0), q_.segment<3>(4), qd_.segment<3>(3), q_.segment<6>(7), qd_.segment<6>(6), tau_cal, q_.segment<6>(7) - q_des, m_ang, m_lin);
        
        integrator.integrate(robot_dynamics_list, robot_data_list);

        
   }
   data_recorder.save("data_floating.txt");

    return 0;
}