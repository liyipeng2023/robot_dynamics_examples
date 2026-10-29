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

using namespace std;
using namespace RDL;
using namespace Eigen;

int main()
{
    Eigen::setNbThreads(1);
    // Build a six-joint serial manipulator with link 0 as the fixed base.
    int num = 7;
    shared_ptr<Graph> graph(createGraph(num), deleteGraph);
    graph->add_edge(0,1); 
    graph->add_edge(1,2); 
    graph->add_edge(2,3);
    graph->add_edge(3,4);
    graph->add_edge(4,5);
    graph->add_edge(5,6);
    
    graph->print();

    // Create the robot dynamics model using modified Denavit-Hartenberg parameters.
    shared_ptr<RobotDynamics> robot_dyn(createRobotDynamics(graph.get()),deleteRobotDynamics);
	MDH dh0(0, 0, 0, 0, false);
    MDH dh1(0,0,0,0.1273,false);
    MDH dh2(-M_PI/2,0,0,0,false);
    MDH dh3(0,0,0.612,0,false);
    MDH dh4(0,0,0.5723,0.1639,false);
    MDH dh5(-M_PI/2,0,0,0.1157,false);
    MDH dh6( M_PI/2,0,0,0.0922,false);

    vector<Eigen::Matrix4d> T;
    T.resize(7);
    mdh_to_transformation(dh0, JointType::REVOLUTE, 0, T[0].data(), 16);
    mdh_to_transformation(dh1, JointType::REVOLUTE, 0, T[1].data(), 16);
    mdh_to_transformation(dh2, JointType::REVOLUTE, 0, T[2].data(), 16);
    mdh_to_transformation(dh3, JointType::REVOLUTE, 0, T[3].data(), 16);
    mdh_to_transformation(dh4, JointType::REVOLUTE, 0, T[4].data(), 16);
    mdh_to_transformation(dh5, JointType::REVOLUTE, 0, T[5].data(), 16);
    mdh_to_transformation(dh6, JointType::REVOLUTE, 0, T[6].data(), 16);

    robot_dyn->set_fixed_transformation(1, T[1].data(), 16);
    robot_dyn->set_fixed_transformation(2, T[2].data(), 16);
    robot_dyn->set_fixed_transformation(3, T[3].data(), 16);
    robot_dyn->set_fixed_transformation(4, T[4].data(), 16);
    robot_dyn->set_fixed_transformation(5, T[5].data(), 16);
    robot_dyn->set_fixed_transformation(6, T[6].data(), 16);
    std::cout << "Robot model: modified Denavit-Hartenberg transforms configured." << std::endl;
    robot_dyn->set_joint_type(1,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(2,JointType::REVOLUTE,0);
    robot_dyn->set_joint_type(3,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(4,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(5,JointType::REVOLUTE, 0);
    robot_dyn->set_joint_type(6, JointType::REVOLUTE, 0);
    std::cout << "Robot model: six revolute joints configured." << std::endl;

    DynamicParam link0_param(0.029053, 0.0, 0.0, 0.029053, 0.0, 0.0243, 7.778, 0.00008, 0.00244, -0.037);
    DynamicParam link1_param(0.029053, 0.0, 0.0, 0.029053, 0.0, 0.0243,   7.778,  0.00008,  0.00244,  -0.037);   // Shoulder link.
    DynamicParam link2_param(0.023247, 0.0, 0.0, 1.157850, 0.0, 1.157850, 12.930, 0.00001,  0.15061,  0.38757);  // Upper-arm link.
    DynamicParam link3_param(0.012586, 0.0, 0.0, 0.408420, 0.0, 0.408420,  3.870, -0.00012, 0.06112,  0.1984);   // Forearm link.
    DynamicParam link4_param(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.005625,  1.960, -0.00021, -0.00112, 0.02269);  // First wrist link.
    DynamicParam link5_param(0.005267, 0.0, 0.0, 0.005267, 0.0, 0.005625,  1.960, -0.00021, 0.00112,  0.002269); // Second wrist link.
    DynamicParam link6_param(0.000405, 0.0, 0.0, 0.000405, 0.0, 0.000577,  0.202,  0.0,     -0.001156,-0.00149);  // Third wrist link.

    robot_dyn->set_dyn_param(1,link1_param);
    robot_dyn->set_dyn_param(2,link2_param);
    robot_dyn->set_dyn_param(3,link3_param);
    robot_dyn->set_dyn_param(4,link4_param);
    robot_dyn->set_dyn_param(5,link5_param);
	robot_dyn->set_dyn_param(6,link6_param);

    robot_dyn->build_model();

    // Set the joint position, velocity, and acceleration test state.
    Eigen::VectorXd q(6), qd(6), qdd(6);
	q << 0, -M_PI / 4, 0, -M_PI / 2, 0, M_PI / 3;
	qd << 0.27, -0.5, -0.2, 0.7, -1.3, 0.2;
    qdd.setZero();

    robot_dyn->forward_kinematics(q.data(),q.size());
    Eigen::MatrixXd M(6,6);
    Eigen::MatrixXd C(6,6);
    Eigen::VectorXd g(6);
	auto start = Clock::now();
	auto end = Clock::now();
    for (size_t i=0; i<20000; i++)
    {
        robot_dyn->forward_kinematics(q.data(),q.size());
        robot_dyn->forward_dynamics_crba(qd.data(),M.data(),C.data(),g.data(),qd.size());
    }
    end = Clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Average CRBA mass, Coriolis, and gravity calculation time: "
         << duration.count()/20000 << " ns/call" << endl;
    
    Eigen::VectorXd tau = M*qdd +C*qd+ g;


    print("Joint-space inertia matrix M:", M);
	print("Coriolis matrix C:", C);
	print("Gravity torque vector g:", g.transpose());
    print("Inverse-dynamics torque vector tau:", tau.transpose());

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

    cout << "Forward-dynamics verification using the articulated-body algorithm (ABA):" << endl;
	vector<SpatialVector> f_ext(7, SpatialVector());
    for (size_t i = 0; i < 7; i++)
    {
		f_ext[i].data[0] = 0;
		f_ext[i].data[1] = 0;
		f_ext[i].data[2] = 0;
		f_ext[i].data[3] = 0;
		f_ext[i].data[4] = 0;
		f_ext[i].data[5] = 0;
    }
	Eigen::VectorXd qdd_aba(6);
    Eigen::Matrix<double, 6, 1> zero_tau;
	zero_tau.setZero();
    start = Clock::now();
    for (size_t i = 0; i < N; i++)
	{
        robot_dyn->forward_kinematics(q.data(),q.size());
        robot_dyn->forward_dynamics_aba(qd.data(),zero_tau.data(), qdd_aba.data(), 6, false);
    }
    end = Clock::now();
    duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Average articulated-body forward-dynamics evaluation time: "
         << duration_.count() / N << " ns/call" << endl;
	print("Joint acceleration from ABA [rad/s^2]:", qdd_aba.transpose());
	print("Joint acceleration from the mass-matrix solution [rad/s^2]:", -1.0*(M.inverse()*(C * qd + g)).transpose());

    Eigen::Matrix4d T1, T2, T3, T4, T5, T6;
    robot_dyn->get_pose(1, T1.data(), 16);
    robot_dyn->get_pose(2, T2.data(), 16);
    robot_dyn->get_pose(3, T3.data(), 16);
    robot_dyn->get_pose(4, T4.data(), 16);
    robot_dyn->get_pose(5, T5.data(), 16);
    robot_dyn->get_pose(6, T6.data(), 16);
    return 0;
}