#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32

#include <graph.h>
#include <robot_dynamics.h>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "./utils/utils.h"
#include <math_utils.h>



using namespace std;
using namespace RDL;
using namespace Eigen;

void cal_3dof_dynamics(vector<double> q, vector<double> qd, vector<double> qdd, DynamicParam dp1, DynamicParam dp2, DynamicParam dp3, Matrix3d& M, Matrix3d& C, Vector3d& g)
{
    double m1 = dp1.m;
    double m2 = dp2.m;
    double m3 = dp3.m;

    double Ixx1 = dp1.Ixx;
    double Iyy1 = dp1.Iyy;
    double Izz1 = dp1.Izz;
    double Ixx2 = dp2.Ixx;
    double Iyy2 = dp2.Iyy;
    double Izz2 = dp2.Izz;
    double Ixx3 = dp3.Ixx;
    double Iyy3 = dp3.Iyy;
    double Izz3 = dp3.Izz;
    
    double lc1 = dp1.zc;
    double lc2 = dp2.xc;
    double lc3 = dp3.xc;
    double d1 = 0.2;
    double l2 = 0.6;
    double l3 = 0.5;

    double q1 = q[0];
    double q2 = q[1];
    double q3 = q[2];
    double qd1 = qd[0];
    double qd2 = qd[1];
    double qd3 = qd[2];

    M(0, 0) = Ixx2 * sin(q2) * sin(q2) + Ixx3 * sin(q2 + q3) * sin(q2 + q3) + Iyy2 * cos(q2) * cos(q2) + Iyy3 * cos(q2 + q3) * cos(q2 + q3) + Izz1 + lc2 * lc2 * m2 * sin(q1) * sin(q1) * cos(q2) * cos(q2) + lc2 * lc2 * m2 * cos(q1) * cos(q1) * cos(q2) * cos(q2) + m3 * (l2 * cos(q2) + lc3 * cos(q2 + q3)) * (l2 * cos(q2) + lc3 * cos(q2 + q3)) * sin(q1) * sin(q1) + m3 * (l2 * cos(q2) + lc3 * cos(q2 + q3)) * (l2 * cos(q2) + lc3 * cos(q2 + q3)) * cos(q1) * cos(q1);
    M(0, 1) = 0;
    M(0, 2) = 0;
    M(1, 0) = M(0, 1);
    M(1, 1) = Izz2 + Izz3 + l2 * l2 * m3 + 2 * l2 * lc3 * m3 * cos(q3) + lc2 * lc2 * m2 + lc3 * lc3 * m3;
    M(1, 2) = Izz3 + l2 * lc3 * m3 * cos(q3) + lc3 * lc3 * m3;
    M(2, 0) = M(0, 2);
    M(2, 1) = M(1, 2);
    M(2, 2) = Izz3 + lc3 * lc3 * m3;
    
    C(0, 0) = -0.5 * qd2 * (-Ixx2 * sin(2 * q2) - Ixx3 * sin(2 * q2 + 2 * q3) + Iyy2 * sin(2 * q2) + Iyy3 * sin(2 * q2 + 2 * q3) + l2 * l2 * m3 * sin(2 * q2) + 2 * l2 * lc3 * m3 * sin(2 * q2 + q3) + lc2 * lc2 * m2 * sin(2 * q2) + lc3 * lc3 * m3 * sin(2 * q2 + 2 * q3)) - 1.0 * qd3 * (-Ixx3 * cos(q2 + q3) + Iyy3 * cos(q2 + q3) + l2 * lc3 * m3 * cos(q2) + lc3 * lc3 * m3 * cos(q2 + q3)) * sin(q2 + q3);
    C(0, 1) = -1.0 * qd1 * (-Ixx2 * sin(2 * q2) / 2 - Ixx3 * sin(2 * q2 + 2 * q3) / 2 + Iyy2 * sin(2 * q2) / 2 + Iyy3 * sin(2 * q2 + 2 * q3) / 2 + l2 * l2 * m3 * sin(2 * q2) / 2 + l2 * lc3 * m3 * sin(2 * q2 + q3) + lc2 * lc2 * m2 * sin(2 * q2) / 2 + lc3 * lc3 * m3 * sin(2 * q2 + 2 * q3) / 2);
    C(0, 2) = -1.0 * qd1 * (-Ixx3 * cos(q2 + q3) + Iyy3 * cos(q2 + q3) + l2 * lc3 * m3 * cos(q2) + lc3 * lc3 * m3 * cos(q2 + q3)) * sin(q2 + q3);
    C(1, 0) = 1.0 * qd1 * (-Ixx2 * sin(2 * q2) / 2 - Ixx3 * sin(2 * q2 + 2 * q3) / 2 + Iyy2 * sin(2 * q2) / 2 + Iyy3 * sin(2 * q2 + 2 * q3) / 2 + l2 * l2 * m3 * sin(2 * q2) / 2 + l2 * lc3 * m3 * sin(2 * q2 + q3) + lc2 * lc2 * m2 * sin(2 * q2) / 2 + lc3 * lc3 * m3 * sin(2 * q2 + 2 * q3) / 2);
    C(1, 1) = -1.0 * l2 * lc3 * m3 * qd3 * sin(q3);
    C(1, 2) = 1.0 * l2 * lc3 * m3 * (-qd2 - qd3) * sin(q3);
    C(2, 0) = 1.0 * qd1 * (-Ixx3 * cos(q2 + q3) + Iyy3 * cos(q2 + q3) + l2 * lc3 * m3 * cos(q2) + lc3 * lc3 * m3 * cos(q2 + q3)) * sin(q2 + q3);
    C(2, 1) = 1.0 * l2 * lc3 * m3 * qd2 * sin(q3);
    C(2, 2) = 0;

    g(0) = 0;
    g(1) = g_acc * (lc2 * m2 * cos(q2) + m3 * (l2 * cos(q2) + lc3 * cos(q2 + q3)));
    g(2) = g_acc * lc3 * m3 * cos(q2 + q3);
}

int main()
{
    // create a two-DOF robot manipulator, 0 represents the base frame
    shared_ptr<Graph> graph(createGraph(4),deleteGraph);
    // 0->1->2->3
    graph->add_edge(0,1); 
    graph->add_edge(1,2); 
    graph->add_edge(2,3);
    
    graph->print();

    // create the dynamics 
    shared_ptr<RobotDynamics> robot_dynamics(createRobotDynamics(graph.get()),deleteRobotDynamics);
    printf("-2\n");
    MDH dh0(0,0,0,0,false);
    MDH dh1(0,0,0,0.2,false);
    MDH dh2(M_PI /2,0,0,0,false);
    MDH dh3(0, 0, 0.6, 0, false);
    Matrix4d T0, T1, T2, T3;
    printf("-1\n");
    mdh_to_transformation(dh0, JointType::FIXED, 0, T0.data(), 16);
    mdh_to_transformation(dh1, JointType::REVOLUTE, 0, T1.data(), 16);
    mdh_to_transformation(dh2, JointType::REVOLUTE, 0, T2.data(), 16);
    mdh_to_transformation(dh3, JointType::REVOLUTE, 0, T3.data(), 16);
    printf("0\n");
    //robot_dynamics->set_auxiliary_Trans(0, T0.data(), 16);
    //robot_dynamics->set_auxiliary_Trans(1, T1.data(), 16);
    //robot_dynamics->set_auxiliary_Trans(2, T2.data(), 16);
    //robot_dynamics->set_auxiliary_Trans(3, T3.data(), 16);
    //robot_dynamics->set_mdh(0,dh0);
    robot_dynamics->set_mdh(1, dh1);
    robot_dynamics->set_mdh(2, dh2);
    robot_dynamics->set_mdh(3, dh3);
    printf("1\n");
    Eigen::Vector3d axis;
    axis << 0,0,1;
    //robot_dynamics->set_joint_type(0, JointType::FIXED);
    robot_dynamics->set_joint_type(1, JointType::REVOLUTE, 0);
    robot_dynamics->set_joint_type(2, JointType::REVOLUTE, 0);
    robot_dynamics->set_joint_type(3, JointType::REVOLUTE, 0);
    printf("2\n");
    DynamicParam dp0(0.5, 0, 0, 0.5, 0, 1, 1, 0.0, 0, -0.1);
    DynamicParam dp1(0.5,0,0,0.5,0,1,1,0.0,0,-0.1);
    DynamicParam dp2(1,0,0,1,0,2,1.5,0.3,0,0);
    DynamicParam dp3(1,0,0,1,0,2,1.5,0.25,0,0);
    //robot_dynamics->set_dyn_param(0, dp0);
    robot_dynamics->set_dyn_param(1,dp1);
    robot_dynamics->set_dyn_param(2,dp2);
    robot_dynamics->set_dyn_param(3,dp3);

    robot_dynamics->build_model();

    vector<double> q, qd, qdd;
    q.resize(3);
    qd.resize(3);
    qdd.resize(3);

    q[0] = 1 * M_PI / 4; q[1] = 1 * M_PI / 2; q[2] = -1* M_PI / 3;
    qd[0] = 0.8; qd[1] = 1.3; qd[2] = -0.4;
    qdd[0] = 0.3; qdd[1] = 0.2; qdd[2] = 0.8;
    cout << "set joint: " << endl;
    //robot_dynamics->set_joint_pos(q.data(),q.size());
    //robot_dynamics->set_joint_vel(qd.data(),qd.size());
    //robot_dynamics->set_joint_acc(qdd.data(),qdd.size());
    cout << "cal torque: " << endl;
    robot_dynamics->forward_kinematics(q.data(),q.size());
    robot_dynamics->forward_iterations(qd.data(),qdd.data(),qd.size());
    
    vector<long long> times1;
    Eigen::MatrixXd M(3,3);// = get_M(robot_dynamics);
    Eigen::MatrixXd C(3,3);// = get_C(robot_dynamics);
    Eigen::VectorXd g(3);// = get_g(robot_dynamics);
	auto start = Clock::now();
	auto end = Clock::now();
    for (size_t i=0; i<100; i++)
    {
        start = Clock::now();
        robot_dynamics->forward_dynamics_crba(qd.data(),M.data(),C.data(),g.data(),qd.size());
        //robot_dynamics->cal_inertia_matrix();
        //robot_dynamics->cal_coriolis_matrix();
        //robot_dynamics->cal_gravitational_torque();
        end = Clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
		times1.push_back(duration.count());
    }
    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    //cout << "cal inertia, coriolis, gravitational time: " << duration.count() << " us" << endl;
    for (size_t i = 0; i < 100; i++)
    {
		cout << "cal inertia, coriolis, gravitational time: " << times1[i] << " us" << endl;
    }

    Map<Vector3d> qd_(qd.data(), 3);

    Matrix3d M1, C1;
    Vector3d g1;
    cal_3dof_dynamics(q, qd, qdd, dp1, dp2, dp3, M1, C1, g1);

    print("M ", M);
    print("M1", M1);
    print("C ", C);
    print("C1", C1);
    print("g ", g.transpose());
    print("g1", g1.transpose());
    print("Cqd ", (C * qd_).transpose());
    print("Cqd1", (C1 * qd_).transpose());

    Map<Vector3d> qdd_(qdd.data(), 3);
    Vector3d tau_ = M * qdd_ + C * qd_ + g;
    Vector3d tau;
    vector<long long> times;
    start = Clock::now();
    size_t N = 10000;
    for (size_t i = 0; i < N; i++)
    {   
        
        robot_dynamics->forward_kinematics(q.data(),q.size());
        robot_dynamics->forward_iterations(qd.data(),qdd.data(), qd.size());
        robot_dynamics->backward_iterations(tau.data(),tau.size());
        
        //cout << "backward iterations time: " << duration_.count() << " us" << endl;
        //printf("elapsed time: %lld \n", duration_.count());
        //times.push_back(duration_.count());
    }
    end = Clock::now();
    auto duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "average iterations time: " << duration_.count()/N << " ns" << endl;
    //for (size_t i = 0; i < 100; i++)
    {
        //cout << "backward iterations time: " << times[i] << " ns" << endl;
    }
    
    print("tau_", tau_.transpose());
    print("tau ", tau.transpose());

    return 0;
}