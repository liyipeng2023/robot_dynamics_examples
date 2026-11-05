//
// Created by liyip on 2026/11/2.
//
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32
#include <iostream>
#include <graph.h>
#define EIGEN_USE_THREADS
#include <Eigen/Core>
#include <cmath>
#include "../utils/utils.h"
#include "../utils/robot_loader.h"
#include <robot_dynamics.h>

using namespace std;
using namespace RDL;
using namespace Eigen;

int main(int argc, char** argv) {
    std::string urdf_path = "ur10.urdf";
    std::shared_ptr<RDL::Graph> graph;
	std::map<std::string, int> joint_map;
    //auto ur10_dyn = RDL::Utils::load_RobotDynamics_from_URDF(urdf_path, joint_map);
	auto ur10_dyn = Utils::load_RobotDynamics(urdf_path,joint_map,Utils::RobotFileType::URDF,false);
    Eigen::VectorXd q(6), qd(6), qdd(6);
	q << 0, -M_PI / 4, 0, -M_PI / 2, 0, M_PI / 3;
	qd << 0.27, -0.5, -0.2, 0.7, -1.3, 0.2;
    qdd.setZero();

    ur10_dyn->forward_kinematics(q.data(),q.size());
    Eigen::MatrixXd M(6,6);
    Eigen::MatrixXd C(6,6);
    Eigen::VectorXd g(6);
	auto start = Clock::now();
	auto end = Clock::now();
    for (size_t i=0; i<20000; i++)
    {
        ur10_dyn->forward_kinematics(q.data(),q.size());
        ur10_dyn->forward_dynamics_crba(qd.data(),M.data(),C.data(),g.data(),qd.size());
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

        ur10_dyn->forward_kinematics(q.data(),q.size());
        ur10_dyn->forward_iterations(qd.data(),qdd.data(),qd.size());
        ur10_dyn->backward_iterations(tau.data(),tau.size());

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
        ur10_dyn->forward_kinematics(q.data(),q.size());
        ur10_dyn->forward_dynamics_aba(qd.data(),zero_tau.data(), qdd_aba.data(), 6, false);
    }
    end = Clock::now();
    duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    cout << "Average articulated-body forward-dynamics evaluation time: "
         << duration_.count() / N << " ns/call" << endl;
	print("Joint acceleration from ABA [rad/s^2]:", qdd_aba.transpose());
	print("Joint acceleration from the mass-matrix solution [rad/s^2]:", -1.0*(M.inverse()*(C * qd + g)).transpose());

	Eigen::Matrix4d T1, T2, T3, T4, T5, T6;
	ur10_dyn->get_pose(1, T1.data(), 16);
	ur10_dyn->get_pose(2, T2.data(), 16);
	ur10_dyn->get_pose(3, T3.data(), 16);
	ur10_dyn->get_pose(4, T4.data(), 16);
	ur10_dyn->get_pose(5, T5.data(), 16);
	ur10_dyn->get_pose(6, T6.data(), 16);
	print("T_end", T6);

    return 0;
}