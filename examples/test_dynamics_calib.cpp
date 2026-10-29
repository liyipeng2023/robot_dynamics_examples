#include <graph.h>
#include <robot_dynamics.h>
#include <dynamics_calibration.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "./utils/utils.h"

using namespace std;
using namespace RDL;

#define PI 3.14159265358979323846

void get_excitation_trajectory(double t, double* q, double* qd, double* qdd, int dof) {
    // Combine two excitation frequencies to provide a broad range of motion.
    double w1 = 0.2 * 2.0 * PI;
    double w2 = 0.3 * 2.0 * PI;

    for (int i = 0; i < dof; ++i) {
        // Offset the joint phases to avoid synchronized motion.
        double phase = (i == 0) ? 0.0 : PI / 4.0;

        // Use a two-frequency sinusoidal trajectory and its analytic derivatives.
        double A = 0.4, B = 0.2;
        q[i] = A * sin(w1 * t + phase) + B * cos(w2 * t);
        qd[i] = A * w1 * cos(w1 * t + phase) - B * w2 * sin(w2 * t);
        qdd[i] = -A * w1 * w1 * sin(w1 * t + phase) - B * w2 * w2 * cos(w2 * t);
    }
}

int main()
{
    cout << "========== 1. Creating Robot Topology and Dynamics Model ==========" << endl;
    // Create a two-joint serial manipulator with link 0 as the fixed base.
    shared_ptr<Graph> graph(createGraph(3));
    graph->add_edge(0, 1);
    graph->add_edge(1, 2);
    

    shared_ptr<RobotDynamics> robot(createRobotDynamics(graph.get()), deleteRobotDynamics);

    // Configure the modified Denavit-Hartenberg parameters.
    MDH dh1(PI / 2, 0, 0, 0, false);
    MDH dh2(0, 0, 0.6, 0, false);
    robot->set_mdh(1, dh1);
    robot->set_mdh(2, dh2);
    robot->set_joint_type(1, JointType::REVOLUTE, 0);
    robot->set_joint_type(2, JointType::REVOLUTE, 0);

    // Set ground truth dynamic parameters for simulation
    DynamicParam real_dp1(1, 0, 0, 1, 0, 2, 2, 0.3, 0, 0);
    DynamicParam real_dp2(1, 0, 0, 1, 0, 2, 1.5, 0.25, 0, 0);
    robot->set_dyn_param(1, real_dp1);
    robot->set_dyn_param(2, real_dp2);

    robot->build_model();
    
    Eigen::Matrix<double, 10 ,1> I0, I1;
    I0 << 0.2, 0, 0, 0.32, 0, 1.02, 0.6, 0, 0, 3.0;
    I1 << 0.2, 0, 0, 0.12, 0, 0.82, 0.2, 0, 0, 2;

    cout << "========== 2. Initializing Dynamics Calibration Module ==========" << endl;
    shared_ptr<DynamicsCalibration> calibrator(
        createDynamicsCalibration(robot.get()),
        destroyDynamicsCalibration
    );
    calibrator->set_LM_params(0.001, 100, 1e-8);

    int num_samples = 100;
    int dof = 2;
    cout << "========== 3. Generating " << num_samples << " Dynamics Measurements ==========" << endl;

    for (int t_idx = 0; t_idx < num_samples; ++t_idx) {
        double t = t_idx * PI/40; // Sample interval is PI/40 seconds.
        vector<double> q, qd, qdd;
        q.resize(dof);
        qd.resize(dof);
        qdd.resize(dof);

        get_excitation_trajectory(t, q.data(), qd.data(), qdd.data(), dof);

        // Compute the reference torque from the configured dynamics model.
        robot->forward_kinematics(q.data(),q.size());
        robot->forward_iterations(qd.data(),qdd.data(),qd.size());
        Eigen::Vector2d tau;
        robot->backward_iterations(tau.data(),tau.size());
        // Add the state and reference torque to the calibration data set.
        calibrator->add_measurement(q.data(), dof, qd.data(), qdd.data(), tau.data(), dof);

        cout << "Measurement " << t_idx + 1 << "/" << num_samples
             << " recorded; reference joint torques [N m]: ["
             << tau(0) << ", " << tau(1) << "]" << endl;
    }

    cout << "========== 4. Executing Calibration Algorithm ==========" << endl;
    calibrator->calibrate();
    cout << "Calibration status: completed." << endl;

    cout << "========== 5. Verifying Calibration Results ==========" << endl;
	double cal_dp1_data[10], cal_dp2_data[10];
    calibrator->get_calibrated_param(0, cal_dp1_data, 10);
    calibrator->get_calibrated_param(1, cal_dp2_data, 10);

    cout << "\n[Link 1 dynamic-parameter comparison]" << endl;
    cout << "Mass [kg] - reference: " << real_dp1.m << " | calibrated: " << cal_dp1_data[9] << endl;
	cout << "First moment mx [kg m] - reference: " << real_dp1.xc*real_dp1.m << " | calibrated: " << cal_dp1_data[6] << endl;
	cout << "First moment my [kg m] - reference: " << real_dp1.yc*real_dp1.m << " | calibrated: " << cal_dp1_data[7] << endl;
	cout << "First moment mz [kg m] - reference: " << real_dp1.zc*real_dp1.m << " | calibrated: " << cal_dp1_data[8] << endl;
    cout << "Inertia Izz [kg m^2] - reference: " << I0[5] << " | calibrated: " << cal_dp1_data[5] << endl;

    cout << "\n[Link 2 dynamic-parameter comparison]" << endl;
    cout << "Mass [kg] - reference: " << real_dp2.m << " | calibrated: " << cal_dp2_data[9] << endl;
	cout << "First moment mx [kg m] - reference: " << real_dp2.xc*real_dp2.m << " | calibrated: " << cal_dp2_data[6] << endl;
    cout << "First moment my [kg m] - reference: " << real_dp2.yc*real_dp2.m << " | calibrated: " << cal_dp2_data[7] << endl;
    cout << "First moment mz [kg m] - reference: " << real_dp2.zc*real_dp2.m << " | calibrated: " << cal_dp2_data[8] << endl;
    cout << "Inertia Izz [kg m^2] - reference: " << I1[5] << " | calibrated: " << cal_dp2_data[5] << endl;

	cout << "\n========== 6. Comparing Dynamics with Reference and Calibrated Parameters ==========" << endl;
    vector<double> q, qd, qdd;
    q.resize(2);
    qd.resize(2);
    qdd.resize(2);

    q[0] = 1 * PI / 3; q[1] = 1 * PI / 3;
    qd[0] = 0.5; qd[1] = 0.7;
    qdd[0] = 0.11; qdd[1] = -0.35;

    robot->set_dyn_param(1, real_dp1);
    robot->set_dyn_param(2, real_dp2);
    Eigen::Matrix2d M;
    Eigen::Matrix2d C;
    Eigen::Vector2d g;
    robot->forward_kinematics(q.data(),q.size());
    robot->forward_iterations(qd.data(),qdd.data(),qd.size());
    robot->forward_dynamics_crba(qd.data(),M.data(),C.data(),g.data(),qd.size());
    

    cout << "Dynamics matrices and gravity vector using reference parameters:" << endl;
    print("Joint-space inertia matrix M:", M);
    print("Coriolis matrix C:", C);
	print("Gravity torque vector g:", g.transpose());

    robot->set_spatial_inertia(1, cal_dp1_data, 10);
    robot->set_spatial_inertia(2, cal_dp2_data, 10);
    robot->forward_dynamics_crba(qd.data(), M.data(), C.data(), g.data(), qd.size());

    cout << "Dynamics matrices and gravity vector using calibrated parameters:" << endl;
    print("Joint-space inertia matrix M:", M);
    print("Coriolis matrix C:", C);
    print("Gravity torque vector g:", g.transpose());

    return 0;
}