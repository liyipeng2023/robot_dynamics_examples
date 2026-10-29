#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif // _WIN32

#include <graph.h>
#include <inverse_kinematics.h>
#include <iostream>
#include <Eigen/Dense>
#define EIGEN_USE_THREADS
#include <Eigen/Core>
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
    double q1,q2,q3,q4,q5,q6;
    q1 = q(0); q2 = q(1); q3 = q(2);
    q4 = q(3); q5 = q(4); q6 = q(5);

    R(0,0) = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6);
    R(0,1) = - sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6);
    R(0,2) = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5);

    R(1,0) = - cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6);
    R(1,1) = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1);
    R(1,2) = - cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5);

    R(2,0) = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6);
    R(2,1) = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6);
    R(2,2) = -sin(q2 + q3 + q4)*sin(q5);

    t(0) = d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + d4*sin(q1) + L2*cos(q1)*cos(q2) + d5*sin(q2 + q3 + q4)*cos(q1) + L3*cos(q1)*cos(q2)*cos(q3) - L3*cos(q1)*sin(q2)*sin(q3);
    t(1) = L2*cos(q2)*sin(q1) - d4*cos(q1) - d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + d5*sin(q2 + q3 + q4)*sin(q1) + L3*cos(q2)*cos(q3)*sin(q1) - L3*sin(q1)*sin(q2)*sin(q3);
    t(2) = d1 + L3*sin(q2 + q3) + L2*sin(q2) - d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) - d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4));
}
void cal_jac(const Eigen::VectorXd& q, Eigen::MatrixXd& Jr)
{
    double q1,q2,q3,q4,q5,q6;
    q1 = q(0); q2 = q(1); q3 = q(2);
    q4 = q(3); q5 = q(4); q6 = q(5);

    Jr.resize(6,6);

    Jr(0,0) = d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + d4*cos(q1) - L2*cos(q2)*sin(q1) - d5*sin(q2 + q3 + q4)*sin(q1) - L3*cos(q2)*cos(q3)*sin(q1) + L3*sin(q1)*sin(q2)*sin(q3);
    Jr(0,1) = -cos(q1)*(L2*sin(q2) - d5*cos(q2 + q3 + q4) + L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2) - d6*sin(q2 + q3 + q4)*sin(q5));
    Jr(0,2) = cos(q1)*(d5*cos(q2 + q3 + q4) - L3*cos(q2)*sin(q3) - L3*cos(q3)*sin(q2) + d6*sin(q2 + q3 + q4)*sin(q5));
    Jr(0,3) = cos(q1)*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5));
    Jr(0,4) = -d6*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5));
    Jr(0,5) = 0;

    Jr(1,0) = d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + d4*sin(q1) + L2*cos(q1)*cos(q2) + d5*sin(q2 + q3 + q4)*cos(q1) + L3*cos(q1)*cos(q2)*cos(q3) - L3*cos(q1)*sin(q2)*sin(q3);
    Jr(1,1) = -sin(q1)*(L2*sin(q2) - d5*cos(q2 + q3 + q4) + L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2) - d6*sin(q2 + q3 + q4)*sin(q5));
    Jr(1,2) = sin(q1)*(d5*cos(q2 + q3 + q4) - L3*cos(q2)*sin(q3) - L3*cos(q3)*sin(q2) + d6*sin(q2 + q3 + q4)*sin(q5)); 
    Jr(1,3) = sin(q1)*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5));
    Jr(1,4) = d6*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1));
    Jr(1,5) = 0;

    Jr(2,0) = 0;
    Jr(2,1) = L3*cos(q2 + q3) + L2*cos(q2) + d5*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4)) - d6*sin(q5)*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4));
    Jr(2,2) = L3*cos(q2 + q3) + d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5);
    Jr(2,3) = d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5);
    Jr(2,4) = -d6*sin(q2 + q3 + q4)*cos(q5);
    Jr(2,5) = 0;
    // Rows 3-5 contain the angular-velocity components of the geometric Jacobian.
    Jr(3,0) = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6);
    Jr(3,1) = cos(q6)*sin(q5);
    Jr(3,2) = cos(q6)*sin(q5);
    Jr(3,3) = cos(q6)*sin(q5);
    Jr(3,4) = -sin(q6);
    Jr(3,5) = 0;


    Jr(4,0) = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6);
    Jr(4,1) = -sin(q5)*sin(q6);
    Jr(4,2) = -sin(q5)*sin(q6);
    Jr(4,3) = -sin(q5)*sin(q6);
    Jr(4,4) = -cos(q6);
    Jr(4,5) = 0;


    Jr(5,0) = -sin(q2 + q3 + q4)*sin(q5);
    Jr(5,1) =  cos(q5);
    Jr(5,2) =  cos(q5);
    Jr(5,3) =  cos(q5);
    Jr(5,4) = 0;
    Jr(5,5) = 1;
}
void cal_jac_derivative(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, Eigen::MatrixXd& Jr)
{
    double q1,q2,q3,q4,q5,q6;
    q1 = q(0); q2 = q(1); q3 = q(2);
    q4 = q(3); q5 = q(4); q6 = q(5);

    double dq1, dq2, dq3, dq4, dq5, dq6;
    dq1 = qd(0); dq2 = qd(1); dq3 = qd(2);
    dq4 = qd(3); dq5 = qd(4); dq6 = qd(5);

    Jr.resize(6,6);

    Jr(0,0) = L2*dq2*sin(q1)*sin(q2) - d5*dq1*sin(q2 + q3 + q4)*cos(q1) - d5*dq2*cos(q2 + q3 + q4)*sin(q1) - d5*dq3*cos(q2 + q3 + q4)*sin(q1) - d5*dq4*cos(q2 + q3 + q4)*sin(q1) - L2*dq1*cos(q1)*cos(q2) - d4*dq1*sin(q1) - d6*dq1*cos(q5)*sin(q1) - d6*dq5*cos(q1)*sin(q5) - d6*dq2*sin(q2 + q3 + q4)*sin(q1)*sin(q5) - d6*dq3*sin(q2 + q3 + q4)*sin(q1)*sin(q5) - d6*dq4*sin(q2 + q3 + q4)*sin(q1)*sin(q5) - L3*dq1*cos(q1)*cos(q2)*cos(q3) + L3*dq1*cos(q1)*sin(q2)*sin(q3) + L3*dq2*cos(q2)*sin(q1)*sin(q3) + L3*dq2*cos(q3)*sin(q1)*sin(q2) + L3*dq3*cos(q2)*sin(q1)*sin(q3) + L3*dq3*cos(q3)*sin(q1)*sin(q2) + d6*dq1*cos(q2 + q3 + q4)*cos(q1)*sin(q5) + d6*dq5*cos(q2 + q3 + q4)*cos(q5)*sin(q1);
    Jr(0,1) = dq1*sin(q1)*(L2*sin(q2) - d5*cos(q2 + q3 + q4) + L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2) - d6*sin(q2 + q3 + q4)*sin(q5)) - dq3*cos(q1)*(d5*sin(q2 + q3 + q4) + L3*cos(q2)*cos(q3) - L3*sin(q2)*sin(q3) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq4*cos(q1)*(d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq2*cos(q1)*(L2*cos(q2) + d5*sin(q2 + q3 + q4) + L3*cos(q2)*cos(q3) - L3*sin(q2)*sin(q3) - d6*cos(q2 + q3 + q4)*sin(q5)) + d6*dq5*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
    Jr(0,2) = d6*dq5*sin(q2 + q3 + q4)*cos(q1)*cos(q5) - dq3*cos(q1)*(d5*sin(q2 + q3 + q4) + L3*cos(q2)*cos(q3) - L3*sin(q2)*sin(q3) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq1*sin(q1)*(d5*cos(q2 + q3 + q4) - L3*cos(q2)*sin(q3) - L3*cos(q3)*sin(q2) + d6*sin(q2 + q3 + q4)*sin(q5)) - dq4*cos(q1)*(d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq2*cos(q1)*(d5*sin(q2 + q3 + q4) + L3*cos(q2)*cos(q3) - L3*sin(q2)*sin(q3) - d6*cos(q2 + q3 + q4)*sin(q5));
    Jr(0,3) = d6*dq5*sin(q2 + q3 + q4)*cos(q1)*cos(q5) - dq3*cos(q1)*(d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq4*cos(q1)*(d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq1*sin(q1)*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5)) - dq2*cos(q1)*(d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5));
    Jr(0,4) = d6*dq2*sin(q2 + q3 + q4)*cos(q1)*cos(q5) - d6*dq5*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - d6*dq1*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) + d6*dq3*sin(q2 + q3 + q4)*cos(q1)*cos(q5) + d6*dq4*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
    Jr(0,5) = 0;

    Jr(1,0) = d4*dq1*cos(q1) + d5*dq2*cos(q2 + q3 + q4)*cos(q1) + d5*dq3*cos(q2 + q3 + q4)*cos(q1) + d5*dq4*cos(q2 + q3 + q4)*cos(q1) - d5*dq1*sin(q2 + q3 + q4)*sin(q1) - L2*dq1*cos(q2)*sin(q1) - L2*dq2*cos(q1)*sin(q2) + d6*dq1*cos(q1)*cos(q5) - d6*dq5*sin(q1)*sin(q5) + d6*dq1*cos(q2 + q3 + q4)*sin(q1)*sin(q5) + d6*dq2*sin(q2 + q3 + q4)*cos(q1)*sin(q5) + d6*dq3*sin(q2 + q3 + q4)*cos(q1)*sin(q5) + d6*dq4*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - L3*dq1*cos(q2)*cos(q3)*sin(q1) - L3*dq2*cos(q1)*cos(q2)*sin(q3) - L3*dq2*cos(q1)*cos(q3)*sin(q2) - L3*dq3*cos(q1)*cos(q2)*sin(q3) - L3*dq3*cos(q1)*cos(q3)*sin(q2) + L3*dq1*sin(q1)*sin(q2)*sin(q3) - d6*dq5*cos(q2 + q3 + q4)*cos(q1)*cos(q5);
    Jr(1,1) = d6*dq5*sin(q2 + q3 + q4)*cos(q5)*sin(q1) - dq4*sin(q1)*(d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq1*cos(q1)*(L2*sin(q2) - d5*cos(q2 + q3 + q4) + L3*cos(q2)*sin(q3) + L3*cos(q3)*sin(q2) - d6*sin(q2 + q3 + q4)*sin(q5)) - dq2*sin(q1)*(L2*cos(q2) + d5*sin(q2 + q3 + q4) + L3*cos(q2)*cos(q3) - L3*sin(q2)*sin(q3) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq3*sin(q1)*(d5*sin(q2 + q3 + q4) + L3*cos(q2)*cos(q3) - L3*sin(q2)*sin(q3) - d6*cos(q2 + q3 + q4)*sin(q5));
    Jr(1,2) = dq1*cos(q1)*(d5*cos(q2 + q3 + q4) - L3*cos(q2)*sin(q3) - L3*cos(q3)*sin(q2) + d6*sin(q2 + q3 + q4)*sin(q5)) - dq2*sin(q1)*(d5*sin(q2 + q3 + q4) + L3*cos(q2)*cos(q3) - L3*sin(q2)*sin(q3) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq3*sin(q1)*(d5*sin(q2 + q3 + q4) + L3*cos(q2)*cos(q3) - L3*sin(q2)*sin(q3) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq4*sin(q1)*(d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5)) + d6*dq5*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
    Jr(1,3) = dq1*cos(q1)*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5)) - dq2*sin(q1)*(d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq3*sin(q1)*(d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5)) - dq4*sin(q1)*(d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5)) + d6*dq5*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
    Jr(1,4) = d6*dq5*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - d6*dq1*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + d6*dq2*sin(q2 + q3 + q4)*cos(q5)*sin(q1) + d6*dq3*sin(q2 + q3 + q4)*cos(q5)*sin(q1) + d6*dq4*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
    Jr(1,5) = 0;

    Jr(2,0) = 0;
    Jr(2,1) = dq3*(d5*cos(q2 + q3 + q4) - L3*sin(q2 + q3) + d6*sin(q2 + q3 + q4)*sin(q5)) - dq2*(L3*sin(q2 + q3) + L2*sin(q2) - d5*cos(q2 + q3)*cos(q4) + d5*sin(q2 + q3)*sin(q4) - d6*cos(q2 + q3)*sin(q4)*sin(q5) - d6*sin(q2 + q3)*cos(q4)*sin(q5)) + dq4*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5)) - d6*dq5*cos(q2 + q3 + q4)*cos(q5);
    Jr(2,2) = dq2*(d5*cos(q2 + q3 + q4) - L3*sin(q2 + q3) + d6*sin(q2 + q3 + q4)*sin(q5)) + dq3*(d5*cos(q2 + q3 + q4) - L3*sin(q2 + q3) + d6*sin(q2 + q3 + q4)*sin(q5)) + dq4*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5)) - d6*dq5*cos(q2 + q3 + q4)*cos(q5);
    Jr(2,3) = dq2*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5)) + dq3*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5)) + dq4*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5)) - d6*dq5*cos(q2 + q3 + q4)*cos(q5);
    Jr(2,4) = d6*dq5*sin(q2 + q3 + q4)*sin(q5) - d6*dq3*cos(q2 + q3 + q4)*cos(q5) - d6*dq4*cos(q2 + q3 + q4)*cos(q5) - d6*dq2*cos(q2 + q3 + q4)*cos(q5);
    Jr(2,5) = 0;

    Jr(3,0) = dq6*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - dq3*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)) - dq4*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)) - dq2*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)) - dq5*sin(q2 + q3 + q4)*cos(q6)*sin(q5);
    Jr(3,1) = dq5*cos(q5)*cos(q6) - dq6*sin(q5)*sin(q6);
    Jr(3,2) = dq5*cos(q5)*cos(q6) - dq6*sin(q5)*sin(q6);
    Jr(3,3) = dq5*cos(q5)*cos(q6) - dq6*sin(q5)*sin(q6);
    Jr(3,4) = -dq6*cos(q6);
    Jr(3,5) = 0;

    Jr(4,0) = dq5*sin(q2 + q3 + q4)*sin(q5)*sin(q6) - dq3*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)) - dq4*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)) - dq6*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) - dq2*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6));
    Jr(4,1) = - dq5*cos(q5)*sin(q6) - dq6*cos(q6)*sin(q5);
    Jr(4,2) = - dq5*cos(q5)*sin(q6) - dq6*cos(q6)*sin(q5);
    Jr(4,3) = - dq5*cos(q5)*sin(q6) - dq6*cos(q6)*sin(q5);
    Jr(4,4) = dq6*sin(q6);
    Jr(4,5) = 0;

    Jr(5,0) = - dq2*cos(q2 + q3 + q4)*sin(q5) - dq3*cos(q2 + q3 + q4)*sin(q5) - dq4*cos(q2 + q3 + q4)*sin(q5) - dq5*sin(q2 + q3 + q4)*cos(q5);
    Jr(5,1) = -dq5*sin(q5);
    Jr(5,2) = -dq5*sin(q5);
    Jr(5,3) = -dq5*sin(q5);
    Jr(5,4) = 0;
    Jr(5,5) = 0;
}

int main()
{
    Eigen::setNbThreads(1);
    // Construct a six-joint serial chain; link 0 is the fixed base frame.
    shared_ptr<Graph> graph(createGraph(7), deleteGraph);
    graph->add_edge(0,1); 
    graph->add_edge(1,2); 
    graph->add_edge(2,3);
    graph->add_edge(3,4);
    graph->add_edge(4,5);
    graph->add_edge(5,6);
    
    // Create the kinematics model and define the modified Denavit-Hartenberg parameters.
    shared_ptr<InverseKinematics> robot_kin(createInverseKinematics(graph.get()),deleteInverseKinematics);
    MDH dh0(0, 0, 0, 0, false);
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
    
    robot_kin->set_joint_type(0, JointType::FIXED, 0);
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

    // Joint angles are specified in radians.
    Eigen::VectorXd q(6);
    q<< 0.0000, -1.5708, 0.0000, -1.5708, 1.5708, 0.0000;

    robot_kin->set_joint_pos(q.data(), q.size());
    robot_kin->forward_kinematics();

    // Retrieve the end-effector pose and Jacobian from the model.
    Matrix4d T;
    robot_kin->get_end_effector_pose(6, T.data(), 16);
 
    robot_kin->cal_jacobian();

    MatrixXd J(6,6);
    robot_kin->get_jacobian(J.data(), 6, 6);

    Matrix3d R_;
    Vector3d t_;
    forward_kinematics(q, t_, R_);
    cout << "=== Forward-kinematics comparison ===" << endl;
    print("Model end-effector rotation matrix:", T.block<3,3>(0,0));
    print("Model end-effector position [m]:", T.block<3,1>(0,3).transpose());
    print("Analytical end-effector rotation matrix:", R_);
    print("Analytical end-effector position [m]:", t_.transpose());
    cout << "=== Geometric-Jacobian comparison ===" << endl;
    MatrixXd J_(6,6);
    cal_jac(q, J_);

    print("Model geometric Jacobian:", J);
    print("Analytical geometric Jacobian:", J_);
    cout << "=== Inverse-kinematics test ===" << endl;
    VectorXd q1(6);
    q1 << 0.1, -0.9, 1.1, -1.4, -1.4, 0.1;
    robot_kin->set_joint_pos(q1.data(), q1.size());   
    robot_kin->forward_kinematics();
    robot_kin->get_end_effector_pose(6, T.data(), 16);
    print("Target end-effector pose:", T);

    VectorXd q1_(6);
    q1_ << 0.01, -0.2, 1.2, -1.2, -1.0, 0.2;
    VectorXd q_min(6), q_max(6);
	q_min << -180, -180, -180, -180, -180, -180;
	q_max << 180, 180, 180, 180, 180, 180;
    q_min /= 57.3 * 2;
    q_max /= 57.3 * 2;
	robot_kin->set_joint_limits(q_min.data(), q_max.data(), q_min.size());
    // Warm up the inverse-kinematics solver before measuring execution time.
    robot_kin->set_target_end_effector_pose(6, T.data(), 16);
    for (size_t i = 0; i < 1; i++)
    {
        robot_kin->set_joint_pos(q1_.data(), q1_.size());
        robot_kin->inverse_kinematics();
    }

    robot_kin->set_joint_pos(q1_.data(), q1_.size());
    robot_kin->set_target_end_effector_pose(6, T.data(), 16);
    robot_kin->set_LM_params(0.0001,150,2e-4,1.0);
    robot_kin->set_verbose(false);

    auto start = Clock::now();
    size_t N = 15;
    vector<long long> times;
    for (size_t i = 0; i < N; ++i)
    {   
        start = Clock::now();
        robot_kin->set_joint_pos(q1_.data(), q1_.size());
        robot_kin->inverse_kinematics();
        auto end = Clock::now();
        times.push_back(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
    }
    for (size_t i = 0; i < N; i++)
    {
        cout << "Inverse-kinematics run " << i + 1 << " duration: "
             << times[i] << " us" << endl;
    }
    robot_kin->get_joint_pos(q1_.data(), q1_.size());
    print("Reference joint configuration [rad]:", q1.transpose());
    print("Inverse-kinematics solution [rad]:", q1_.transpose());
    cout << "=== End of kinematics test ===" << endl;

    return 0;
}