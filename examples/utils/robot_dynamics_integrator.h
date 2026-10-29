#ifndef ROBOT_DYNAMICS_INTEGRATOR_H
#define ROBOT_DYNAMICS_INTEGRATOR_H

#include <iostream>
#include <vector>
#include <cstring> // 引入 std::memcpy
#include <definitions.h>
#include <robot_dynamics.h>
#include <constrained_dynamics_solver.h>

namespace RDL_UTILS {

    enum class IntegrationMethod {
        EULER = 0,
        SEMI_IMPLICIT_EULER = 1,
        RUNGE_KUTTA_4 = 2
    };

    class RobotDynamicsIntegrator {
    public:
        RobotDynamicsIntegrator(double step_ = 0.001, IntegrationMethod method_ = IntegrationMethod::RUNGE_KUTTA_4) 
            : step(step_), method(method_) {}

        void integrate(std::vector<RDL::RobotDynamics*> robot_dyn, std::vector<RDL::RobotData*> data) 
        {
            

            if (first_step) 
            {
                if (method == IntegrationMethod::RUNGE_KUTTA_4)
                {
                    size_t n_robots = robot_dyn.size();
                    q0.resize(n_robots);
                    qd0.resize(n_robots);
                    k1_qd.resize(n_robots); k2_qd.resize(n_robots); k3_qd.resize(n_robots); k4_qd.resize(n_robots);
                    k1_q.resize(n_robots);  k2_q.resize(n_robots);  k3_q.resize(n_robots);  k4_q.resize(n_robots);
                    k_q.resize(n_robots);

                    for (size_t i = 0; i < n_robots; ++i) 
                    {
                        size_t pos_dim = robot_dyn[i]->dim_position();
                        size_t dof = robot_dyn[i]->DOF();

                        q0[i].resize(pos_dim);
                        qd0[i].resize(dof);
                        k1_qd[i].resize(dof); k2_qd[i].resize(dof); k3_qd[i].resize(dof); k4_qd[i].resize(dof);
                        k1_q[i].resize(dof);  k2_q[i].resize(dof);  k3_q[i].resize(dof);  k4_q[i].resize(dof);
                        k_q[i].resize(dof);
                    }
                }
                first_step = false;
            }

            switch (method) {
                case IntegrationMethod::EULER:
                {
                    for (size_t i = 0; i < robot_dyn.size(); ++i)
                    {
                        size_t pos_dim = robot_dyn[i]->dim_position();
                        size_t dof = robot_dyn[i]->DOF();

                        robot_dyn[i]->forward_kinematics(data[i]->q, pos_dim);
                        robot_dyn[i]->forward_dynamics_aba(data[i]->qd, data[i]->tau, data[i]->qdd, dof,false);

                        robot_dyn[i]->update_position(data[i]->qd, dof, data[i]->q, data[i]->q, pos_dim, step);

                        for (size_t j = 0; j < dof; ++j) {
                            data[i]->qd[j] += data[i]->qdd[j] * step;
                        }
                    }
                    break;
                }
                case IntegrationMethod::SEMI_IMPLICIT_EULER:
                {
                    for (size_t i = 0; i < robot_dyn.size(); ++i)
                    {
                        size_t pos_dim = robot_dyn[i]->dim_position();
                        size_t dof = robot_dyn[i]->DOF();

                        robot_dyn[i]->forward_kinematics(data[i]->q, pos_dim);
                        robot_dyn[i]->forward_dynamics_aba(data[i]->qd, data[i]->tau, data[i]->qdd, dof,false);

                        for (size_t j = 0; j < dof; ++j) {
                            data[i]->qd[j] += data[i]->qdd[j] * step;
                        }

                        robot_dyn[i]->update_position(data[i]->qd, dof, data[i]->q, data[i]->q, pos_dim, step);
                    }
                    break;
                }
                case IntegrationMethod::RUNGE_KUTTA_4:
                {
                    for (size_t i=0; i<robot_dyn.size(); i++)
                    {
                        size_t pos_dim = robot_dyn[i]->dim_position();
                        size_t dof = robot_dyn[i]->DOF();
                        // --- Step 1 ---
                        std::memcpy(q0[i].data(), data[i]->q, pos_dim * sizeof(double));
                        std::memcpy(qd0[i].data(), data[i]->qd, dof * sizeof(double));

                        robot_dyn[i]->forward_kinematics(q0[i].data(), q0[i].size());
                        robot_dyn[i]->forward_dynamics_aba(qd0[i].data(), data[i]->tau, data[i]->qdd, dof,false);

                        std::memcpy(k1_q[i].data(), qd0[i].data(), dof * sizeof(double));
                        std::memcpy(k1_qd[i].data(), data[i]->qdd, dof * sizeof(double));
                        
                        robot_dyn[i]->update_position(k1_q[i].data(), k1_q[i].size(), data[i]->q, q0[i].data(), q0[i].size(), 0.5 * step);
                        for (size_t j = 0; j < dof; ++j) {
                            qd0[i][j] = data[i]->qd[j] + 0.5 * step * k1_qd[i][j];
                        }

                        // --- Step 2 ---
                        robot_dyn[i]->forward_kinematics(q0[i].data(), q0[i].size());
                        robot_dyn[i]->forward_dynamics_aba(qd0[i].data(), data[i]->tau, data[i]->qdd, dof, false);
                        
                        std::memcpy(k2_q[i].data(), qd0[i].data(), dof * sizeof(double));
                        std::memcpy(k2_qd[i].data(), data[i]->qdd, dof * sizeof(double));

                        robot_dyn[i]->update_position(k2_q[i].data(), k2_q[i].size(), data[i]->q, q0[i].data(), q0[i].size(), 0.5 * step);
                        for (size_t j = 0; j < dof; ++j) {
                            qd0[i][j] = data[i]->qd[j] + 0.5 * step * k2_qd[i][j];
                        }

                        // --- Step 3 ---
                        robot_dyn[i]->forward_kinematics(q0[i].data(), q0[i].size());
                        robot_dyn[i]->forward_dynamics_aba(qd0[i].data(), data[i]->tau, data[i]->qdd, dof, false);

                        std::memcpy(k3_q[i].data(), qd0[i].data(), dof * sizeof(double));
                        std::memcpy(k3_qd[i].data(), data[i]->qdd, dof * sizeof(double));

                        robot_dyn[i]->update_position(k3_q[i].data(), k3_q[i].size(), data[i]->q, q0[i].data(), q0[i].size(), step);
                        for (size_t j = 0; j < dof; ++j) {
                            qd0[i][j] = data[i]->qd[j] + step * k3_qd[i][j];
                        }

                        // --- Step 4 ---
                        robot_dyn[i]->forward_kinematics(q0[i].data(), q0[i].size());
                        robot_dyn[i]->forward_dynamics_aba(qd0[i].data(), data[i]->tau, data[i]->qdd, dof, false);

                        std::memcpy(k4_q[i].data(), qd0[i].data(), dof * sizeof(double));
                        std::memcpy(k4_qd[i].data(), data[i]->qdd, dof * sizeof(double));

                        // --- State Update ---
                        for (size_t j = 0; j < dof; ++j) {
                            data[i]->qd[j] += (k1_qd[i][j] + 2.0 * k2_qd[i][j] + 2.0 * k3_qd[i][j] + k4_qd[i][j]) * (step / 6.0);
                            k_q[i][j] = (k1_q[i][j] + 2.0 * k2_q[i][j] + 2.0 * k3_q[i][j] + k4_q[i][j]) * (step / 6.0);
                        }
                        
                        robot_dyn[i]->update_position(k_q[i].data(), k_q[i].size(), data[i]->q, data[i]->q, pos_dim, 1.0);
                    }
                    break;
                }
                default:
                    std::cerr << "Unknown integration method!" << std::endl;
                    break;
            }
        }

        void integrate(std::vector<RDL::RobotDynamics*> robot_dyn, std::vector<RDL::RobotData*> data, RDL::ConstrainedDynamicsSolver* solver, bool using_newton_raphson = false) 
        {
            if (first_step)
            {
                first_step = false;
                if (method == IntegrationMethod::RUNGE_KUTTA_4)
                {
                    q0.resize(robot_dyn.size());
                    qd0.resize(robot_dyn.size());
                    k1_qd.resize(robot_dyn.size()); k2_qd.resize(robot_dyn.size()); 
                    k3_qd.resize(robot_dyn.size()); k4_qd.resize(robot_dyn.size());
                    k1_q.resize(robot_dyn.size());  k2_q.resize(robot_dyn.size());  
                    k3_q.resize(robot_dyn.size());  k4_q.resize(robot_dyn.size());
                    k_q.resize(robot_dyn.size());

                    for (size_t i = 0; i < robot_dyn.size(); i++)
                    {
                        size_t pos_dim = robot_dyn[i]->dim_position();
                        size_t dof = robot_dyn[i]->DOF();

                        q0[i].resize(pos_dim);
                        qd0[i].resize(dof);
                        k1_qd[i].resize(dof); k2_qd[i].resize(dof); 
                        k3_qd[i].resize(dof); k4_qd[i].resize(dof);
                        k1_q[i].resize(dof);  k2_q[i].resize(dof);  
                        k3_q[i].resize(dof);  k4_q[i].resize(dof);
                        k_q[i].resize(dof);
                    }
                }
            }

            switch (method) {
                case IntegrationMethod::EULER:
                {
                    solver->solve(robot_dyn.data(), data.data(), robot_dyn.size());
                    for (size_t i = 0; i < robot_dyn.size(); ++i)
                    {
                        size_t pos_dim = robot_dyn[i]->dim_position();
                        size_t dof = robot_dyn[i]->DOF();

                        robot_dyn[i]->update_position(data[i]->qd, dof, data[i]->q, data[i]->q, pos_dim, step);

                        for (size_t j = 0; j < dof; ++j) {
                            data[i]->qd[j] += data[i]->qdd[j] * step;
                        }
                    }
                    if (using_newton_raphson)
                    {
                        solver->project_constraint(robot_dyn.data(), data.data(), robot_dyn.size());
                    }
                    break;
                }
                case IntegrationMethod::SEMI_IMPLICIT_EULER:
                {
                    solver->solve(robot_dyn.data(), data.data(), robot_dyn.size());
                    for (size_t i = 0; i < robot_dyn.size(); ++i)
                    {
                        size_t pos_dim = robot_dyn[i]->dim_position();
                        size_t dof = robot_dyn[i]->DOF();

                        for (size_t j = 0; j < dof; ++j) {
                            data[i]->qd[j] += data[i]->qdd[j] * step;
                        }

                        robot_dyn[i]->update_position(data[i]->qd, dof, data[i]->q, data[i]->q, pos_dim, step);
                    }
                    if (using_newton_raphson)
                    {
                        solver->project_constraint(robot_dyn.data(), data.data(), robot_dyn.size());
                    }
                    break;
                }
                case IntegrationMethod::RUNGE_KUTTA_4:
                {
                    for (size_t i = 0; i < robot_dyn.size(); i++)
                    {
                        size_t pos_dim = robot_dyn[i]->dim_position();
                        size_t dof = robot_dyn[i]->DOF();

                        std::memcpy(q0[i].data(), data[i]->q, pos_dim * sizeof(double));
                        std::memcpy(qd0[i].data(), data[i]->qd, dof * sizeof(double));
                    }

                    // Step 1
                    solver->solve(robot_dyn.data(), data.data(), robot_dyn.size());
                    for (size_t i = 0; i < robot_dyn.size(); i++)
                    {
                        size_t dof = robot_dyn[i]->DOF();
                        
                        std::memcpy(k1_qd[i].data(), data[i]->qdd, dof * sizeof(double));
                        std::memcpy(k1_q[i].data(), data[i]->qd, dof * sizeof(double));
                        
                        robot_dyn[i]->update_position(k1_q[i].data(), k1_q[i].size(), q0[i].data(), data[i]->q, robot_dyn[i]->dim_position(), 0.5 * step);
                        for (size_t j = 0; j < dof; j++)
                        {
                            data[i]->qd[j] = qd0[i][j] + 0.5 * step * k1_qd[i][j];
                        }
                    }
                    if (using_newton_raphson)
                    {
                        solver->project_constraint(robot_dyn.data(), data.data(), robot_dyn.size());
                    }

                    // Step 2
                    solver->solve(robot_dyn.data(), data.data(), robot_dyn.size());
                    for (size_t i = 0; i < robot_dyn.size(); i++)
                    {
                        size_t dof = robot_dyn[i]->DOF();
                        
                        std::memcpy(k2_qd[i].data(), data[i]->qdd, dof * sizeof(double));
                        std::memcpy(k2_q[i].data(), data[i]->qd, dof * sizeof(double));

                        robot_dyn[i]->update_position(k2_q[i].data(), k2_q[i].size(), q0[i].data(), data[i]->q, robot_dyn[i]->dim_position(), 0.5 * step);
                        for (size_t j = 0; j < dof; j++)
                        {
                            data[i]->qd[j] = qd0[i][j] + 0.5 * step * k2_qd[i][j];
                        }
                    }
                    if (using_newton_raphson)
                    {
                        solver->project_constraint(robot_dyn.data(), data.data(), robot_dyn.size());
                    }

                    // Step 3
                    solver->solve(robot_dyn.data(), data.data(), robot_dyn.size());
                    for (size_t i = 0; i < robot_dyn.size(); i++)
                    {
                        size_t dof = robot_dyn[i]->DOF();
                        
                        std::memcpy(k3_qd[i].data(), data[i]->qdd, dof * sizeof(double));
                        std::memcpy(k3_q[i].data(), data[i]->qd, dof * sizeof(double));

                        robot_dyn[i]->update_position(k3_q[i].data(), k3_q[i].size(), q0[i].data(), data[i]->q, robot_dyn[i]->dim_position(), step);
                        for (size_t j = 0; j < dof; j++)
                        {
                            data[i]->qd[j] = qd0[i][j] + step * k3_qd[i][j];
                        }
                    }
                    if (using_newton_raphson)
                    {
                        solver->project_constraint(robot_dyn.data(), data.data(), robot_dyn.size());
                    }

                    // Step 4
                    solver->solve(robot_dyn.data(), data.data(), robot_dyn.size());
                    for (size_t i = 0; i < robot_dyn.size(); i++)
                    {
                        size_t dof = robot_dyn[i]->DOF();
                        
                        std::memcpy(k4_qd[i].data(), data[i]->qdd, dof * sizeof(double));
                        std::memcpy(k4_q[i].data(), data[i]->qd, dof * sizeof(double));
                    }

                    // update
                    for (size_t i = 0; i < robot_dyn.size(); i++)
                    {
                        size_t dof = robot_dyn[i]->DOF();
                        size_t pos_dim = robot_dyn[i]->dim_position();

                        for (size_t j = 0; j < dof; ++j) {
                            k_q[i][j] = (k1_q[i][j] + 2.0 * k2_q[i][j] + 2.0 * k3_q[i][j] + k4_q[i][j]) * (step / 6.0);
                        }

                        robot_dyn[i]->update_position(k_q[i].data(), k_q[i].size(), q0[i].data(), data[i]->q, pos_dim, 1.0);
                        for (size_t j = 0; j < dof; j++)
                        {
                            data[i]->qd[j] = qd0[i][j] + (step / 6.0) * (k1_qd[i][j] + 2.0 * k2_qd[i][j] + 2.0 * k3_qd[i][j] + k4_qd[i][j]);
                        }
                    }
                    if (using_newton_raphson)
                    {
                        solver->project_constraint(robot_dyn.data(), data.data(), robot_dyn.size());
                    }

                    break;
                }
                default:
                    break;
            }
        }

    private:
        std::vector<std::vector<double>> k1_qd, k2_qd, k3_qd, k4_qd;
        std::vector<std::vector<double>> k1_q, k2_q, k3_q, k4_q, k_q;
        std::vector<std::vector<double>> q0, qd0;
        bool first_step = true;
        double step;
        IntegrationMethod method;
    };

} // namespace RDL_UTILS

#endif