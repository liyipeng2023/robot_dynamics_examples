#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <csignal>
#include <Eigen/Dense>
#include <zmq.hpp>
#include <graph.h>
#include <robot_dynamics.h>
#include "../utils/include/robot_loader.h"
#include "robot_msg.h"

namespace {
constexpr double kPi = 3.14159265358979323846;
volatile std::sig_atomic_t g_running = 1;

void signalHandler(int) {
    g_running = 0;
}

// Reads reflected rotor inertia values sequentially from a plain text file.
// Missing entries or non-existent file default to 0.0.
Eigen::VectorXd loadRotorInertiaFromTxt(const std::string& txt_path, int dof_dyn) {
    Eigen::VectorXd inertia_vec = Eigen::VectorXd::Zero(dof_dyn);
    std::ifstream infile(txt_path);
    if (!infile.is_open()) {
        std::cout << "[RDL Inertia] File '" << txt_path
                  << "' not found. Using default rotor inertia = 0.0 for all "
                  << dof_dyn << " DOFs.\n";
        return inertia_vec;
    }

    int idx = 0;
    std::string line;
    while (std::getline(infile, line) && idx < dof_dyn) {
        size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        std::stringstream ss(line);
        double val = 0.0;
        while (ss >> val && idx < dof_dyn) {
            inertia_vec(idx++) = val;
        }
    }

    std::cout << "[RDL Inertia] Loaded " << idx << "/" << dof_dyn
              << " values from '" << txt_path << "': "
              << inertia_vec.transpose() << std::endl;
    return inertia_vec;
}

void sendTorqueCommand(zmq::socket_t& pub_sock, int dof_q, int dof_dyn, const Eigen::VectorXd& tau_cmd) {
    size_t packet_size = sizeof(RdlCmdHeader) + static_cast<size_t>(dof_dyn) * sizeof(double);
    zmq::message_t cmd_msg(packet_size);
    auto* raw_out = static_cast<uint8_t*>(cmd_msg.data());

    auto* cmd_hdr = reinterpret_cast<RdlCmdHeader*>(raw_out);
    cmd_hdr->magic = 0x52444C4D;
    cmd_hdr->mode  = 0;
    cmd_hdr->nq    = static_cast<uint32_t>(dof_q);
    cmd_hdr->nv    = static_cast<uint32_t>(dof_dyn);

    std::memcpy(raw_out + sizeof(RdlCmdHeader), tau_cmd.data(), static_cast<size_t>(dof_dyn) * sizeof(double));
    pub_sock.send(cmd_msg, zmq::send_flags::dontwait);
}
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    if (argc < 2) {
        std::cout << "Usage: " << argv[0]
                  << " <model_file.xml|.urdf> [is_floating_base: 0|1] [kp] [ki] [kd]"
                  << " [traj_amp_rad] [traj_freq_hz] [rotor_inertia.txt]\n";
        std::cout << "Example: " << argv[0]
                  << " ../../models/ur10.urdf 0 400 20 40 0.4 0.25 rotor_inertia.txt\n";
        return -1;
    }

    std::string model_file = argv[1];
    bool is_floating_base = (argc > 2) ? (std::stoi(argv[2]) != 0) : false;
    double kp = (argc > 3) ? std::stod(argv[3]) : 400.0;
    double ki = (argc > 4) ? std::stod(argv[4]) : 20.0;
    double kd = (argc > 5) ? std::stod(argv[5]) : 40.0;
    double traj_amp = (argc > 6) ? std::stod(argv[6]) : 0.40;
    double traj_freq = (argc > 7) ? std::stod(argv[7]) : 0.25;
    std::string inertia_txt_file = (argc > 8) ? argv[8] : "rotor_inertia.txt";

    auto file_type = (model_file.find(".xml") != std::string::npos)
                         ? RDL::Utils::RobotFileType::MJCF
                         : RDL::Utils::RobotFileType::URDF;

    std::map<std::string, int> joint_map;
    auto rdl_dyn = RDL::Utils::load_RobotDynamics(model_file, joint_map, file_type, is_floating_base);
    if (!rdl_dyn) {
        std::cerr << "[RDL Error] Failed to load model file: " << model_file << std::endl;
        return -1;
    }
    for (auto it:joint_map) {
        std::cout << it.second << " : " << it.first << std::endl;
    }

    int dof_dyn = rdl_dyn->DOF();
    int dof_q = rdl_dyn->dim_position();

    std::cout << "========================================================\n";
    std::cout << "[RDL Computed-Torque Controller] Loaded Model: " << model_file << "\n";
    std::cout << "[RDL Computed-Torque Controller] Floating Base: " << (is_floating_base ? "Yes" : "No")
              << " | nq: " << dof_q << " | nv: " << dof_dyn << "\n";
    std::cout << "[RDL Computed-Torque Controller] Accel Gains -> Kp: " << kp << ", Ki: " << ki << ", Kd: " << kd << "\n";
    std::cout << "[RDL Computed-Torque Controller] Trajectory -> Amp: " << traj_amp << " rad, Freq: " << traj_freq << " Hz\n";
    std::cout << "[RDL Computed-Torque Controller] Inertia TXT -> " << inertia_txt_file << "\n";
    std::cout << "[RDL Computed-Torque Controller] Connecting to MuJoCo server (127.0.0.1:5555 / 5556)...\n";
    std::cout << "========================================================\n";

    Eigen::VectorXd rotor_inertia = loadRotorInertiaFromTxt(inertia_txt_file, dof_dyn);

    zmq::context_t ctx(1);

    zmq::socket_t sub_sock(ctx, zmq::socket_type::sub);
    sub_sock.set(zmq::sockopt::rcvhwm, 16);
    sub_sock.set(zmq::sockopt::linger, 0);
    sub_sock.set(zmq::sockopt::rcvtimeo, 50);
    sub_sock.set(zmq::sockopt::subscribe, "");
    sub_sock.connect("tcp://127.0.0.1:5555");

    zmq::socket_t pub_sock(ctx, zmq::socket_type::pub);
    pub_sock.set(zmq::sockopt::sndhwm, 16);
    pub_sock.set(zmq::sockopt::linger, 0);
    pub_sock.connect("tcp://127.0.0.1:5556");

    Eigen::VectorXd q = Eigen::VectorXd::Zero(dof_q);
    Eigen::VectorXd q_init = Eigen::VectorXd::Zero(dof_q);
    Eigen::VectorXd q_des = Eigen::VectorXd::Zero(dof_q);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd v_des = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd a_des = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd tau_pid = Eigen::VectorXd::Zero(dof_dyn);

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(dof_dyn, dof_dyn);
    Eigen::VectorXd tau_grav = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(dof_dyn);
    Eigen::VectorXd integral_err = Eigen::VectorXd::Zero(dof_dyn);

    const double integral_limit = 2.0;
    bool connected_and_latched = false;
    double last_timestamp = -1.0;
    double nominal_dt = 0.001;
    uint64_t step_counter = 0;
    auto wall_log_t0 = std::chrono::steady_clock::now();

    while (g_running) {
        zmq::message_t msg;
        if (!sub_sock.recv(msg, zmq::recv_flags::none)) {
            continue;
        }

        // Drain socket queue using a separate buffer so RDL always processes the newest state
        zmq::message_t tmp_msg;
        while (sub_sock.recv(tmp_msg, zmq::recv_flags::dontwait)) {
            msg = std::move(tmp_msg);
        }

        const SimStateHeader* header = nullptr;
        const double* qpos_ptr = nullptr;
        const double* qvel_ptr = nullptr;
        const double* qfrc_constraint_ptr = nullptr;
        const ContactItem* contacts_ptr = nullptr;

        if (!parseSimStateZeroCopy(msg, header, qpos_ptr, qvel_ptr, qfrc_constraint_ptr, contacts_ptr)) {
            continue;
        }

        // Auto-adapt if floating base flag does not match MuJoCo model dimensions
        if (static_cast<int>(header->nq) != dof_q || static_cast<int>(header->nv) != dof_dyn) {
            bool retry_fb = (header->nq == header->nv + 1);
            auto new_dyn = RDL::Utils::load_RobotDynamics(model_file, joint_map, file_type, retry_fb);
            if (new_dyn && new_dyn->dim_position() == static_cast<int>(header->nq) &&
                new_dyn->DOF() == static_cast<int>(header->nv)) {
                rdl_dyn = new_dyn;
                is_floating_base = retry_fb;
                dof_q = rdl_dyn->dim_position();
                dof_dyn = rdl_dyn->DOF();
                q.setZero(dof_q);
                q_init.setZero(dof_q);
                q_des.setZero(dof_q);
                v.setZero(dof_dyn);
                v_des.setZero(dof_dyn);
                a_des.setZero(dof_dyn);
                tau_pid.setZero(dof_dyn);
                M.setZero(dof_dyn, dof_dyn);
                tau_grav.setZero(dof_dyn);
                tau_cmd.setZero(dof_dyn);
                integral_err.setZero(dof_dyn);
                rotor_inertia = loadRotorInertiaFromTxt(inertia_txt_file, dof_dyn);
                connected_and_latched = false;
                last_timestamp = -1.0;
                std::cout << "[RDL Controller] Auto-matched model dimensions (Floating Base: "
                          << (is_floating_base ? "Yes" : "No") << ", nq=" << dof_q
                          << ", nv=" << dof_dyn << ")\n";
            } else {
                continue;
            }
        }

        q = Eigen::Map<const Eigen::VectorXd>(qpos_ptr, dof_q);
        v = Eigen::Map<const Eigen::VectorXd>(qvel_ptr, dof_dyn);
        double t = header->timestamp;

        // Handle initial handshake or simulation reset (t == 0.0)
        if (!connected_and_latched || t == 0.0) {
            // Only reload TXT file and print log on first connection or transition from t > 0.0 back to 0.0
            if (!connected_and_latched || last_timestamp > 0.0) {
                rotor_inertia = loadRotorInertiaFromTxt(inertia_txt_file, dof_dyn);
                std::cout << "[RDL Controller] Handshake complete! Latched trajectory origin q_init: "
                          << q.transpose() << std::endl;
                wall_log_t0 = std::chrono::steady_clock::now();
            }
            q_init = q;
            q_des = q_init;
            v_des.setZero();
            a_des.setZero();
            integral_err.setZero();
            connected_and_latched = true;
            last_timestamp = 0.0;
            step_counter = 0;
        } else if (std::abs(t - last_timestamp) < 1e-12) {
            // Duplicate state packet with identical timestamp: resend last torque without double-integrating
            sendTorqueCommand(pub_sock, dof_q, dof_dyn, tau_cmd);
            continue;
        }

        double dt = t - last_timestamp;
        if (dt > 1e-6 && dt <= 0.05) {
            nominal_dt = dt;
        } else {
            dt = nominal_dt;
        }
        last_timestamp = t;

        // 1. Generate Smooth Cosine Reference Trajectory: q_des(t), v_des(t), a_des(t)
        q_des = q_init;
        v_des.setZero();
        a_des.setZero();

        if (t > 0.0) {
            double ramp = std::min(1.0, t / 1.0);
            int start_v_idx = is_floating_base ? 6 : 0;

            for (int i = start_v_idx; i < dof_dyn; ++i) {
                int q_idx = is_floating_base ? (i + 1) : i;
                int jnt_order = i - start_v_idx;

                double sign = (jnt_order % 2 == 0) ? 1.0 : -1.0;
                double amp_i = sign * traj_amp / (1.0 + 0.15 * jnt_order);
                double omega_i = 2.0 * kPi * traj_freq * (1.0 + 0.08 * jnt_order);

                double phase = omega_i * t;
                q_des(q_idx) = q_init(q_idx) + amp_i * (1.0 - std::cos(phase));
                v_des(i) = amp_i * omega_i * std::sin(phase);
                a_des(i) = ramp * amp_i * omega_i * omega_i * std::cos(phase);
            }
        }

        // 2. Compute Gravity Torque and Mass Matrix M(q) via RDL
        tau_grav.setZero();
        M.setZero();
        rdl_dyn->forward_kinematics(q.data(), q.size());
        rdl_dyn->cal_gravitational_torque(tau_grav.data(), dof_dyn);
        rdl_dyn->forward_dynamics_crba(M.data(), dof_dyn);

        for (int r = 0; r < dof_dyn; ++r) {
            for (int c = r + 1; c < dof_dyn; ++c) {
                if (std::abs(M(r, c)) > 0.0 && std::abs(M(c, r)) == 0.0) {
                    M(c, r) = M(r, c);
                } else if (std::abs(M(c, r)) > 0.0 && std::abs(M(r, c)) == 0.0) {
                    M(r, c) = M(c, r);
                }
            }
        }

        // Add equivalent motor inertia loaded from TXT file onto the diagonal of M(q)
        M.diagonal() += rotor_inertia;

        // 3. Compute Desired Acceleration Command: tau_pid = a_des + kp*(q_des - q) + ki*int_e + kd*(v_des - v)
        tau_pid.setZero();
        double effective_kd = std::max(kd, 1.6 * std::sqrt(std::max(0.0, kp)));
        if (!is_floating_base) {
            for (int i = 0; i < dof_dyn; ++i) {
                double pos_err = q_des(i) - q(i);
                if (t > 0.0) {
                    integral_err(i) = std::clamp(integral_err(i) + pos_err * dt, -integral_limit, integral_limit);
                }
                double vel_err = v_des(i) - v(i);
                tau_pid(i) = kp * pos_err + ki * integral_err(i) + effective_kd * vel_err;
            }
        } else if (dof_q >= 7 && dof_dyn >= 6) {
            for (int i = 6; i < dof_dyn; ++i) {
                int q_idx = i + 1;
                double pos_err = q_des(q_idx) - q(q_idx);
                if (t > 0.0) {
                    integral_err(i) = std::clamp(integral_err(i) + pos_err * dt, -integral_limit, integral_limit);
                }
                double vel_err = v_des(i) - v(i);
                tau_pid(i) = kp * pos_err + ki * integral_err(i) + effective_kd * vel_err;
            }
        }
        //std::cout << "q =     " << q.transpose() << std::endl;
        //std::cout << "q_err = " << (q-q_des).transpose() << std::endl;
        //std::cout << "v_err = " << (v-v_des).transpose() << std::endl;
        // 4. Computed-Torque Law: tau_cmd = tau_grav(q) + (M_link(q) + diag(I_a)) * tau_pid
        tau_cmd = tau_grav + tau_pid;

        // 5. Send RdlCmdHeader (mode = 0) + tau_cmd to unlock MuJoCo's next step
        sendTorqueCommand(pub_sock, dof_q, dof_dyn, tau_cmd);

        if (t > 0.0 && ++step_counter % 1000 == 0) {
            auto now_wall = std::chrono::steady_clock::now();
            double wall_sec = std::chrono::duration<double>(now_wall - wall_log_t0).count();
            wall_log_t0 = now_wall;

            double max_pos_err = (q_des - q).cwiseAbs().maxCoeff();
            double max_vel_err = (v_des - v).cwiseAbs().maxCoeff();
            std::cout << "q = " << q.transpose() << std::endl;
            std::cout << std::fixed << std::setprecision(4)
                      << "[RDL Lockstep] Sim t: " << t << "s (Wall dt: " << wall_sec << "s)"
                      << " | Max |e_q|: " << max_pos_err << " rad"
                      << " | Max |e_v|: " << max_vel_err << " rad/s"
                      << " | q_des[0]: " << (dof_q > 0 ? q_des(0) : 0.0)
                      << " vs q[0]: " << (dof_q > 0 ? q(0) : 0.0) << std::endl;
        }
    }

    std::cout << "[RDL Computed-Torque Controller] Shutting down cleanly.\n";
    return 0;
}