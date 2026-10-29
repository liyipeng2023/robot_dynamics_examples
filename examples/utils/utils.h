#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <robot_dynamics.h>
#include <memory>
#include <chrono>
#include <fstream>
#include <string>

const static Eigen::IOFormat MatlabFmt(6, Eigen::DontAlignCols, ", ", ";\n", "", "", "[", "]");
const static Eigen::IOFormat PythonFmt(4, Eigen::DontAlignCols, ", ", "\n", "", "", "[", "]");
const static Eigen::IOFormat CSVFmt(4, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");

void print(std::string name, Eigen::MatrixXd mat) {
    name.append(" = ");
    const size_t ident_len = name.length()+1;
    const std::string indent(ident_len, ' ');
    std::stringstream ss;
    ss << mat.format(MatlabFmt);
    std::string line;
    bool first_line = true;
    std::cout << name;
    while (std::getline(ss, line)) {
        if (first_line) {
            std::cout << line << std::endl;
            first_line = false;
        } else {
            std::cout << indent << line << std::endl;
        }
    }
}

using Clock = std::chrono::high_resolution_clock;

void save_to_csv(const std::vector<long long>& data,
                      const std::string& filename)
{
    std::ofstream file(filename);

    if (!file.is_open()) return;

    for (size_t i = 0; i < data.size(); ++i)
    {
        file << data[i];
        if (i + 1 != data.size())
            file << "\n";
    }
    file << "\n";

    file.close();
}

// column-major
void matrix3d_to_array(const Eigen::Matrix3d& mat_, double* arr_) {
    arr_[0] = mat_(0,0);
    arr_[1] = mat_(1,0);
    arr_[2] = mat_(2,0);
    arr_[3] = mat_(0,1);
    arr_[4] = mat_(1,1);
    arr_[5] = mat_(2,1);
    arr_[6] = mat_(0,2);
    arr_[7] = mat_(1,2);
    arr_[8] = mat_(2,2);
}

void vector3d_to_array(const Eigen::Vector3d& vec_, double* arr_) {
    arr_[0] = vec_(0);
    arr_[1] = vec_(1);
    arr_[2] = vec_(2);
}

Eigen::Matrix3d cross_3d(const Eigen::Vector3d& a) {
    Eigen::Matrix3d cross_mat;
    cross_mat << 0, -a(2), a(1),
                 a(2), 0, -a(0),
                 -a(1), a(0), 0;
    return cross_mat;
}