#ifndef DATA_RECORDER_H
#define DATA_RECORDER_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <initializer_list>
#include <Eigen/Core>

namespace RDL_UTILS
{
    class DataRecorder {
    private:
        std::vector<std::string> ordered_categories;               // 
        std::vector<std::vector<Eigen::VectorXd>> data_storage;    // 

        Eigen::VectorXd toEigenVector(const Eigen::VectorXd& val) {
            return val;
        }

        Eigen::VectorXd toEigenVector(double val) {
            Eigen::VectorXd vec(1);
            vec(0) = val;
            return vec;
        }

        Eigen::VectorXd toEigenVector(const std::vector<double>& val) {
            return Eigen::Map<const Eigen::VectorXd>(val.data(), val.size());
        }

    public:
        DataRecorder() = default;
        DataRecorder(std::initializer_list<std::string> categories) {
            set_categories(categories);
        }

        void set_categories(const std::vector<std::string>& categories) {
            ordered_categories = categories;
        }


        template <typename... Args>
        void record_frame(const Args&... args) {
            if (sizeof...(args) != ordered_categories.size()) {
                std::cerr << "DateRecorder Error: the number of data (" << sizeof...(args) 
                        << ") does not match with (" << ordered_categories.size() << ")！\n";
                return;
            }

            std::vector<Eigen::VectorXd> frame_data { toEigenVector(args)... };
            
            data_storage.push_back(std::move(frame_data));
        }

        bool save(const std::string& filename, const std::string& delimiter = ",") const {
            if (data_storage.empty() || ordered_categories.empty()) return false;

            std::ofstream outfile(filename);
            if (!outfile.is_open()) return false;

            const auto& first_frame = data_storage[0];
            std::vector<std::string> header_columns;
            for (size_t cat_idx = 0; cat_idx < ordered_categories.size(); ++cat_idx) {
                const std::string& cat_name = ordered_categories[cat_idx];
                int dim = first_frame[cat_idx].size();
                if (dim == 1) {
                    header_columns.push_back(cat_name);
                } else {
                    for (int d = 0; d < dim; ++d) {
                        header_columns.push_back(cat_name + "_" + std::to_string(d));
                    }
                }
            }

            for (size_t i = 0; i < header_columns.size(); ++i) {
                outfile << header_columns[i] << (i == header_columns.size() - 1 ? "" : delimiter);
            }
            outfile << "\n";

            outfile.precision(8);
            outfile << std::fixed;
            for (const auto& frame : data_storage) {
                bool is_first = true;
                for (const auto& vec : frame) {
                    for (int i = 0; i < vec.size(); ++i) {
                        if (!is_first) outfile << delimiter;
                        outfile << vec(i);
                        is_first = false;
                    }
                }
                outfile << "\n";
            }
            std::cout << "Save data to " << filename << "\n";
            return true;
        }
    };
}

#endif