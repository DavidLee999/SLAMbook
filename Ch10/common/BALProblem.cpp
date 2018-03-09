#include "BALProblem.h"
#include "tools/random.h"
#include "tools/rotation.h"

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>

typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;

template<typename T>
void FscanfOrDie(FILE* fptr, const char* format, T* value)
{
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1)
        std::cerr << "invalid UW data file.";
}

void PerturbPoint3(const double sigma, double* point)
{
    for (int i = 0; i < 3; ++i)
        point[i] += RandNormal() * sigma;
}

double Median(std::vector<double>* data)
{
    int n = data->size();
    std::vector<double>::iterator mid_point = data->begin() + n / 2;
    std::nth_element(data->begin(), mid_point, data->end());

    return *mid_point;
}

BALProblem::BALProblem(const std::string& filename, bool use_quaternions)
{
    FILE* fptr = fopen(filename.c_str(), "r");

    if (fptr == NULL)
    {
        std::cerr << "Error: unable to open file " << filename;
        return;
    }

    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);

    std::cout << "Header: " << num_cameras_ << " " << num_points_ << " " << num_observations_;

    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations_; ++i)
    {
        FscanfOrDie(fptr, "%d", camera_index_ + i);
        FscanfOrDie(fptr, "%d", point_index_ + i);
        
        for (int j = 0; j < 2; ++j)
            FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
    }

    for (int i = 0; i < num_parameters_; ++i)
        FscanfOrDie(fptr, "%lf", parameters_ + i);

    fclose(fptr);

    use_quaternions_ = use_quaternions;

    if (use_quaternions_)
    {
        num_parameters_ = 10 * num_cameras_ + 3 * num_points_;

        double* quaternion_parameters = new double[num_parameters_];
        double* original_cursor = parameters_;
        double* quaternion_cursor = quaternion_parameters;

        for (int i = 0; i < num_cameras_; ++i)
        {
            AngleAxisToQuaternion(original_cursor, quaternion_cursor);
            quaternion_cursor += 4;
            original_cursor += 3;

            for (int j = 4; j < 10; ++j)
                *quaternion_cursor++ = *original_cursor++;
        }

        delete[] parameters_;

        parameters_ = quaternion_parameters;
    }
}
