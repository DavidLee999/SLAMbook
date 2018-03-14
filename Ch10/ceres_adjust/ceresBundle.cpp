#include <iostream>
#include <fstream>

#include "ceres/ceres.h"

#include "SnavelyReprojectionError.h"
#include "common/BALProblem.h"
#include "common/BundleParams.h"


using namespace ceres;

void SetLinearSolver(ceres::Solver::Options* options, const BundleParams& params)
{
    CHECK(ceres::StringToLinearSolverType(params.linear_solver, &options->linear_solver_type));

    CHECK(ceres::StringToSparseLinearAlgebraLibraryType(params.sparse_linear_algebra_library, &options->sparse_linear_algebra_library_type));

    CHECK(ceres::StringToDenseLinearAlgebraLibraryType(params.dense_linear_algebra_library, &options->dense_linear_algebra_library_type));

    options->num_linear_solver_threads = params.num_threads;
}

void SetOrdering(BALProblem* bal_problem, ceres::Solver::Options* options, const BundleParams& params)
{
    const int num_points = bal_problem->num_points();
    const int point_block_size = bal_problem->point_block_size();
    double* points = bal_problem->mutable_points();

    const int num_cameras = bal_problem->num_cameras();
    const int camera_block_size = bal_problem->camera_block_size();
    double* cameras = bal_problem->mutable_cameras();

    if (params.ordering == "automatic")
        return;

    ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;

    for (int i = 0; i < num_points; ++i)
        ordering->AddElementToGroup(points + point_block_size * i, 0);

    for (int i = 0; i < num_cameras; ++i)
        ordering->AddElementToGroup(cameras + camera_block_size * i, 1);

    options->linear_solver_ordering.reset(ordering);
}

void SetMinimizerOptions(Solver::Options* options, const BundleParams& params)
{
    options->max_num_iterations = params.num_iterations;
    options->minimizer_progress_to_stdout = true;
    options->num_threads = params.num_threads;

    CHECK(StringToTrustRegionStrategyType(params.trust_region_strategy, &options->trust_region_strategy_type));

}

void SetSolverOptionsFromFlags(BALProblem* bal_problem, const BundleParams& params, Solver::Options* options)
{
    SetMinimizerOptions(options, params);
    SetLinearSolver(options, params);
    SetOrdering(bal_problem, options, params);
}

void BuildProblem(BALProblem* bal_problem, Problem* problem, const BundleParams& params)
{
    const int point_block_size = bal_problem->point_block_size();
    const int camera_block_size = bal_problem->camera_block_size();
    double* points = bal_problem->mutable_points();
    double* cameras = bal_problem->mutable_cameras();

    const double* observations = bal_problem->observations();

    for (int i = 0; i < bal_problem->num_observations(); ++i)
    {
        CostFunction* cost_function;

        cost_function = SnavelyReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);

        LossFunction* loss_function = params.robustify ? new HuberLoss(1.0) : NULL;

        double* camera = cameras + camera_block_size * bal_problem->camera_index()[i];
        double* point = points + point_block_size * bal_problem->point_index()[i];

        problem->AddResidualBlock(cost_function, loss_function, camera, point);
    }
}

void SolveProblem(const char* filename, const BundleParams& params)
{
    BALProblem bal_problem(filename);

    std::cout << "bal problem file loaded ...\n";
    std::cout << "bal_problem have " << bal_problem.num_cameras() << " cameras and " << bal_problem.num_points() << " points.\n";
    std::cout << "Forming " << bal_problem.num_observations() << " observations.\n";

    if (!params.initial_ply.empty())
        bal_problem.WriteToPLYFile(params.initial_ply);

    std::cout << "beginning problem ...\n";

    srand(params.random_seed);

    bal_problem.Normalize();
    bal_problem.Perturb(params.rotation_sigma, params.translation_sigma, params.point_sigma);

    std::cout << "Normalization complete ...\n";

    Problem problem;
    BuildProblem(&bal_problem, &problem, params);

    std::cout << "the problem is successfully build.\n";

    Solver::Options options;
    SetSolverOptionsFromFlags(&bal_problem, params, &options);
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;

    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";

    if (!params.final_ply.empty())
    {
        bal_problem.WriteToPLYFile(params.final_ply);
    }
}

int main(int argc, char** argv)
{
    BundleParams params(argc, argv);

    google::InitGoogleLogging(argv[0]);

    std::cout << params.input << std::endl;

    if (params.input.empty())
    {
        std::cout << "Usage: bundle_adjuster -input <path to dataset>";
        return 1;
    }

    SolveProblem(params.input.c_str(), params);

    return 0;

}
