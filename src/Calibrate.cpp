#include "Calibrate.h"
#include <cassert>


static int uniform_random(int min_val, int max_val) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(min_val, max_val);

    return dis(gen);
}

static void generate_sequence(int count, std::vector<int> &out) {
    out.resize(count);
    for(int i=0; i<count; i++)
        out[i] = i;

    // shuffle
    for(int i=0; i<count*2; i++) {
        int l, r;
        do {
            l = uniform_random(0, count-1);
            r = uniform_random(0, count-1);
        } while(l==r);

        std::swap(out[l], out[r]);
    }
}

void BundleAdjustment(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, std::vector<cv::Point3d> &structure, cv::Mat &intrinsic, cv::Mat &extrinsicl, cv::Mat &extrinsicr, int num_matches)
{
    assert(pts1.size() == pts2.size() && pts2.size()==structure.size());

    ceres::Problem problem;

    problem.AddParameterBlock(extrinsicl.ptr<double>(), 6);
    problem.SetParameterBlockConstant(extrinsicl.ptr<double>());
    problem.AddParameterBlock(intrinsic.ptr<double>(), 9);

    ceres::LossFunction *loss_function = new ceres::HuberLoss(4);

    if(num_matches == -1)
        num_matches = pts1.size();

    std::vector<int> seq;
    generate_sequence(num_matches, seq);

    for(int i=0; i<num_matches; i++) {
        int idx = seq[i];
        
        ceres::CostFunction* costl = new ceres::AutoDiffCostFunction<ReprojectCost, 2, 9, 6, 3>(new ReprojectCost(pts1[idx]));
        ceres::CostFunction* costr = new ceres::AutoDiffCostFunction<ReprojectCost, 2, 9, 6, 3>(new ReprojectCost(pts2[idx]));

        problem.AddResidualBlock(
            costl,
            loss_function,
            intrinsic.ptr<double>(),            // Intrinsic
            extrinsicl.ptr<double>(),  // View Rotation and Translation
            &(structure[idx].x)          // Point in 3D space
        );

        problem.AddResidualBlock(
            costr,
            loss_function,
            intrinsic.ptr<double>(),            // Intrinsic
            extrinsicr.ptr<double>(),  // View Rotation and Translation
            &(structure[idx].x)          // Point in 3D space
        );

    }

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable())
    {
        std::cout << "Bundle Adjustment failed." << std::endl;
    }
    else
    {
        // Display statistics about the minimization
        std::cout << std::endl
            << "Bundle Adjustment statistics (approximated RMSE):\n"
            << " #views: 2\n"
            << " #residuals: " << summary.num_residuals << "\n"
            << " Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
            << " Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
            << " Time (s): " << summary.total_time_in_seconds << "\n"
            << std::endl;
    }

}

OneCameraBAProblem::OneCameraBAProblem(int num_camera, int structure_size, cv::Mat intrinsic) 
    : intrinsic(intrinsic)
{
    problem.AddParameterBlock(intrinsic.ptr<double>(), 9);
    structure.reserve(structure_size);
    for(int i=0; i<num_camera; i++)
        addView();
}

void OneCameraBAProblem::lockView(int index)
{
    problem.SetParameterBlockConstant(extrinsic[index].ptr<double>());
}

void OneCameraBAProblem::lockIntrinsic()
{
    problem.SetParameterBlockConstant(intrinsic.ptr<double>());
}

void OneCameraBAProblem::addView()
{
    if(extrinsic.size() >= 1)
        extrinsic.push_back(extrinsic.back().clone());
    else
        extrinsic.push_back((cv::Mat_<double>(1, 6, CV_64FC1)<<0,0,0,0,0,0));
    problem.AddParameterBlock(extrinsic.back().ptr<double>(), 6);
}

void OneCameraBAProblem::addCorrespondPoints(const std::vector<int> &cam_indices, std::vector<std::vector<cv::Point2d>> &ptss, int num_matches)
{
    assert(ptss.size() == cam_indices.size());

    if(num_matches == -1)
        num_matches = ptss[0].size();

    std::vector<int> seq;
    generate_sequence(num_matches, seq);

    ceres::LossFunction *loss_function = new ceres::HuberLoss(4);
    for(int i=0; i<num_matches; i++) {
        structure.push_back(cv::Point3d{0, 0, 1000});
        for(int j=0; j<cam_indices.size(); j++) {
            ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<ReprojectCost, 2, 9, 6, 3>(new ReprojectCost(ptss[j][seq[i]]));

            problem.AddResidualBlock(
                cost,
                loss_function,
                intrinsic.ptr<double>(),            // Intrinsic
                extrinsic[cam_indices[j]].ptr<double>(),  // View Rotation and Translation
                &(structure.back().x)          // Point in 3D space
            );
        }
    }
}

void OneCameraBAProblem::solve()
{
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable())
    {
        std::cout << "Bundle Adjustment failed." << std::endl;
    }
    else
    {
        // Display statistics about the minimization
        std::cout << std::endl
            << "Bundle Adjustment statistics (approximated RMSE):\n"
            << " #views: 2\n"
            << " #residuals: " << summary.num_residuals << "\n"
            << " Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
            << " Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
            << " Time (s): " << summary.total_time_in_seconds << "\n"
            << std::endl;
    }
}