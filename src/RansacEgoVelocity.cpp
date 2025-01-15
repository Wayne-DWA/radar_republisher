#include <Eigen/Dense>
#include <chrono>
#include <random>
#include <thread>
#include <atomic>
#include <iostream>
#include <random>
#include <algorithm>
#include <stdexcept>
#include "RansacEgoVelocity.h"

RansacEgoVelocity::RansacEgoVelocity(int max_iterations, double tolerance)
    : max_iterations_(max_iterations), tolerance_(tolerance) {}

RANSACResult RansacEgoVelocity::calculate(const std::vector<RadarPoint>& radar_points) {
    size_t num_points = radar_points.size();
    if (num_points < 3) {
        throw std::runtime_error("Insufficient points for RANSAC. At least 3 points are required.");
    }

    using Clock = std::chrono::high_resolution_clock;
    // auto ransac_start = Clock::now();

    // Random generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, num_points - 1);

    // Atomic variables for parallelization
    std::atomic<int> best_inlier_count(0);
    Eigen::Vector3d best_ego_velocity = Eigen::Vector3d::Zero();

    // RANSAC iterations
    const int max_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;

    for (int t = 0; t < max_threads; ++t) {
        threads.emplace_back([&, t]() {
            for (int iter = t; iter < max_iterations_; iter += max_threads) {
                // Randomly select 3 points
                Eigen::MatrixXd A(3, 3);
                Eigen::VectorXd b(3);
                for (int i = 0; i < 3; ++i) {
                    int idx = dis(gen);
                    A.row(i) = radar_points[idx].normal_vector;
                    b(i) = radar_points[idx].doppler_velocity;
                }

                // Solve for ego velocity using efficient solver
                // Eigen::Vector3d ego_velocity = A.colPivHouseholderQr().solve(b);
                Eigen::Vector3d ego_velocity = (A.transpose() * A).ldlt().solve(A.transpose() * b);

                // Count inliers using vectorized operations
                Eigen::VectorXd residuals(num_points);
                for (size_t i = 0; i < num_points; ++i) {
                    residuals(i) = std::abs(radar_points[i].normal_vector.dot(ego_velocity) - radar_points[i].doppler_velocity);
                }

                int inlier_count = (residuals.array() < tolerance_).count();

                // Atomically update the best model
                if (inlier_count > best_inlier_count) {
                    best_inlier_count = inlier_count;
                    best_ego_velocity = ego_velocity;
                }
            }
        });
    }

    // Join threads
    for (auto& thread : threads) {
        thread.join();
    }
    // Separate inliers and outliers based on the best ego velocity
    std::vector<RadarPoint> inliers;
    std::vector<RadarPoint> outliers;
    for (const auto& point : radar_points) {
        double residual = std::abs(point.normal_vector.dot(best_ego_velocity) - point.doppler_velocity);
        if (residual < tolerance_) {
            inliers.push_back(point);
        } else {
            outliers.push_back(point);
        }
    }
    // auto ransac_end = Clock::now();
    // double ransac_time = std::chrono::duration<double, std::milli>(ransac_end - ransac_start).count();
    // std::cout << "Optimized RANSAC Time: " << ransac_time << " ms" << std::endl;

    // Final refinement using all inliers
    size_t num_inliers = inliers.size();
    Eigen::MatrixXd A_inliers(num_inliers, 3);
    Eigen::VectorXd b_inliers(num_inliers);
    int inlier_idx = 0;

    for (size_t i = 0; i < num_inliers; ++i) {
        A_inliers.row(i) = inliers[i].normal_vector;
        b_inliers(i) = inliers[i].doppler_velocity;
    }


    // auto lsq_start = Clock::now();
    // Eigen::Vector3d refined_ego_velocity = A_inliers.colPivHouseholderQr().solve(b_inliers);
    Eigen::Vector3d refined_ego_velocity = (A_inliers.transpose() * A_inliers).ldlt().solve(A_inliers.transpose() * b_inliers);

    // auto lsq_end = Clock::now();
    // double lsq_time = std::chrono::duration<double, std::milli>(lsq_end - lsq_start).count();

    // std::cout << "Optimized LSQ Time: " << lsq_time << " ms" << std::endl;

    // Covariance computation
    Eigen::VectorXd residuals = b_inliers - A_inliers * refined_ego_velocity;
    double residual_variance = residuals.squaredNorm() / (best_inlier_count - 3);
    Eigen::Matrix3d covariance = residual_variance * (A_inliers.transpose() * A_inliers).inverse();

    return {refined_ego_velocity, covariance, inliers, outliers};
}
