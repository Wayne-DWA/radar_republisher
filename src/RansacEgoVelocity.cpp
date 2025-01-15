#include "RansacEgoVelocity.h"
#include <random>
#include <algorithm>
#include <stdexcept>

RansacEgoVelocity::RansacEgoVelocity(int max_iterations, double tolerance)
    : max_iterations_(max_iterations), tolerance_(tolerance) {}


RANSACResult RansacEgoVelocity::calculate(const std::vector<RadarPoint>& radar_points) {
    size_t num_points = radar_points.size();
    if (num_points < 3) {
        throw std::runtime_error("Insufficient points for RANSAC. At least 3 points are required.");
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, num_points - 1);

    Eigen::Vector3d best_ego_velocity;
    int best_inlier_count = 0;
    std::vector<int> best_inlier_indices;

    for (int iter = 0; iter < max_iterations_; ++iter) {
        // Randomly select 3 points
        std::vector<int> indices(3);
        for (int i = 0; i < 3; ++i) {
            indices[i] = dis(gen);
        }

        // Form matrices A and b using the 3 selected points
        Eigen::MatrixXd A(3, 3);
        Eigen::VectorXd b(3);
        for (int i = 0; i < 3; ++i) {
            A.row(i) = radar_points[indices[i]].normal_vector;
            b(i) = radar_points[indices[i]].doppler_velocity;
        }

        // Solve for ego velocity using least squares
        Eigen::Vector3d ego_velocity = (A.transpose() * A).ldlt().solve(A.transpose() * b);

        // Count inliers
        std::vector<int> inlier_indices;
        for (size_t i = 0; i < num_points; ++i) {
            double residual = std::abs(radar_points[i].normal_vector.dot(ego_velocity) - radar_points[i].doppler_velocity);
            if (residual < tolerance_) {
                inlier_indices.push_back(i);
            }
        }

        // Update the best model if more inliers are found
        if (inlier_indices.size() > best_inlier_count) {
            best_inlier_count = inlier_indices.size();
            best_inlier_indices = inlier_indices;
            best_ego_velocity = ego_velocity;
        }
    }

    // Separate inliers and outliers
    std::vector<RadarPoint> inliers;
    std::vector<RadarPoint> outliers;
    for (size_t i = 0; i < num_points; ++i) {
        if (std::find(best_inlier_indices.begin(), best_inlier_indices.end(), i) != best_inlier_indices.end()) {
            inliers.push_back(radar_points[i]);
        } else {
            outliers.push_back(radar_points[i]);
        }
    }

    // Refine the solution using all inliers
    Eigen::MatrixXd A_inliers(inliers.size(), 3);
    Eigen::VectorXd b_inliers(inliers.size());
    for (size_t i = 0; i < inliers.size(); ++i) {
        A_inliers.row(i) = inliers[i].normal_vector;
        b_inliers(i) = inliers[i].doppler_velocity;
    }
    Eigen::Vector3d refined_ego_velocity = (A_inliers.transpose() * A_inliers).ldlt().solve(A_inliers.transpose() * b_inliers);

    // Calculate residuals
    Eigen::VectorXd residuals = b_inliers - A_inliers * refined_ego_velocity;
    double residual_variance = residuals.squaredNorm() / (inliers.size() - 3);

    // Calculate covariance matrix
    Eigen::Matrix3d covariance = residual_variance * (A_inliers.transpose() * A_inliers).inverse();

    return {refined_ego_velocity, covariance, inliers, outliers};
}
