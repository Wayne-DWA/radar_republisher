#ifndef RANSAC_EGO_VELOCITY_H
#define RANSAC_EGO_VELOCITY_H

#include <vector>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct RadarPoint {
    Eigen::Vector3d normal_vector; // Normalized vector [nx, ny, nz]
    Eigen::Vector3d position;      // Position [x, y, z]
    double doppler_velocity;      // Doppler velocity
};

struct RANSACResult {
    Eigen::Vector3d ego_velocity;       // Estimated ego velocity
    Eigen::Matrix3d covariance;        // Covariance matrix of the ego velocity
    std::vector<RadarPoint> inliers;   // Points classified as inliers
    std::vector<RadarPoint> outliers;  // Points classified as outliers
};

class RansacEgoVelocity {
public:
    RansacEgoVelocity(int max_iterations, double tolerance);
    RANSACResult calculate(const std::vector<RadarPoint>& radar_points);

private:
    int max_iterations_;
    double tolerance_;
};

#endif // RANSAC_EGO_VELOCITY_H
