#pragma once
#include <chrono>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "radar_ego_velocity_estimator.hpp"
#include "RansacEgoVelocity.h"

using PointCloud2 = sensor_msgs::PointCloud2;
using PointCloud = sensor_msgs::PointCloud;
using PointCloud2Ptr = sensor_msgs::PointCloud2::Ptr;
using PointCloud2ConstPtr = sensor_msgs::PointCloud2::ConstPtr;
typedef pcl::PointXYZI PointT;

struct HuginPointCloudType
{
  PCL_ADD_POINT4D      // x,y,z position in [m]
  float doppler;
  float power;
  float range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 
} ; // 

POINT_CLOUD_REGISTER_POINT_STRUCT
(
    HuginPointCloudType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, doppler, doppler)
    (float, power, power)
    (float, range, range)
)

struct RadarPointCloudType
{
  PCL_ADD_POINT4D      // x,y,z position in [m]
  PCL_ADD_INTENSITY;
  union
    {
      struct
      {
        float doppler;
      };
      float data_c[4];
    };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 
} EIGEN_ALIGN16; // 
POINT_CLOUD_REGISTER_POINT_STRUCT
(
    RadarPointCloudType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, doppler, doppler)
)
namespace ars_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float doppler;
      float intensity;
      float range_std;
      float azimuth_std;    //yaw
      float elevation_std;  //pitch
      float doppler_std;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ars_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(ars_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, doppler, doppler)
    (float, intensity, intensity)
    (float, range_std, range_std)
    (float, azimuth_std, azimuth_std)
    (float, elevation_std, elevation_std)
    (float, doppler_std, doppler_std)
)
namespace ars_ros_simple {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float doppler;
      float intensity;
      float doppler_std;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ars_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(ars_ros_simple::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, doppler, doppler)
    (float, intensity, intensity)
    (float, doppler_std, doppler_std)
)
namespace eagle_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float Doppler;
      float Power;
      float Range;
      float Alpha;  
      float Beta;  
      float rangeAccu;
      float aziAccu;
      float eleAccu;
      float dopplerAccu;
      float recoveredSpeed;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace eagle_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(eagle_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, Doppler, Doppler)
    (float, Power, Power)
    (float, Range, Range)
    (float, Alpha, Alpha)
    (float, Beta, Beta)
    (float, rangeAccu, rangeAccu)
    (float, aziAccu, aziAccu)
    (float, eleAccu, eleAccu)
    (float, dopplerAccu, dopplerAccu)
    (float, recoveredSpeed, recoveredSpeed)
)
namespace altos {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float h; // doppler velocity of point
      float s; // RCS of point
      float v; //direction of point (-1:opposite 0:static 1:same)
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace altos

POINT_CLOUD_REGISTER_POINT_STRUCT(altos::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, h, h)
    (float, s, s)
    (float, v, v)
)
namespace hercules {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float v; // doppler velocity of point
      float r; //range
      int8_t RCS; // RCS of point
      float azimuth; //yaw
      float elevation; //pitch
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace altos

POINT_CLOUD_REGISTER_POINT_STRUCT(hercules::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, v, v)
    (float, r, r)
    (int8_t, RCS, RCS)
    (float, azimuth, azimuth)
    (float, elevation, elevation)
)
class RadarMsgConverter {
public:
  RadarMsgConverter() {
    pc2_raw_msg.reset(new PointCloud2);
    reve_en = true;
  }

  PointCloud2ConstPtr convert(const PointCloud2& radar_msg) {
    //********** Convert HuginPointCloud to RadarPointCloud **********
    pcl::PointCloud<HuginPointCloudType>::Ptr hugin_raw( new pcl::PointCloud<HuginPointCloudType> );
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType> );
    pcl::fromROSMsg(radar_msg, *hugin_raw);
    RadarPointCloudType radarpoint_raw;
    for (size_t i = 0; i < hugin_raw->size(); ++i)
    {
      //! Hugin has a different coordinate system compared to Radar
      radarpoint_raw.x = hugin_raw->at(i).y;
      radarpoint_raw.y = -hugin_raw->at(i).x;
      radarpoint_raw.z = hugin_raw->at(i).z;
      radarpoint_raw.intensity = hugin_raw->at(i).power;
      radarpoint_raw.doppler = hugin_raw->at(i).doppler;
      radarcloud_raw->points.push_back(radarpoint_raw);
    }
    pcl::toROSMsg(*radarcloud_raw, *pc2_raw_msg);
    pc2_raw_msg->header = radar_msg.header;
    //! some radar data has a different frame_id
    pc2_raw_msg->header.frame_id = "hugin_radar";
    return pc2_raw_msg;
  }
  PointCloud2ConstPtr convert(const PointCloud& radar_msg) {
    //********** Convert sensor_msgs::PointCloud to RadarPointCloud **********
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType> );
    for (size_t i = 0; i < radar_msg.points.size(); ++i)
    {
        if (radar_msg.points[i].x == NAN || radar_msg.points[i].y == NAN || radar_msg.points[i].z == NAN) continue;
        if (radar_msg.points[i].x == INFINITY || radar_msg.points[i].y == INFINITY || radar_msg.points[i].z == INFINITY) continue;

        RadarPointCloudType radarpoint_raw;

        radarpoint_raw.x = radar_msg.points[i].x;
        radarpoint_raw.y = radar_msg.points[i].y;
        radarpoint_raw.z = radar_msg.points[i].z;
        radarpoint_raw.intensity = radar_msg.channels[2].values[i];
        radarpoint_raw.doppler = radar_msg.channels[0].values[i];
        radarcloud_raw->points.push_back(radarpoint_raw);
    }
    pcl::toROSMsg(*radarcloud_raw, *pc2_raw_msg);
    pc2_raw_msg->header = radar_msg.header;
    return pc2_raw_msg;
  }
  PointCloud2ConstPtr convert_ars(const PointCloud2& ars_msg){
    //********** Convert ars_ros_msg to RadarPointCloud **********
    pcl::PointCloud<ars_ros::Point> ars_raw;
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType> );
    pcl::fromROSMsg(ars_msg, ars_raw);
    int point_size = ars_raw.points.size();
    radarcloud_raw->reserve(point_size);
    RadarPointCloudType radarpoint_raw;
    for (size_t i = 0; i < point_size; ++i)
    {
        radarpoint_raw.x = ars_raw.points[i].x;
        radarpoint_raw.y = ars_raw.points[i].y;
        radarpoint_raw.z = ars_raw.points[i].z;
        radarpoint_raw.intensity = ars_raw.points[i].intensity;
        radarpoint_raw.doppler = ars_raw.points[i].doppler;
        radarcloud_raw->points.push_back(radarpoint_raw);
    }
    pcl::toROSMsg(*radarcloud_raw, *pc2_raw_msg);
    pc2_raw_msg->header = ars_msg.header;
    return pc2_raw_msg;
  }
  PointCloud2ConstPtr convert_simple_ars(const PointCloud2& ars_simple_msg){
    //********** Convert ars_ros_msg to RadarPointCloud **********
    pcl::PointCloud<ars_ros_simple::Point> ars_raw;
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType> );
    pcl::fromROSMsg(ars_simple_msg, ars_raw);
    int point_size = ars_raw.points.size();
    radarcloud_raw->reserve(point_size);
    RadarPointCloudType radarpoint_raw;
    for (size_t i = 0; i < point_size; ++i)
    {
        radarpoint_raw.x = ars_raw.points[i].x;
        radarpoint_raw.y = ars_raw.points[i].y;
        radarpoint_raw.z = ars_raw.points[i].z;
        radarpoint_raw.intensity = std::abs(ars_raw.points[i].intensity);
        radarpoint_raw.doppler = ars_raw.points[i].doppler;
        radarcloud_raw->points.push_back(radarpoint_raw);
    }
    pcl::toROSMsg(*radarcloud_raw, *pc2_raw_msg);
    pc2_raw_msg->header = ars_simple_msg.header;
    //! some radar data has a different frame_id
    pc2_raw_msg->header.frame_id = "base_link";
    return pc2_raw_msg;
  }
  PointCloud2ConstPtr convert_eagle(const PointCloud2& eagle_msg){
    //********** Convert eagle_msg to RadarPointCloud **********
    pcl::PointCloud<eagle_ros::Point> eagle_raw;
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType> );
    pcl::fromROSMsg(eagle_msg, eagle_raw);
    int point_size = eagle_raw.points.size();
    radarcloud_raw->reserve(point_size);
    RadarPointCloudType radarpoint_raw;
    for (size_t i = 0; i < point_size; ++i)
    {
        radarpoint_raw.x = eagle_raw.points[i].x;
        radarpoint_raw.y = eagle_raw.points[i].y;
        radarpoint_raw.z = eagle_raw.points[i].z;
        radarpoint_raw.intensity = eagle_raw.points[i].Power;
        radarpoint_raw.doppler = eagle_raw.points[i].Doppler;
        radarcloud_raw->points.push_back(radarpoint_raw);
    }
    pcl::toROSMsg(*radarcloud_raw, *pc2_raw_msg);
    pc2_raw_msg->header = eagle_msg.header;
    return pc2_raw_msg;
  }
  PointCloud2ConstPtr convert_altos(const PointCloud2& altos_msg){
    //********** Convert altos_msg to RadarPointCloud **********
    pcl::PointCloud<altos::Point> altos_raw;
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType> );
    pcl::fromROSMsg(altos_msg, altos_raw);
    int point_size = altos_raw.points.size();
    // radarcloud_raw->reserve(point_size);
    RadarPointCloudType radarpoint_raw;
    for (size_t i = 0; i < point_size; ++i)
    {
        //altos radar point reserves 16000 Points per frame, the valid points should be filtered
        if (altos_raw.points[i].x < 0.5) continue;
        radarpoint_raw.x = altos_raw.points[i].x;
        radarpoint_raw.y = altos_raw.points[i].y;
        radarpoint_raw.z = altos_raw.points[i].z;
        radarpoint_raw.intensity = altos_raw.points[i].s;
        radarpoint_raw.doppler = altos_raw.points[i].h;
        radarcloud_raw->points.push_back(radarpoint_raw);
    }
    pcl::toROSMsg(*radarcloud_raw, *pc2_raw_msg);
    pc2_raw_msg->header = altos_msg.header;
    return pc2_raw_msg;
  }
  PointCloud2ConstPtr convert_hercules(const PointCloud2& hercules_msg){
    //********** Convert altos_msg to RadarPointCloud **********
    pcl::PointCloud<hercules::Point> hercules_raw;
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType> );
    pcl::fromROSMsg(hercules_msg, hercules_raw);
    int point_size = hercules_raw.points.size();
    // radarcloud_raw->reserve(point_size);
    RadarPointCloudType radarpoint_raw;
    for (size_t i = 0; i < point_size; ++i)
    {
        //altos radar point reserves 16000 Points per frame, the valid points should be filtered
        if (hercules_raw.points[i].x < 0.5) continue;
        radarpoint_raw.x = hercules_raw.points[i].x;
        radarpoint_raw.y = hercules_raw.points[i].y;
        radarpoint_raw.z = hercules_raw.points[i].z;
        radarpoint_raw.intensity = static_cast<float>(hercules_raw.points[i].RCS);
        radarpoint_raw.intensity = 10.0;

        radarpoint_raw.doppler = hercules_raw.points[i].v;
        radarcloud_raw->points.push_back(radarpoint_raw);
    }
    pcl::toROSMsg(*radarcloud_raw, *pc2_raw_msg);
    pc2_raw_msg->header = hercules_msg.header;
    return pc2_raw_msg;
  }

  std::pair<PointCloud2ConstPtr, Eigen::Vector3d> filter(const PointCloud2ConstPtr& radar_msg) {
    //convert radar_msg to RadarPoint(x, y, z, doppler)
    // compare the time consumption of the two methods
    using Clock = std::chrono::high_resolution_clock;

    if(!reve_en)
    {
      auto lsq_start = Clock::now();
      auto radar_points = convertPointCloudToRadarPoints(radar_msg);
      if (radar_points.empty()) {
        std::cout << "[Ransca-LSQ]: No valid radar points found" << std::endl;
        return std::make_pair(radar_msg, Eigen::Vector3d::Zero());
      }
      RansacEgoVelocity ransac_ego_evel(100, 0.5); // max_iterations = 100, tolerance = 0.5
      RANSACResult result = ransac_ego_evel.calculate(radar_points);
      auto lsq_end = Clock::now();
      double lsq_time = std::chrono::duration<double, std::milli>(lsq_end - lsq_start).count();
      // std::cout << "Least squares method time consumption: " << lsq_time << " ms" << std::endl;
      // convert result to PointCloud2
      sensor_msgs::PointCloud2 outlier_radar_msg;
      sensor_msgs::PointCloud2 inlier_radar_msg;
      inlier_radar_msg = convertRadarPointsToPointCloud(result.inliers);
      outlier_radar_msg = convertRadarPointsToPointCloud(result.outliers);
      inlier_radar_msg.header = radar_msg->header;
      outlier_radar_msg.header = radar_msg->header;
      PointCloud2Ptr inlier_radar_msg_ptr = boost::make_shared<sensor_msgs::PointCloud2>(inlier_radar_msg);
      Eigen::Vector3d v_r, sigma_v_r;
      v_r = result.ego_velocity;
      sigma_v_r = result.covariance.diagonal();
      // std::cout << "[Ransca-LSQ]: Ego velocity is " << result.ego_velocity.transpose() << std::endl;
      return std::make_pair(inlier_radar_msg_ptr, v_r);

      // std::cout << "Covariance matrix: " << std::endl << result.covariance << std::endl;
      // std::cout << "Number of inliers: " << result.inliers.size() << std::endl;
      // std::cout << "Number of outliers: " << result.outliers.size() << std::endl;
    }
    else
    {
      sensor_msgs::PointCloud2 outlier_radar_msg;
      sensor_msgs::PointCloud2 inlier_radar_msg;
      auto reve_start = Clock::now();

      Eigen::Vector3d v_r, sigma_v_r;
      ego_velocity_estimator_.estimate(*radar_msg, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg);
      // std::cout << "[REVE]: radar msg size is " << radar_msg->width << std::endl;
      // std::cout << "inlier size: " << inlier_radar_msg.width << std::endl;
      // Create a shared pointer for the inlier message
      PointCloud2Ptr inlier_radar_msg_ptr = boost::make_shared<sensor_msgs::PointCloud2>(inlier_radar_msg);
      auto reve_end = Clock::now();
      double reve_time = std::chrono::duration<double, std::milli>(reve_end - reve_start).count();
      // std::cout << "REVE method time consumption: " << reve_time << " ms" << std::endl;

      return std::make_pair(inlier_radar_msg_ptr, v_r);
    }
  }

  std::vector<RadarPoint> convertPointCloudToRadarPoints(const sensor_msgs::PointCloud2ConstPtr& radar_msg) {
      // Convert PointCloud2 message to PCL point cloud
      pcl::PointCloud<RadarPointCloudType> pcl_cloud;
      pcl::fromROSMsg(*radar_msg, pcl_cloud);

      std::vector<RadarPoint> radar_points;
      radar_points.reserve(pcl_cloud.points.size());

      for (const auto& point : pcl_cloud.points) {
          // Extract the 3D point and Doppler velocity
          Eigen::Vector3d normal_vector(point.x, point.y, point.z);
          Eigen::Vector3d position(point.x, point.y, point.z);
          double doppler_velocity = point.doppler; // 

          // Normalize the direction vector
          double norm = normal_vector.norm();
          if (norm > 1e-6) { // Avoid division by zero
              normal_vector /= norm;

              // Add the point to the radar_points vector
              radar_points.push_back({normal_vector, position, doppler_velocity});
          }
      }
      return radar_points;
  }
  sensor_msgs::PointCloud2 convertRadarPointsToPointCloud(const std::vector<RadarPoint>& radar_points) {
    // Create a PCL PointCloud object
    pcl::PointCloud<RadarPointCloudType> pcl_cloud;
    // Populate the PCL PointCloud
    for (const auto& point : radar_points) {
        RadarPointCloudType pcl_point;
        pcl_point.x = point.position[0]; // X coordinate
        pcl_point.y = point.position[1]; // Y coordinate
        pcl_point.z = point.position[2]; // Z coordinate
        pcl_point.doppler = point.doppler_velocity; // Use intensity field for Doppler velocity
        pcl_cloud.points.push_back(pcl_point);
    }

    // Convert PCL PointCloud to ROS PointCloud2 message
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(pcl_cloud, cloud_msg);

    // Set additional fields in the ROS message
    cloud_msg.header.frame_id = "radar_frame"; // Set appropriate frame
    cloud_msg.header.stamp = ros::Time::now();

    return cloud_msg;
}

bool reve_en;
private:
  PointCloud2Ptr pc2_raw_msg;
  RadarEgoVelocityEstimator ego_velocity_estimator_;
};
