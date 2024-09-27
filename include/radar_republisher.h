#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "radar_ego_velocity_estimator.hpp"

using PointCloud2 = sensor_msgs::PointCloud2;
using PointCloud = sensor_msgs::PointCloud;
using PointCloud2Ptr = sensor_msgs::PointCloud2::Ptr;
using PointCloud2ConstPtr = sensor_msgs::PointCloud2::ConstPtr;

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

class RadarMsgConverter {
public:
  RadarMsgConverter() {
    pc2_raw_msg.reset(new PointCloud2);
  }

  PointCloud2ConstPtr convert(const PointCloud2& radar_msg) {
    //********** Convert HuginPointCloud to RadarPointCloud **********
    pcl::PointCloud<HuginPointCloudType>::Ptr hugin_raw( new pcl::PointCloud<HuginPointCloudType> );
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw( new pcl::PointCloud<RadarPointCloudType> );
    pcl::fromROSMsg(radar_msg, *hugin_raw);
    RadarPointCloudType radarpoint_raw;
    for (size_t i = 0; i < hugin_raw->size(); ++i)
    {
        radarpoint_raw.x = hugin_raw->at(i).x;
        radarpoint_raw.y = hugin_raw->at(i).y;
        radarpoint_raw.z = hugin_raw->at(i).z;
        radarpoint_raw.intensity = hugin_raw->at(i).power;
        radarpoint_raw.doppler = hugin_raw->at(i).doppler;
        radarcloud_raw->points.push_back(radarpoint_raw);
    }
    pcl::toROSMsg(*radarcloud_raw, *pc2_raw_msg);
    pc2_raw_msg->header = radar_msg.header;
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
  PointCloud2ConstPtr filter(const PointCloud2ConstPtr& radar_msg) {
    sensor_msgs::PointCloud2 outlier_radar_msg;
    sensor_msgs::PointCloud2 inlier_radar_msg;
    Eigen::Vector3d v_r, sigma_v_r;
    ego_velocity_estimator_.estimate(*radar_msg, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg);
        // Create a shared pointer for the inlier message
    PointCloud2Ptr inlier_radar_msg_ptr = boost::make_shared<sensor_msgs::PointCloud2>(inlier_radar_msg);

    return inlier_radar_msg_ptr;
  }


private:
  PointCloud2Ptr pc2_raw_msg;
  RadarEgoVelocityEstimator ego_velocity_estimator_;
};
