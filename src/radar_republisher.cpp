#include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "radar_republisher.h"

namespace radar_republisher {

class RadarRepublisher {
public:
  RadarRepublisher() : nh("~") {
    std::string radar_topic_pc;
    nh.param<std::string>("radar_topic_pc", radar_topic_pc, "/radar_pc");
    std::string radar_topic_pc2;
    nh.param<std::string>("radar_topic_pc2", radar_topic_pc2, "/radar_pc_2");
    std::string radar_topic_ars;
    nh.param<std::string>("radar_topic_ars", radar_topic_ars, "/ars548_pc");
    std::string radar_topic_eagle_pc2;
    nh.param<std::string>("radar_topic_eagle_pc2", radar_topic_eagle_pc2, "/eagle_pc2");    
    //pointcloud2
    pc2_msg_sub = nh.subscribe(radar_topic_pc2, 10, &RadarRepublisher::pc2_callback, this);
    //pointcloud
    pc_msg_sub = nh.subscribe(radar_topic_pc, 10, &RadarRepublisher::pc_callback, this);
    ///! pointcloud2 ars548 ros msg
    ars_msg_sub = nh.subscribe(radar_topic_ars, 10, &RadarRepublisher::ars_pc2_callback, this);
    ///! pointcloud2 eagle ros msg
    eagle_pc2_sub = nh.subscribe(radar_topic_eagle_pc2, 10, &RadarRepublisher::eagle_pc2_callback, this);
    nh.param<bool>("debug_mode", debug, true);
    nh.param<bool>("lsq_filter", lsq_filter, false);

    msg_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_pc2", 10);
  }

  void pc2_callback(const sensor_msgs::PointCloud2::ConstPtr&  hugin_msg) {
    auto raw_msg_size = hugin_msg->width;
    auto radar_msg = RadarConverter.convert(*hugin_msg);
    PointCloud2ConstPtr filter_radar_msg;
    if (lsq_filter)
    {
      filter_radar_msg = RadarConverter.filter(radar_msg);
    }
    else {
      filter_radar_msg = radar_msg;
    }
    auto filter_msg_size = filter_radar_msg->width;
    auto lost_msg_size = raw_msg_size - filter_msg_size;
    float lost_rate = 100 * (static_cast<float>(lost_msg_size) / raw_msg_size);
    if(debug) ROS_INFO_STREAM_THROTTLE(1, "size of lost radar points: " << lost_msg_size << "  lost rate: " << lost_rate << " %");
    msg_pub.publish(filter_radar_msg);
  }

  void pc_callback(const sensor_msgs::PointCloud::ConstPtr&  eagle_msg) {
    auto raw_msg_size = eagle_msg->points.size();
    auto radar_msg = RadarConverter.convert(*eagle_msg);
    PointCloud2ConstPtr filter_radar_msg;
    if (lsq_filter)
    {
      filter_radar_msg = RadarConverter.filter(radar_msg);
    }
    else {
      filter_radar_msg = radar_msg;
    }
    auto filter_msg_size = filter_radar_msg->width;
    auto lost_msg_size = raw_msg_size - filter_msg_size;
    float lost_rate = 100 * (static_cast<float>(lost_msg_size) / raw_msg_size);
    if(debug) ROS_INFO_STREAM_THROTTLE(1, "size of lost radar points: " << lost_msg_size << "  lost rate: " << lost_rate << " %");
    msg_pub.publish(filter_radar_msg);
  }
  void ars_pc2_callback(const sensor_msgs::PointCloud2::ConstPtr&  ars_msg) {
    auto raw_msg_size = ars_msg->width;
    auto radar_msg = RadarConverter.convert_ars(*ars_msg);
    PointCloud2ConstPtr filter_radar_msg;
    if (lsq_filter)
    {
      filter_radar_msg = RadarConverter.filter(radar_msg);
    }
    else {
      filter_radar_msg = radar_msg;
    }
    auto filter_msg_size = filter_radar_msg->width;
    auto lost_msg_size = raw_msg_size - filter_msg_size;
    float lost_rate = 100 * (static_cast<float>(lost_msg_size) / raw_msg_size);
    if(debug) ROS_INFO_STREAM_THROTTLE(1, "size of lost radar points: " << lost_msg_size << "  lost rate: " << lost_rate << " %");
    msg_pub.publish(filter_radar_msg);
  }
  void eagle_pc2_callback(const sensor_msgs::PointCloud2::ConstPtr&  eagle_msg) {
    auto raw_msg_size = eagle_msg->width;
    auto radar_msg = RadarConverter.convert_eagle(*eagle_msg);
    PointCloud2ConstPtr filter_radar_msg;
    if (lsq_filter)
    {
      filter_radar_msg = RadarConverter.filter(radar_msg);
    }
    else {
      filter_radar_msg = radar_msg;
    }
    auto filter_msg_size = filter_radar_msg->width;
    auto lost_msg_size = raw_msg_size - filter_msg_size;
    float lost_rate = 100 * (static_cast<float>(lost_msg_size) / raw_msg_size);
    if(debug) ROS_INFO_STREAM_THROTTLE(1, "size of lost radar points: " << lost_msg_size << "  lost rate: " << lost_rate << " %");
    msg_pub.publish(filter_radar_msg);
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber pc2_msg_sub;
  ros::Subscriber pc_msg_sub;
  ros::Subscriber ars_msg_sub;
  ros::Subscriber eagle_pc2_sub;
  ros::Publisher msg_pub;

  RadarMsgConverter RadarConverter;
  bool debug;
  bool lsq_filter;
};

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "radar_republisher");
  radar_republisher::RadarRepublisher node;
  ROS_INFO("radar_republisher node started");
  ros::spin();

  return 0;
}