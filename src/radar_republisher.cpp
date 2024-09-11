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
    nh.param<std::string>("radar_topic_pc", radar_topic_pc, "/radar_enhanced_pcl");
    std::string radar_topic_pc2;
    nh.param<std::string>("radar_topic_pc2", radar_topic_pc2, "/hugin_raf_1/radar_data");    
    //pointcloud2
    pc2_msg_sub = nh.subscribe(radar_topic_pc2, 10, &RadarRepublisher::pc2_callback, this);
    //pointcloud
    pc_msg_sub = nh.subscribe(radar_topic_pc, 10, &RadarRepublisher::pc_callback, this);
    msg_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_pc2", 10);
  }

  void pc2_callback(const sensor_msgs::PointCloud2::ConstPtr&  hugin_msg) {
    const auto radar_msg = RadarConverter.convert(*hugin_msg);
    msg_pub.publish(radar_msg);
  }

  void pc_callback(const sensor_msgs::PointCloud::ConstPtr&  eagle_msg) {
    const auto radar_msg = RadarConverter.convert(*eagle_msg);
    msg_pub.publish(radar_msg);
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber pc2_msg_sub;
  ros::Subscriber pc_msg_sub;
  ros::Publisher msg_pub;

  RadarMsgConverter RadarConverter;
};

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "radar_republisher");
  radar_republisher::RadarRepublisher node;
  ROS_INFO("radar_republisher node started");
  ros::spin();

  return 0;
}