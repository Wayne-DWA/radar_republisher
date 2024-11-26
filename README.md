# radar_republisher

This package provides ROS1 interfaces to convert various radar msg into ```sensor_msgs::PointCloud2```.

## Usage

1. Modify the topic name in the launchfile
2. Launch radar republisher
```bash
  # Launch as a ROS1 node
  roslaunch radar_republisher radar_repub.launch
```
- In/Out Topics:
  - Input topic: **/radar_topic_pc** or **/radar_topic_pc2**
  - Output topic: **/radar_pc2**

- Point fields in the output pointcloud:
  - RadarPointCloudType,
  - (float, x, x)
  - (float, y, y)
  - (float, z, z)
  - (float, intensity, intensity)
  - (float, doppler, doppler)

## License

This package is released under the MIT license.