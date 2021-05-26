# Tresc3 scan algorithm gl
Tresc3 scan algorithm using gl_driver with ROS

## Guide
#### Set permission of USB port
```
$ sudo chmod a+rw /dev/ttyUSB0
```
#### Set permission of USB port permanently
```
$ sudo usermod -a -G dialout $USER
```
and reboot.

#### Change serial port in `tresc3_scan_algorithm_gl/launch/tresc3_scan_algorithm_gl.launch`

### Run Tresc3_scan_node and Gl-3 Publisher node with RViz
```
$ roslaunch tresc3_scan_algorithm_gl view_tresc3_scan_algorithm_gl.launch direction:=(up or down) velocity:=(m/s) height:=(m)
```
- You need to launch with such params. direction, velocity, height.
  - The direction means direction of target object. It is either up or down.
  - The velocity means velocity of target object.
  - The height means height of lidar.
  - The unit of velocity is m/s and the unit of height is m.

#### Published Topics
- _pcl_data_ (sensor_msgs/PointCloud): it publishes pcl_data topic with xyz point cloud.

### Test environment
- ROS Melodic Morenia
- Ubuntu 18.04
- x86_64 (PC)
- SOSLAB GL-310 2D Lidar