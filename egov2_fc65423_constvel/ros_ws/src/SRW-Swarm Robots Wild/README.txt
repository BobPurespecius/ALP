This project contains two packages:
1. object_detection
  This package provides object detection based on yolov5 for the tracking experiemnt.
  Pre-request module:
  ros_deep_learning: https://github.com/dusty-nv/ros_deep_learning: for camera streaming in Jetson Xavier NX.
2. uwb_odom_correct
  This package provides localization and drift corection. 
  Pre-request modules:
  Realsense: https://github.com/IntelRealSense/realsense-ros: source images for VINS-Fusion.
  VINS-Fusion-gpu: https://github.com/pjrambo/VINS-Fusion-gpu: VIO localization.
  UWB: https://github.com/nooploop-dev/nlink_parser.git: provides distance between two drones
