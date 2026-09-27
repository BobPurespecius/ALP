#include "detection_filter/detection_filter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_filter");
    ros::NodeHandle nh("~");

    detection_filter::DetectionFilter filter(nh);

    std::thread ekf_process{ &detection_filter::DetectionFilter::EKFProcess, &filter };

    ros::spin();

    ROS_ERROR("************************************");
    ROS_ERROR("The detection_filter node is dead!!!");
    ROS_ERROR("************************************");
    return 0;
}

/***********************************************************/
// 1. ekf位置限制
// 2. 姿态角判断
// 3. yolo点云先后顺序改变
// 4. yolo近大远小
/***********************************************************/
