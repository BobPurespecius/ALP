#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>

int main(int argc, char** argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "yolov5trt");
    ros::NodeHandle n; 
    ros::Rate loop_rate(5);
    
     // 定义节点句柄   
    image_transport::ImageTransport it(n);
    //image_transport::Publisher pub = it.advertise("/camera/rgb/image_raw", 1);
    image_transport::Publisher pub = it.advertise("/playvideo/camera1", 1);
    sensor_msgs::ImagePtr msg;

    // opencv准备读取视频
    cv::VideoCapture video;
    video.open("/home/abcd/3.mp4");

    if( !video.isOpened() )
    {
        ROS_INFO("Read Video failed!\n");
        return 0;
    }
    cv::Mat frame;
    int count = 0;
    while(1)
    {
        video >> frame;
        if( frame.empty() )
            break;
        count++;

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);

        ROS_INFO( "read the %dth frame successfully!", count );
        loop_rate.sleep();
        ros::spinOnce();
    }
    video.release();

    return 0;
}