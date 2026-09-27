#pragma once
// ROS
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
// detection
#include <object_detection_msgs/BoundingBox.h>
#include <object_detection_msgs/BoundingBoxes.h>
#include <object_detection_msgs/ObjectCount.h>
#include <object_detection_msgs/CarPosition.h>
// c++
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string> 
#include <math.h>
#include <chrono>
#include <mutex>
#include <thread>

namespace detection_filter {

class EKF
{
    public:
        EKF(){};
        virtual ~EKF(){};
        void prediction();
        void update( double observation );
        Eigen::Vector2d getState();
        void initialization(ros::NodeHandle& nodeHandle);
        void resetEKF( double delta_t, double observation );
    
    private:
        ros::NodeHandle nh_;

        Eigen::Vector2d state_; // p v
        Eigen::Matrix2d P_, F_, Q_;
        Eigen::RowVector2d H_;
        double R_;
        double p_noise_prediction_, v_noise_prediction_;
        double noise_observation_;
        double p_prior_var_, v_prior_var_;
        double max_vel_;

        std::list<nav_msgs::Odometry> drone_odom_buf_;
};

class DetectionFilter
{
    public:
        DetectionFilter(ros::NodeHandle& nodeHandle);
        virtual ~DetectionFilter(){};
        void EKFProcess();


    private:
        void readParameters();
        void initialization();
        Eigen::Vector3d  filterFunctionYolo( Eigen::Vector3d odom_p, Eigen::Quaterniond odom_q,
                                             double xmin, double xmax, double ymin, double ymax );

        // callback
        void YoloOdomCallback(const object_detection_msgs::BoundingBoxesConstPtr &bboxes_msg, const nav_msgs::OdometryConstPtr &odom_msg);
        void detectionBboxesCallback(const object_detection_msgs::BoundingBoxesConstPtr &bboxes_msg);
        
        /* *** */
        // ROS node handle
        ros::NodeHandle nh_;

        // param
        // rostopic 
        std::string detection_bboxes_topic_;
        std::string yolo_car_odom_pub_topic_;
        std::string ekf_car_odom_pub_topic_;
        // const param
        Eigen::Matrix3d R_cam2pixel_;    // camera to pixel 
        Eigen::Matrix3d R_cam_;          // camera to world
        double cam_fx_, cam_fy_, cam_cx_, cam_cy_;
        double R_cam_00_, R_cam_10_, R_cam_20_, R_cam_01_, R_cam_11_, R_cam_21_, R_cam_02_, R_cam_12_, R_cam_22_;
        int ekf_pred_times_thr_;
        double ekf_frequence_, ekf_sleep_time_;
        double pitch_thr_;
        double known_height_;
        double ekf_reset_dist_thr_;

        // subscriber
        typedef message_filters::sync_policies::ApproximateTime<object_detection_msgs::BoundingBoxes, nav_msgs::Odometry>
            SyncPolicyYoloOdom;
        typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyYoloOdom>> SynchronizerYoloOdom;

        std::shared_ptr<message_filters::Subscriber<object_detection_msgs::BoundingBoxes>> yolo_sub_;
        std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> drone_odom_sub_for_yolo_;
        SynchronizerYoloOdom sync_yolo_odom_;
        

        // publisher
        ros::Publisher yolo_car_odom_pub_;
        ros::Publisher ekf_car_odom_pub_;

        // global
        std::mutex yolo_mutex_, ekf_mutex_;
        Eigen::Vector3d drone_odom_p_;
        EKF x_ekf_, y_ekf_;
        bool ekf_process_;
        int ekf_pred_times_;
        bool have_yolo_;
        object_detection_msgs::BoundingBox yolo_bbox_;
        Eigen::Vector3d yolo_car_odom_;

};

}