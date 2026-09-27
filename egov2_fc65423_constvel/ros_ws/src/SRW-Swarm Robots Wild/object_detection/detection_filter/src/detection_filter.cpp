#include "detection_filter/detection_filter.h"

namespace detection_filter
{

    /****************************Detection Filter****************************/

    using namespace std;

    DetectionFilter::DetectionFilter(ros::NodeHandle &nodeHandle)
        : nh_(nodeHandle)
    {
        readParameters();
        initialization();
    }

    void DetectionFilter::readParameters()
    {
        nh_.getParam("detection_filter/detection_bboxes_topic", detection_bboxes_topic_);
        nh_.getParam("detection_filter/yolo_car_odom_pub_topic", yolo_car_odom_pub_topic_);
        nh_.getParam("detection_filter/ekf_car_odom_pub_topic", ekf_car_odom_pub_topic_);
        nh_.getParam("detection_filter/pitch_thr", pitch_thr_);
        nh_.getParam("detection_filter/ekf_reset_dist_thr", ekf_reset_dist_thr_);
        // camera
        nh_.getParam("detection_filter/cam_fx", cam_fx_);
        nh_.getParam("detection_filter/cam_fy", cam_fy_);
        nh_.getParam("detection_filter/cam_cx", cam_cx_);
        nh_.getParam("detection_filter/cam_cy", cam_cy_);
        nh_.getParam("detection_filter/R_cam_00", R_cam_00_);
        nh_.getParam("detection_filter/R_cam_10", R_cam_10_);
        nh_.getParam("detection_filter/R_cam_20", R_cam_20_);
        nh_.getParam("detection_filter/R_cam_01", R_cam_01_);
        nh_.getParam("detection_filter/R_cam_11", R_cam_11_);
        nh_.getParam("detection_filter/R_cam_21", R_cam_21_);
        nh_.getParam("detection_filter/R_cam_02", R_cam_02_);
        nh_.getParam("detection_filter/R_cam_12", R_cam_12_);
        nh_.getParam("detection_filter/R_cam_22", R_cam_22_);

        nh_.getParam("detection_filter/object_height", known_height_);

        nh_.getParam("ekf/ekf_frequence", ekf_frequence_);
        nh_.getParam("ekf/ekf_pred_times_thr", ekf_pred_times_thr_);
        ekf_sleep_time_ = 1.0 / ekf_frequence_ * 1000;
    }

    void DetectionFilter::initialization()
    {
        x_ekf_.initialization(nh_);
        y_ekf_.initialization(nh_);
        ekf_process_ = false;
        ekf_pred_times_ = 0;

        // ! variable
        R_cam2pixel_ << cam_fx_, 0, cam_cx_,
            0, cam_fy_, cam_cy_,
            0, 0, 1;
        R_cam_ << R_cam_00_, R_cam_01_, R_cam_02_,
            R_cam_10_, R_cam_11_, R_cam_12_,
            R_cam_20_, R_cam_21_, R_cam_22_;

        have_yolo_ = false;

        yolo_sub_.reset(new message_filters::Subscriber<object_detection_msgs::BoundingBoxes>(nh_, detection_bboxes_topic_, 1, ros::TransportHints().tcpNoDelay()));
        drone_odom_sub_for_yolo_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "drone_odom_topic", 100, ros::TransportHints().tcpNoDelay()));

        sync_yolo_odom_.reset(new message_filters::Synchronizer<SyncPolicyYoloOdom>(
            SyncPolicyYoloOdom(200), *yolo_sub_, *drone_odom_sub_for_yolo_));
        sync_yolo_odom_->registerCallback(boost::bind(&DetectionFilter::YoloOdomCallback, this, _1, _2));

        yolo_car_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(yolo_car_odom_pub_topic_, 1);
        ekf_car_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(ekf_car_odom_pub_topic_, 1);
    }

    void DetectionFilter::YoloOdomCallback(const object_detection_msgs::BoundingBoxesConstPtr &bboxes_msg, const nav_msgs::OdometryConstPtr &odom_msg)
    {
        Eigen::Vector3d odom_p;
        Eigen::Quaterniond odom_q;
        odom_p(0) = odom_msg->pose.pose.position.x;
        odom_p(1) = odom_msg->pose.pose.position.y;
        odom_p(2) = odom_msg->pose.pose.position.z;
        odom_q.w() = odom_msg->pose.pose.orientation.w;
        odom_q.x() = odom_msg->pose.pose.orientation.x;
        odom_q.y() = odom_msg->pose.pose.orientation.y;
        odom_q.z() = odom_msg->pose.pose.orientation.z;

        std::lock_guard<std::mutex> lock(yolo_mutex_);
        have_yolo_ = true;
        yolo_bbox_ = (*bboxes_msg).bounding_boxes[0];

        if ( yolo_bbox_.ymin <= 5 || yolo_bbox_.ymax >= 715 ) // 1280*720 image
        {
            return;
        }

        double xmin, ymin, xmax, ymax;
        xmin = yolo_bbox_.xmin;
        xmax = yolo_bbox_.xmax;
        ymin = yolo_bbox_.ymin;
        ymax = yolo_bbox_.ymax;

        yolo_car_odom_ = filterFunctionYolo(odom_p, odom_q, xmin, xmax, ymin, ymax);

        // Eigen::Vector3d eulerAngle = odom_q.matrix().eulerAngles(2,1,0);
        // double pitch_now = fabs( eulerAngle[1] / M_PI * 180 );
        // if( pitch_now > 90 )
        //     pitch_now = 180 - pitch_now;

        // std::cout << "drone pitch = " << pitch_now << std::endl;

        // if( pitch_now > pitch_thr_ )
        // {
        //     ROS_ERROR("YOLO:The pitch of the drone is too big!!!We don't believe the PC!");
        //     ROS_ERROR("YOLO:The pitch of the drone is too big!!!We don't believe the PC!");
        //     ROS_ERROR("YOLO:The pitch of the drone is too big!!!We don't believe the PC!");
        //     have_yolo_ = false;

        //     return;
        // }
    }

    Eigen::Vector3d DetectionFilter::filterFunctionYolo(Eigen::Vector3d odom_p, Eigen::Quaterniond odom_q,
                                                        double xmin, double xmax, double ymin, double ymax)
    {
        Eigen::Vector3d yolo_point_min, yolo_point_max;
        yolo_point_min[0] = xmin;
        yolo_point_min[1] = ymin;
        yolo_point_min[2] = 1.0;
        yolo_point_max[0] = xmax;
        yolo_point_max[1] = ymax;
        yolo_point_max[2] = 1.0;

        Eigen::Vector3d new_yolo_point_min = R_cam2pixel_.inverse() * yolo_point_min;
        Eigen::Vector3d new_yolo_point_max = R_cam2pixel_.inverse() * yolo_point_max;

        double observed_height = new_yolo_point_max[1] - new_yolo_point_min[1];
        double Z = known_height_ / observed_height;
        Eigen::Vector3d yolo_point = (Z / 2.0) * (new_yolo_point_min + new_yolo_point_max);

        yolo_point = R_cam_ * yolo_point;
        yolo_point = odom_q * yolo_point + odom_p;

        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "world";
        odom_msg.pose.pose.position.x = yolo_point[0];
        odom_msg.pose.pose.position.y = yolo_point[1];
        odom_msg.pose.pose.position.z = yolo_point[2];
        odom_msg.pose.pose.orientation.w = 1.0;
        yolo_car_odom_pub_.publish(odom_msg);

        // ekf
        if (ekf_process_)
        {
            // update
            ekf_mutex_.lock();
            x_ekf_.update(yolo_point[0]);
            y_ekf_.update(yolo_point[1]);
            ekf_pred_times_ = 0;
            ekf_mutex_.unlock();
        }
        else
        {
            // reset
            // check reset
            // Eigen::Vector2d x_state = x_ekf_.getState();
            // Eigen::Vector2d y_state = y_ekf_.getState();
            // double ekf_dist = ( x_state[0] - current_car_p[0] ) * ( x_state[0] - current_car_p[0] ) + ( y_state[0] - current_car_p[1] ) * ( y_state[0] - current_car_p[1] );
            // ekf_dist = sqrt(ekf_dist);

            // if( ekf_dist > ekf_reset_dist_thr_ )
            // {
            //     ROS_ERROR("The new ekf odom is too far away!!!");
            //     ROS_ERROR("The new ekf odom is too far away!!!");
            //     ROS_ERROR("The new ekf odom is too far away!!!");
            //     return;
            // }

            double delta_t = 1.0 / ekf_frequence_;
            x_ekf_.resetEKF(delta_t, yolo_point[0]);
            y_ekf_.resetEKF(delta_t, yolo_point[1]);
            ekf_process_ = true;
        }

        return yolo_point;
    }

    void DetectionFilter::EKFProcess()
    {
        int ekf_sleep_time_ms = (int)ekf_sleep_time_;
        std::cout << ekf_sleep_time_ms << std::endl;
        while (ros::ok())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(ekf_sleep_time_ms));

            if (!ekf_process_)
                continue;

            ekf_mutex_.lock();

            x_ekf_.prediction();
            y_ekf_.prediction();
            Eigen::Vector2d x_state = x_ekf_.getState();
            Eigen::Vector2d y_state = y_ekf_.getState();

            ekf_pred_times_++;
            if (ekf_pred_times_ > ekf_pred_times_thr_)
            {
                ekf_pred_times_ = 0;
                ekf_process_ = false;
            }

            ekf_mutex_.unlock();

            nav_msgs::Odometry odom_msg;
            if (!ekf_process_)
            {
                std::cout << "detection fail too long time" << std::endl;
            }
            //  pub ekf result
            odom_msg.header.frame_id = "world";
            odom_msg.pose.pose.position.x = x_state[0];
            odom_msg.pose.pose.position.y = y_state[0];
            odom_msg.pose.pose.position.z = 0.0;
            odom_msg.pose.pose.orientation.w = 1.0;
            odom_msg.twist.twist.linear.x = x_state[1];
            odom_msg.twist.twist.linear.y = y_state[1];
            ekf_car_odom_pub_.publish(odom_msg);
        }
    }

    /****************************EKF****************************/

    void EKF::initialization(ros::NodeHandle &nodeHandle)
    {
        nh_ = nodeHandle;
        nh_.getParam("ekf/p_noise_prediction", p_noise_prediction_);
        nh_.getParam("ekf/v_noise_prediction", v_noise_prediction_);
        nh_.getParam("ekf/noise_observation", noise_observation_);
        nh_.getParam("ekf/p_prior_var", p_prior_var_);
        nh_.getParam("ekf/v_prior_var", v_prior_var_);
        nh_.getParam("ekf/max_vel", max_vel_);
    }

    void EKF::resetEKF(double delta_t, double observation)
    {
        std::cout << "reset EKF !!!" << std::endl;
        state_[0] = observation;
        state_[1] = 0;
        Q_ << p_noise_prediction_, 0,
            0, v_noise_prediction_;
        F_ << 1, delta_t,
            0, 0.8;
        R_ = noise_observation_;
        P_ << p_prior_var_, 0,
            0, v_prior_var_;
        H_ << 1, 0;
    }

    void EKF::prediction()
    {
        state_ = F_ * state_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void EKF::update(double observation)
    {
        Eigen::Vector2d K = P_ * H_.transpose() / (H_ * P_ * H_.transpose() + R_);
        Eigen::Vector2d state_nolimit = state_ + K * (observation - H_ * state_);
        // cout << "K=" << K.transpose() << " observation=" << observation << " state=" << state_.transpose() << endl;
        double vel_from_diff = (state_nolimit(0) - state_(0)) / F_(0,1);
        if ( vel_from_diff > max_vel_ )
        {
            // cout << "max_vel_=" << max_vel_ << endl;
            // cout << "state_nolimit=" << state_nolimit.transpose() << endl;
            double limited_obsv = max_vel_ * F_(0,1) / K(0) + H_ * state_;
            state_ = state_ + K * (limited_obsv - H_ * state_);
            // cout << "state_=" << state_.transpose() << endl;
        }
        else if( vel_from_diff < -max_vel_ )
        {
            // cout << "max_vel_=" << -max_vel_ << endl;
            // cout << "state_nolimit=" << state_nolimit.transpose() << endl;
            double limited_obsv = -max_vel_ * F_(0,1) / K(0) + H_ * state_;
            state_ = state_ + K * (limited_obsv - H_ * state_);
            // cout << "state_=" << state_.transpose() << endl;
        }
        else
        {
            state_ = state_nolimit;
        }
        P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;
    }

    Eigen::Vector2d EKF::getState()
    {
        return state_;
    }

} // namespace detection_filter
