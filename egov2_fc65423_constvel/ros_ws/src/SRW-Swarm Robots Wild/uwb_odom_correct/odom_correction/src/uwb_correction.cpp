#include "nav_msgs/Odometry.h"
#include "eigen3/Eigen/Eigen"
#include <iostream>
#include "ceres/ceres.h"
#include "ros/ros.h"
#include <nlink_parser/LinktrackNodeframe3.h>
#include "vector"
#include "map"
#include "geometry_msgs/PointStamped.h"
#include "random"
#include "std_msgs/Float64.h"

using namespace std;

class AnchorTriangulationErrorTerm
{
public:
    AnchorTriangulationErrorTerm(const Eigen::Vector3d anchor,
                                 const double r, const double weight, const double fixed_z = 0.0) : m_anchor(anchor), m_r(r), m_weight(weight), m_fixed_z(fixed_z)
    {
        ; // m_self_d = anchor - self_pos;
    }

    template <typename T>
    bool operator()(const T *const x, const T *const y, const T *const z,
                    T *residuals_ptr) const
    {
        const Eigen::Matrix<T, 3, 1> anchor = m_anchor.cast<T>();
        const Eigen::Matrix<T, 3, 1> self_pos(*x, *y, *z);
        Eigen::Matrix<T, 3, 1> diff = self_pos - anchor;

        residuals_ptr[0] = (ceres::abs(diff.norm() - T(m_r))) * T(m_weight);
        return true;
    }

    template <typename T>
    bool operator()(const T *const x, const T *const y,
                    T *residuals_ptr) const
    {
        const Eigen::Matrix<T, 3, 1> anchor = m_anchor.cast<T>();
        const Eigen::Matrix<T, 3, 1> self_pos(*x, *y, T(m_fixed_z));
        Eigen::Matrix<T, 3, 1> diff = self_pos - anchor;

        residuals_ptr[0] = (ceres::abs(diff.norm() - T(m_r))) * T(m_weight);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d anchor,
                                       const double r, const double weight,
                                       const bool optimize_z, double fixed_z)
    {
        if (optimize_z)
            return (new ceres::AutoDiffCostFunction<AnchorTriangulationErrorTerm, 1, 1, 1, 1>(new AnchorTriangulationErrorTerm(anchor, r, weight)));
        else
            return (new ceres::AutoDiffCostFunction<AnchorTriangulationErrorTerm, 1, 1, 1>(new AnchorTriangulationErrorTerm(anchor, r, weight, fixed_z)));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Eigen::Vector3d m_anchor;
    const double m_r, m_weight, m_fixed_z;
};

ros::Subscriber self_odom_sub_, other_odom_sub_, uwb_info_sub_, ground_height_sub_;
ros::Publisher correction_odom_pub_, drift_point_pub_;
// bool is_output_drift_;

nav_msgs::Odometry self_odom_msg_;
Eigen::Vector3d self_pos_;
map<int, nav_msgs::Odometry> other_odom_msg_;
map<int, Eigen::Vector3d> other_pos_;
int fixed_anchor_num_;
map<int, Eigen::Vector3d> fixed_anchor_pos_;

int self_id_;
double max_error_;
double min_distance_;
double max_step_;
double filter_factor_;
double in_filter_factor_;
Eigen::Vector3d delta_filter_;

int self_odom_count_ = 0;
int other_odom_count_ = 0;
int uwb_count_ = 0;

bool optimize_z_;

void self_odom_cb(const nav_msgs::OdometryPtr &msg)
{
    self_odom_count_++;

    self_odom_msg_ = *msg;
    self_pos_ << self_odom_msg_.pose.pose.position.x,
        self_odom_msg_.pose.pose.position.y,
        self_odom_msg_.pose.pose.position.z;
    nav_msgs::Odometry corr_msg(self_odom_msg_);
    corr_msg.pose.pose.position.x += delta_filter_.x();
    corr_msg.pose.pose.position.y += delta_filter_.y();
    corr_msg.pose.pose.position.z += delta_filter_.z();
    correction_odom_pub_.publish(corr_msg);
}

void other_odom_cb(const nav_msgs::OdometryPtr &msg)
{
    other_odom_count_++;

    int id = atoi(msg->child_frame_id.substr(6, 10).c_str());
    if (msg->child_frame_id.substr(0, 6) != string("drone_"))
    {
        ROS_ERROR("[random_goals_node] Wrong child_frame_id: %s", msg->child_frame_id.substr(0, 6).c_str());
        return;
    }
    other_odom_msg_[id] = *msg;
    // other_odom_msg_[id].header.stamp = ros::Time::now(); // zxzxzxzx
    other_pos_[id] << msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z;
}

bool check_data_avalible(const nlink_parser::LinktrackNode2 &n, Eigen::Vector3d &other_pos)
{
    // if (n.dis < min_distance_)
    // {
    //     return false;
    // }
    Eigen::Vector3d diff = other_pos - self_pos_ - delta_filter_;
    double distance = diff.norm();
    // for (size_t i = 0; i < 3; i++)
    // {
    //     if (fabs(diff[i])< min_distance_)
    //     {
    //         return false;
    //     }
    // }

    if (fabs(distance - n.dis) > max_error_)
    {
        return false;
    }

    return true;
}

void drift_vis(Eigen::Vector3d displcement)
{
    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/world";

    msg.point.x = displcement.x();
    msg.point.y = displcement.y();
    msg.point.z = displcement.z();

    drift_point_pub_.publish(msg);
}

void fuse_result(Eigen::Vector3d &uwb_pos)
{
    Eigen::Vector3d diff = uwb_pos - self_pos_;
    Eigen::Vector3d diff_diff = diff - delta_filter_;
    Eigen::Vector3d diff_data = filter_factor_ * diff + in_filter_factor_ * delta_filter_;
    Eigen::Vector3d diff_data_d = diff_data - delta_filter_;
    for (size_t i = 0; i < 3; i++)
    {
        if (diff_data_d[i] < -max_step_)
        {
            delta_filter_[i] = delta_filter_[i] - max_step_;
        }
        else if (diff_data_d[i] > max_step_)
        {
            delta_filter_[i] = delta_filter_[i] + max_step_;
        }
        else
        {
            delta_filter_[i] = diff_data[i];
        }
    }
}

void uwb_info_cb(const nlink_parser::LinktrackNodeframe3Ptr &msg)
{
    uwb_count_++;

    ros::Time now = ros::Time::now();
    if (self_id_ != msg->id)
    {
        ROS_ERROR("Your drone ID is not equal to UWB ID!!");
        return;
    }

    ceres::Problem m_problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    double x = self_pos_.x() + delta_filter_.x();
    double y = self_pos_.y() + delta_filter_.y();
    double z = self_pos_.z() + delta_filter_.z();
    m_problem.AddParameterBlock(&x, 1);
    m_problem.AddParameterBlock(&y, 1);
    if (optimize_z_)
        m_problem.AddParameterBlock(&z, 1);

    vector<Eigen::Vector3d> correct_odom_buf;
    for (auto &n : msg->nodes)
    {
        int find_times = 0;

        auto anchor_pos = other_pos_.find(n.id);
        if (anchor_pos != other_pos_.end() && abs((now - other_odom_msg_.find(n.id)->second.header.stamp).toSec()) < 0.2)
        {

            if (!check_data_avalible(n, anchor_pos->second))
            {
                continue;
            }

            ceres::CostFunction *cost_function = AnchorTriangulationErrorTerm::Create(anchor_pos->second, n.dis, 1, optimize_z_, z); //weight is 1

            if (optimize_z_)
                m_problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0), &x, &y, &z);
            else
                m_problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0), &x, &y);

            find_times++;
        }

        anchor_pos = fixed_anchor_pos_.find(n.id);
        if (anchor_pos != fixed_anchor_pos_.end())
        {

            if (!check_data_avalible(n, anchor_pos->second))
            {
                continue;
            }

            ceres::CostFunction *cost_function = AnchorTriangulationErrorTerm::Create(anchor_pos->second, n.dis, 1, optimize_z_, z); //weight is 1

            if (optimize_z_)
                m_problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0), &x, &y, &z);
            else
                m_problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0), &x, &y);

            find_times++;
        }

        if (find_times >= 2)
        {
            ROS_ERROR("ID of drone anchors and fixed anchors are in conflict!");
        }
    }

    // ceres::CostFunction *cost_function = AnchorTriangulationErrorTerm::Create(self_pos_ + delta_filter_, 0.0, 1, optimize_z_, z); //close to myself
    // if (optimize_z_)
    //     m_problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0), &x, &y, &z);
    // else
    //     m_problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0), &x, &y);

    if (m_problem.NumResidualBlocks() >= 1)
    {
        ceres::Solve(options, &m_problem, &summary);
        // std::cout << summary.FullReport() << "\n";
        // std::cout << "Current Resluat: x=" << x << " y=" << y << " z=" << z << endl;
        Eigen::Vector3d pos_basedon_anchors(x, y, z);
        fuse_result(pos_basedon_anchors);
        drift_vis(delta_filter_);
    }
    // else
    // {
    //     ROS_ERROR("m_problem.NumResidualBlocks()=%d", m_problem.NumResidualBlocks());
    // }
}

void ground_height_cb(const std_msgs::Float64Ptr &msg)
{
    double ground_height = msg->data;
    if (ground_height > 0.5)
        ground_height = 0.5;
    if (ground_height < -0.5)
        ground_height = -0.5;

    static std::list<double> g_hei_list;
    g_hei_list.push_back(ground_height);
    while (g_hei_list.size() > 20)
    {
        g_hei_list.pop_front();
    }

    double avg_hei = 0;
    int i = 0;
    for (auto it = g_hei_list.begin(); it != g_hei_list.end(); ++it)
    {
        ++i;
        avg_hei += *it;
    }
    avg_hei /= i;

    Eigen::Vector3d corr_odom = (self_pos_ + delta_filter_) - Eigen::Vector3d(0, 0, avg_hei);
    fuse_result(corr_odom);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uwb_correction");
    ros::NodeHandle nh("~");
    nh.param("self_id", self_id_, 0);
    nh.param("max_odom_uwb_dist_error", max_error_, 1.0);
    nh.param("min_odom_distance", min_distance_, 0.1);
    nh.param("filter_factor", filter_factor_, 0.001);
    nh.param("max_drift_change_rate", max_step_, 0.01);
    nh.param("fixed_anchor_num", fixed_anchor_num_, 0);
    nh.param("optimize_z", optimize_z_, false);

    for (int i = 0; i < fixed_anchor_num_; ++i)
    {
        std::vector<double> id_and_pos;
        nh.getParam("fixed_anchor_id_pos_" + to_string(i), id_and_pos);
        fixed_anchor_pos_[round(id_and_pos[0])] << id_and_pos[1], id_and_pos[2], id_and_pos[3];
    }

    in_filter_factor_ = 1 - filter_factor_;
    delta_filter_ << 0, 0, 0;

    uwb_info_sub_ = nh.subscribe("nlink_linktrack_nodeframe3", 10, uwb_info_cb, ros::TransportHints().tcpNoDelay());
    self_odom_sub_ = nh.subscribe("my_odom", 10, self_odom_cb, ros::TransportHints().tcpNoDelay());
    other_odom_sub_ = nh.subscribe("/others_odom", 10, other_odom_cb, ros::TransportHints().tcpNoDelay());
    ground_height_sub_ = nh.subscribe("/ground_height_measurement", 10, ground_height_cb);

    correction_odom_pub_ = nh.advertise<nav_msgs::Odometry>("corr_odom", 100);
    drift_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("drift", 100);

    ros::Rate rate(100);
    int count = 0;
    while (ros::ok())
    {
        if (count > 1000) // 10s
        {

            if (self_odom_count_ < 90)
            {
                ROS_ERROR("my_odom frequency < 9Hz, too low!");
            }
            if (other_odom_count_ < 90)
            {
                ROS_ERROR("others_odom frequency < 9Hz, too low!");
            }
            if (uwb_count_ < 90)
            {
                ROS_ERROR("uwb frequency < 9Hz, too low!");
            }

            count = 0;
            self_odom_count_ = 0;
            other_odom_count_ = 0;
            uwb_count_ = 0;
        }

        count++;
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
