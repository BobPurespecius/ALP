#ifndef _OBJ_PREDICTOR_H_
#define _OBJ_PREDICTOR_H_

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <list>
#include <memory>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>

namespace fast_planner
{

class PolynomialPrediction
{
private:
  struct Sample
  {
    double time{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  };
  std::vector<Sample> samples_;

public:
  bool valid() const;
  void setPredictionPath(const nav_msgs::Path &path);
  Eigen::Vector3d evaluateConstVel(double time) const;
  Eigen::Vector3d evaluateConstVelVelocity(double time) const;
};

class ObjHistory
{
public:
  void init(int id, int skip_num, int queue_size, ros::Time global_start_time);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void clear();
  void getHistory(std::list<Eigen::Vector4d> &history) const;

private:
  std::list<Eigen::Vector4d> history_;
  int skip_{0};
  int obj_idx_{0};
  int skip_num_{1};
  int queue_size_{10};
  ros::Time global_start_time_;
};

class ObjPredictor
{
private:
  ros::NodeHandle node_handle_;

  int obj_num_{0};
  double lambda_{1.0};
  double predict_rate_{1.0};

  std::vector<ros::Subscriber> pose_subs_;
  std::vector<ros::Subscriber> prediction_subs_;
  ros::Subscriber marker_sub_;
  ros::Timer predict_timer_;
  std::vector<std::shared_ptr<ObjHistory>> obj_histories_;
  std::shared_ptr<std::vector<PolynomialPrediction>> predict_trajs_;
  std::shared_ptr<std::vector<Eigen::Vector3d>> obj_scale_;
  std::vector<bool> scale_init_;

  void markerCallback(const visualization_msgs::MarkerConstPtr &msg);
  void predictionPathCallback(const nav_msgs::PathConstPtr &msg, int obj_id);
  void predictCallback(const ros::TimerEvent &event);
  void predictConstVel();

public:
  ObjPredictor() = default;
  explicit ObjPredictor(ros::NodeHandle &node);

  void init();
  int getObjNums() const;
  bool hasPrediction(int obj_id) const;
  Eigen::Vector3d evaluateConstVel(int obj_id, double time) const;
  Eigen::Vector3d evaluateConstVelVelocity(int obj_id, double time) const;
  Eigen::Vector3d getObjScale(int obj_id) const;

  typedef std::shared_ptr<ObjPredictor> Ptr;
};

} // namespace fast_planner

#endif
