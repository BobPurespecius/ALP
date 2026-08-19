#include <plan_env/obj_predictor.h>

#include <algorithm>
#include <boost/bind.hpp>
#include <cmath>
#include <limits>
#include <string>

namespace fast_planner
{

bool PolynomialPrediction::valid() const
{
  return samples_.size() >= 2;
}

void PolynomialPrediction::setPredictionPath(const nav_msgs::Path &path)
{
  samples_.clear();
  samples_.reserve(path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i)
  {
    const geometry_msgs::PoseStamped &pose = path.poses[i];
    const ros::Time stamp = pose.header.stamp.isZero()
                                ? path.header.stamp + ros::Duration(0.1 * static_cast<double>(i))
                                : pose.header.stamp;
    Sample sample;
    sample.time = stamp.toSec();
    sample.position << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
    samples_.push_back(sample);
  }

  std::vector<Sample> monotonic_samples;
  monotonic_samples.reserve(samples_.size());
  for (const Sample &sample : samples_)
  {
    if (monotonic_samples.empty() || sample.time > monotonic_samples.back().time + 1.0e-6)
      monotonic_samples.push_back(sample);
  }
  samples_.swap(monotonic_samples);
  if (samples_.size() < 2)
    return;

  for (size_t i = 0; i < samples_.size(); ++i)
  {
    if (i == 0)
      samples_[i].velocity = (samples_[1].position - samples_[0].position) /
                             std::max(1.0e-3, samples_[1].time - samples_[0].time);
    else if (i + 1 == samples_.size())
      samples_[i].velocity = (samples_[i].position - samples_[i - 1].position) /
                             std::max(1.0e-3, samples_[i].time - samples_[i - 1].time);
    else
      samples_[i].velocity = (samples_[i + 1].position - samples_[i - 1].position) /
                             std::max(1.0e-3, samples_[i + 1].time - samples_[i - 1].time);
  }
}

Eigen::Vector3d PolynomialPrediction::evaluateConstVel(double time) const
{
  if (!valid())
  {
    const double max_value = std::numeric_limits<double>::max();
    return Eigen::Vector3d(max_value, max_value, max_value);
  }

  if (time <= samples_.front().time)
    return samples_.front().position;
  if (time >= samples_.back().time)
    return samples_.back().position;
  for (size_t i = 1; i < samples_.size(); ++i)
  {
    if (time <= samples_[i].time)
    {
      const double dt = std::max(1.0e-3, samples_[i].time - samples_[i - 1].time);
      const double alpha = (time - samples_[i - 1].time) / dt;
      return (1.0 - alpha) * samples_[i - 1].position + alpha * samples_[i].position;
    }
  }
  return samples_.back().position;
}

Eigen::Vector3d PolynomialPrediction::evaluateConstVelVelocity(double time) const
{
  if (!valid())
    return Eigen::Vector3d::Zero();

  if (time <= samples_.front().time)
    return samples_.front().velocity;
  if (time >= samples_.back().time)
    return samples_.back().velocity;
  for (size_t i = 1; i < samples_.size(); ++i)
  {
    if (time <= samples_[i].time)
    {
      const double dt = std::max(1.0e-3, samples_[i].time - samples_[i - 1].time);
      const double alpha = (time - samples_[i - 1].time) / dt;
      return (1.0 - alpha) * samples_[i - 1].velocity + alpha * samples_[i].velocity;
    }
  }
  return samples_.back().velocity;
}

void ObjHistory::init(int id, int skip_num, int queue_size, ros::Time global_start_time)
{
  clear();
  skip_ = 0;
  obj_idx_ = id;
  skip_num_ = std::max(1, skip_num);
  queue_size_ = std::max(2, queue_size);
  global_start_time_ = global_start_time;
}

void ObjHistory::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  ++skip_;
  if (skip_ < skip_num_)
    return;

  Eigen::Vector4d pos_t;
  pos_t(0) = msg->pose.position.x;
  pos_t(1) = msg->pose.position.y;
  pos_t(2) = msg->pose.position.z;
  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  pos_t(3) = stamp.toSec();

  history_.push_back(pos_t);
  while ((int)history_.size() > queue_size_)
    history_.pop_front();

  skip_ = 0;
}

void ObjHistory::clear()
{
  history_.clear();
}

void ObjHistory::getHistory(std::list<Eigen::Vector4d> &history) const
{
  history = history_;
}

ObjPredictor::ObjPredictor(ros::NodeHandle &node) : node_handle_(node)
{
}

void ObjPredictor::init()
{
  int queue_size, skip_nums;

  node_handle_.param("prediction/obj_num", obj_num_, 0);
  node_handle_.param("prediction/lambda", lambda_, 1.0);
  node_handle_.param("prediction/predict_rate", predict_rate_, 1.0);
  node_handle_.param("prediction/queue_size", queue_size, 10);
  node_handle_.param("prediction/skip_nums", skip_nums, 1);

  ROS_INFO("[moving_obj] predictor init: obj_num=%d, predict_rate=%.2f, queue_size=%d, skip_nums=%d",
           obj_num_, predict_rate_, queue_size, skip_nums);

  predict_trajs_.reset(new std::vector<PolynomialPrediction>);
  predict_trajs_->resize(std::max(0, obj_num_));

  obj_scale_.reset(new std::vector<Eigen::Vector3d>);
  obj_scale_->resize(std::max(0, obj_num_), Eigen::Vector3d::Ones());
  scale_init_.assign(std::max(0, obj_num_), false);

  ros::Time t_now = ros::Time::now();
  for (int i = 0; i < obj_num_; ++i)
  {
    std::shared_ptr<ObjHistory> obj_history(new ObjHistory);
    obj_history->init(i, skip_nums, queue_size, t_now);
    obj_histories_.push_back(obj_history);

    pose_subs_.push_back(node_handle_.subscribe<geometry_msgs::PoseStamped>(
        "/dynamic/pose_" + std::to_string(i), 10, &ObjHistory::poseCallback, obj_history.get()));

    prediction_subs_.push_back(node_handle_.subscribe<nav_msgs::Path>(
        "/dynamic/prediction_" + std::to_string(i), 2,
        boost::bind(&ObjPredictor::predictionPathCallback, this, _1, i)));

    ROS_INFO("[moving_obj] subscribing /dynamic/pose_%d", i);

  }

  marker_sub_ = node_handle_.subscribe<visualization_msgs::Marker>(
      "/dynamic/obj", 10, &ObjPredictor::markerCallback, this);

  const double period = 1.0 / std::max(1.0e-3, predict_rate_);
  predict_timer_ = node_handle_.createTimer(ros::Duration(period), &ObjPredictor::predictCallback, this);
}

int ObjPredictor::getObjNums() const
{
  return obj_num_;
}

bool ObjPredictor::hasPrediction(int obj_id) const
{
  return predict_trajs_ && obj_id >= 0 && obj_id < obj_num_ && predict_trajs_->at(obj_id).valid();
}

Eigen::Vector3d ObjPredictor::evaluateConstVel(int obj_id, double time) const
{
  if (!hasPrediction(obj_id))
  {
    const double max_value = std::numeric_limits<double>::max();
    return Eigen::Vector3d(max_value, max_value, max_value);
  }
  return predict_trajs_->at(obj_id).evaluateConstVel(time);
}

Eigen::Vector3d ObjPredictor::evaluateConstVelVelocity(int obj_id, double time) const
{
  if (!hasPrediction(obj_id))
    return Eigen::Vector3d::Zero();
  return predict_trajs_->at(obj_id).evaluateConstVelVelocity(time);
}

void ObjPredictor::predictionPathCallback(const nav_msgs::PathConstPtr &msg, int obj_id)
{
  if (!msg || !predict_trajs_ || obj_id < 0 || obj_id >= obj_num_)
    return;
  predict_trajs_->at(obj_id).setPredictionPath(*msg);
}

Eigen::Vector3d ObjPredictor::getObjScale(int obj_id) const
{
  if (!obj_scale_ || obj_id < 0 || obj_id >= obj_num_)
    return Eigen::Vector3d::Ones();
  return obj_scale_->at(obj_id);
}

void ObjPredictor::markerCallback(const visualization_msgs::MarkerConstPtr &msg)
{
  int idx = msg->id;
  if (!obj_scale_ || idx < 0 || idx >= obj_num_)
    return;

  (*obj_scale_)[idx](0) = msg->scale.x;
  (*obj_scale_)[idx](1) = msg->scale.y;
  (*obj_scale_)[idx](2) = msg->scale.z;
  scale_init_[idx] = true;

  int finish_num = 0;
  for (bool initialized : scale_init_)
  {
    if (initialized)
      ++finish_num;
  }
  if (finish_num == obj_num_)
    marker_sub_.shutdown();
}

void ObjPredictor::predictCallback(const ros::TimerEvent &)
{
  predictConstVel();
}

void ObjPredictor::predictConstVel()
{
  int history_ready_num = 0;
  int valid_prediction_num = 0;

  for (int i = 0; i < obj_num_; ++i)
  {
    if (predict_trajs_->at(i).valid())
    {
      ++valid_prediction_num;
      continue;
    }

    std::list<Eigen::Vector4d> history;
    obj_histories_[i]->getHistory(history);
    if (history.size() < 2)
      continue;
    ++history_ready_num;

    auto q2_it = history.end();
    --q2_it;
    auto q1_it = q2_it;
    --q1_it;

    const Eigen::Vector3d q1 = q1_it->head(3);
    const Eigen::Vector3d q2 = q2_it->head(3);
    const double t1 = (*q1_it)(3);
    const double t2 = (*q2_it)(3);
    if (std::fabs(t2 - t1) < 1.0e-4)
      continue;

    nav_msgs::Path fallback;
    fallback.header.frame_id = "world";
    geometry_msgs::PoseStamped pose1, pose2, pose3;
    pose1.header.stamp.fromSec(t1);
    pose2.header.stamp.fromSec(t2);
    const double dt = std::max(1.0e-3, t2 - t1);
    const Eigen::Vector3d velocity = (q2 - q1) / dt;
    pose3.header.stamp.fromSec(t2 + 10.0);
    pose1.pose.position.x = q1.x();
    pose1.pose.position.y = q1.y();
    pose1.pose.position.z = q1.z();
    pose2.pose.position.x = q2.x();
    pose2.pose.position.y = q2.y();
    pose2.pose.position.z = q2.z();
    const Eigen::Vector3d q3 = q2 + 10.0 * velocity;
    pose3.pose.position.x = q3.x();
    pose3.pose.position.y = q3.y();
    pose3.pose.position.z = q3.z();
    fallback.poses.push_back(pose1);
    fallback.poses.push_back(pose2);
    fallback.poses.push_back(pose3);
    predict_trajs_->at(i).setPredictionPath(fallback);
    if (predict_trajs_->at(i).valid())
      ++valid_prediction_num;
  }

  ROS_INFO_THROTTLE(2.0,
                    "[moving_obj] predictor status: pose_histories=%d/%d, valid_predictions=%d/%d",
                    history_ready_num, obj_num_, valid_prediction_num, obj_num_);
}

} // namespace fast_planner
