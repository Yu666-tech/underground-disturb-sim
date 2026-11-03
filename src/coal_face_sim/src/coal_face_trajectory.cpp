// Prescribed kinematic motion for the simplified coal-face body.
// Evaluate in the physics update thread; never queue ROS pose-service commands.
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace coal_face_sim
{
class CoalFaceTrajectory : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);
    y_min_ = sdf->Get<double>("y_min", 5.0).first;
    y_max_ = sdf->Get<double>("y_max", 195.0).first;
    period_ = sdf->Get<double>("period", 1200.0).first;
    x_ = sdf->Get<double>("x", 0.6).first;
    z_ = sdf->Get<double>("z", 0.6605).first;
    rate_ = sdf->Get<double>("reference_rate", 50.0).first;
    for (double v : {y_min_, y_max_, period_, x_, z_, rate_}) {
      if (!std::isfinite(v)) {throw std::runtime_error("Non-finite trajectory parameter");}
    }
    if (y_min_ < 5.0 || y_max_ > 195.0 || y_min_ >= y_max_ || period_ <= 0 ||
      rate_ <= 0 || std::abs(x_ - 0.6) > 1e-6 || std::abs(z_ - 0.6605) > 1e-6)
    {
      throw std::runtime_error("Trajectory exceeds validated coal-face envelope");
    }
    reference_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/shearer/reference", rclcpp::QoS(100));
    Reset();
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&CoalFaceTrajectory::OnUpdate, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(),
      "Physics-step trajectory active: y=%.1f..%.1f m, period=%.1f s; no pose-service queue",
      y_min_, y_max_, period_);
  }

  void Reset() override
  {
    epoch_ = model_->GetWorld()->SimTime();
    last_time_ = epoch_;
    last_publish_ = epoch_;
    Apply(0.0);
  }

private:
  void Apply(double elapsed)
  {
    const double omega = 2.0 * std::acos(-1.0) / period_;
    const double phase = omega * std::fmod(elapsed, period_);
    y_ = (y_min_ + y_max_) * 0.5 - (y_max_ - y_min_) * 0.5 * std::cos(phase);
    vy_ = (y_max_ - y_min_) * 0.5 * omega * std::sin(phase);
    model_->SetWorldPose(ignition::math::Pose3d(x_, y_, z_, 0, 0, 0));
    model_->SetLinearVel(ignition::math::Vector3d(0, vy_, 0));
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    if (info.simTime < last_time_) {Reset();}
    last_time_ = info.simTime;
    Apply((info.simTime - epoch_).Double());
    // Physics control runs every step; only reference telemetry is rate limited.
    if ((info.simTime - last_publish_).Double() + 1e-10 < 1.0 / rate_) {return;}
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(info.simTime);
    msg.header.frame_id = "world";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = z_;
    msg.pose.pose.orientation.w = 1.0;
    msg.twist.twist.linear.y = vy_;
    reference_->publish(msg);
    last_publish_ = info.simTime;
  }

  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reference_;
  gazebo::common::Time epoch_, last_time_, last_publish_;
  double y_min_, y_max_, period_, x_, z_, rate_, y_{5.0}, vy_{0.0};
};
GZ_REGISTER_MODEL_PLUGIN(CoalFaceTrajectory)
}  // namespace coal_face_sim
