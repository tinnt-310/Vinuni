#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>

// FL, FR, RL, RR. Geometry is loaded from the same YAML as the Xacro.
class SwerveController : public rclcpp::Node {
public:
  SwerveController() : Node("swerve_controller") {
    const double length = declare_parameter("wheelbase", 0.80);
    const double width = declare_parameter("track_width", 0.70);
    radius_ = declare_parameter("wheel_radius", 0.210);
    timeout_ = declare_parameter("cmd_vel_timeout", 0.5);
    const auto topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    if (length <= 0 || width <= 0 || radius_ <= 0 || timeout_ <= 0)
      throw std::invalid_argument("Positive wheel geometry and command timeout required");
    xs_ = {length/2, length/2, -length/2, -length/2};
    ys_ = {width/2, -width/2, width/2, -width/2};
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(topic, 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
            !std::isfinite(msg->angular.z)) return;
        cmd_ = *msg; cmd_time_ = now(); have_cmd_ = true;
      });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::array<double,4> angles{}, rates{};
        for (size_t i=0; i<4; ++i) {
          const auto n = std::to_string(i+1);
          const auto s = std::find(msg->name.begin(), msg->name.end(), "Revolute"+n+"_to_base_link");
          const auto d = std::find(msg->name.begin(), msg->name.end(), "Wheel"+n+"_to_Revolute"+n);
          if (s == msg->name.end() || d == msg->name.end()) return;
          const size_t si=std::distance(msg->name.begin(), s), di=std::distance(msg->name.begin(), d);
          if (si>=msg->position.size() || di>=msg->velocity.size()) return;
          angles[i]=msg->position[si]; rates[i]=msg->velocity[di];
          if (!std::isfinite(angles[i]) || !std::isfinite(rates[i])) return;
        }
        angles_=angles; rates_=rates; state_time_=now(); have_state_=true;
      });
    steer_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/steering_controller/commands",10);
    drive_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/drive_controller/commands",10);
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/odom",10);
    broadcaster_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    last_time_=now();
    timer_=create_wall_timer(std::chrono::milliseconds(20), [this] { update(); });
    RCLCPP_INFO(get_logger(),"Swerve L=%.3f W=%.3f r=%.3f, input=%s, wheel-feedback odometry",
      length,width,radius_,topic.c_str());
  }
private:
  void update() {
    auto stamp=now(); const double dt=(stamp-last_time_).seconds(); last_time_=stamp;
    if (dt<=0) return;
    const bool fresh=have_state_ && (stamp-state_time_).seconds()<timeout_;
    const bool command=have_cmd_ && (stamp-cmd_time_).seconds()>=0 &&
      (stamp-cmd_time_).seconds()<timeout_ && fresh;
    std_msgs::msg::Float64MultiArray steering, driving;
    for(size_t i=0;i<4;++i) {
      const double vx=command ? cmd_.linear.x-cmd_.angular.z*ys_[i] : 0;
      const double vy=command ? cmd_.linear.y+cmd_.angular.z*xs_[i] : 0;
      double speed=std::hypot(vx,vy);
      if(speed>1e-6) {
        double delta=std::remainder(std::atan2(vy,vx)-angles_[i],2*M_PI);
        if(std::abs(delta)>M_PI/2) { delta-=std::copysign(M_PI,delta); speed=-speed; }
        targets_[i]=angles_[i]+delta;
        // Avoid driving across the desired path during steering alignment.
        speed*=std::max(0.0,std::cos(delta));
      }
      steering.data.push_back(targets_[i]);
      driving.data.push_back(speed/radius_);
    }
    // Preserve the velocity vector if a requested wheel rate exceeds its interface limit.
    double max_rate=0; for(double v:driving.data) max_rate=std::max(max_rate,std::abs(v));
    if(max_rate>10) for(double &v:driving.data) v*=10/max_rate;
    steer_pub_->publish(steering); drive_pub_->publish(driving);
    double vx=0,vy=0,wz=0,den=0;
    if(fresh) for(size_t i=0;i<4;++i) {
      const double wx=radius_*rates_[i]*std::cos(angles_[i]);
      const double wy=radius_*rates_[i]*std::sin(angles_[i]);
      vx+=wx/4;vy+=wy/4;wz+=-ys_[i]*wx+xs_[i]*wy;
      den+=xs_[i]*xs_[i]+ys_[i]*ys_[i];
    }
    if(den>0) wz/=den;
    // Never fabricate motion from commands when wheels are stopped or feedback is missing.
    if(dt<0.5) {
      const double mid=yaw_+wz*dt/2;
      x_+=(vx*std::cos(mid)-vy*std::sin(mid))*dt;
      y_+=(vx*std::sin(mid)+vy*std::cos(mid))*dt;
      yaw_=std::remainder(yaw_+wz*dt,2*M_PI);
    }
    tf2::Quaternion q; q.setRPY(0,0,yaw_);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp=stamp;odom.header.frame_id="odom";odom.child_frame_id="base_footprint";
    odom.pose.pose.position.x=x_;odom.pose.pose.position.y=y_;
    odom.pose.pose.orientation.x=q.x();odom.pose.pose.orientation.y=q.y();
    odom.pose.pose.orientation.z=q.z();odom.pose.pose.orientation.w=q.w();
    odom.twist.twist.linear.x=vx;odom.twist.twist.linear.y=vy;odom.twist.twist.angular.z=wz;
    odom.pose.covariance[0]=odom.pose.covariance[7]=odom.pose.covariance[35]=0.03;
    odom.twist.covariance[0]=odom.twist.covariance[7]=odom.twist.covariance[35]=0.03;
    odom_pub_->publish(odom);
    geometry_msgs::msg::TransformStamped tf;
    tf.header=odom.header;tf.child_frame_id=odom.child_frame_id;
    tf.transform.translation.x=x_;tf.transform.translation.y=y_;tf.transform.rotation=odom.pose.pose.orientation;
    broadcaster_->sendTransform(tf);
  }
  double radius_,timeout_,x_=0,y_=0,yaw_=0;
  std::array<double,4> xs_{},ys_{},angles_{},rates_{},targets_{};
  bool have_state_=false,have_cmd_=false;
  geometry_msgs::msg::Twist cmd_;
  rclcpp::Time last_time_{0,0,RCL_ROS_TIME},cmd_time_{0,0,RCL_ROS_TIME},state_time_{0,0,RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_,drive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<SwerveController>());rclcpp::shutdown();return 0;
}
