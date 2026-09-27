#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <fcntl.h>
#include <map>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class MissionManagerNode : public rclcpp::Node
{
public:
  using Navigate = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;

  explicit MissionManagerNode(const rclcpp::NodeOptions & options)
  : Node("mission_manager", options)
  {
    if (!has_parameter("auto_route")) declare_parameter<std::vector<std::string>>("auto_route", std::vector<std::string>{});
    if (!has_parameter("pause_between_stations_sec")) declare_parameter("pause_between_stations_sec", 1.0);
    if (!has_parameter("goal_timeout_sec")) declare_parameter("goal_timeout_sec", 180.0);
    if (!has_parameter("max_retries")) declare_parameter("max_retries", 1);
    route_ = get_parameter("auto_route").as_string_array();
    pause_sec_ = get_parameter("pause_between_stations_sec").as_double();
    timeout_sec_ = get_parameter("goal_timeout_sec").as_double();
    max_retries_ = get_parameter("max_retries").as_int();
    if (!std::isfinite(pause_sec_) || pause_sec_ < 0 ||
        !std::isfinite(timeout_sec_) || timeout_sec_ <= 0 ||
        max_retries_ < 0) {
      throw std::invalid_argument("Invalid mission pause, timeout or retry count");
    }
    load_stations();
    for (const auto & name : route_) {
      if (!stations_.count(name)) throw std::invalid_argument("Unknown station in auto_route: " + name);
    }
    nav_client_ = rclcpp_action::create_client<Navigate>(this, "navigate_to_pose");
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission_command", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { handle_command(msg->data); });
    watchdog_ = create_wall_timer(200ms, [this]() { on_watchdog(); });

    // No detached console thread: stdin is polled by a mutually exclusive ROS callback.
    stdin_flags_ = fcntl(STDIN_FILENO, F_GETFL);
    if (stdin_flags_ >= 0 && fcntl(STDIN_FILENO, F_SETFL, stdin_flags_ | O_NONBLOCK) == 0) {
      console_timer_ = create_wall_timer(100ms, [this]() { read_console(); });
      if (isatty(STDIN_FILENO)) print_menu();
    }
    RCLCPP_INFO(get_logger(), "Mission manager ready: %zu stations, %zu route legs; commands: <station>, auto, cancel, list",
      stations_.size(), route_.size());
  }

  ~MissionManagerNode() override
  {
    if (stdin_flags_ >= 0) fcntl(STDIN_FILENO, F_SETFL, stdin_flags_);
  }

private:
  struct Pose2D { double x{}, y{}, yaw{}; };
  enum class State {IDLE, WAIT_SERVER, WAIT_GOAL, NAVIGATING, CANCELLING, PAUSE};
  std::map<std::string, Pose2D> stations_;
  std::vector<std::string> route_;
  rclcpp_action::Client<Navigate>::SharedPtr nav_client_;
  GoalHandle::SharedPtr active_goal_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_, pause_timer_, console_timer_;
  State state_{State::IDLE};
  std::string current_station_, queued_station_, console_buffer_;
  bool queued_auto_{false}, auto_mode_{false}, manual_cancel_{false}, cancel_on_accept_{false};
  size_t route_index_{0};
  int retry_count_{0}, max_retries_{1}, stdin_flags_{-1};
  uint64_t goal_sequence_{0};
  double pause_sec_{1.0}, timeout_sec_{180.0};
  std::chrono::steady_clock::time_point goal_start_{};

  void load_stations()
  {
    const auto names = list_parameters({"stations"}, 100).names;
    std::map<std::string, unsigned int> fields;
    for (const auto & full_name : names) {
      if (full_name.rfind("stations.", 0) != 0) continue;
      const auto key = full_name.substr(9);
      const auto parameter = get_parameter(full_name);
      const auto dot = key.find('.');
      if (dot == std::string::npos) throw std::invalid_argument("Malformed station parameter: " + key);
      const auto name = key.substr(0, dot);
      const auto field = key.substr(dot + 1);
      const double value = parameter.as_double();
      if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite station coordinate: " + key);
      auto & pose = stations_[name];
      if (field == "x") {pose.x = value; fields[name] |= 1U;}
      else if (field == "y") {pose.y = value; fields[name] |= 2U;}
      else if (field == "yaw") {pose.yaw = value; fields[name] |= 4U;}
      else throw std::invalid_argument("Unknown station field: " + key);
    }
    if (stations_.empty()) throw std::invalid_argument("No stations loaded from ROS parameters");
    for (const auto & [name, bits] : fields) {
      if (bits != 7U) throw std::invalid_argument("Station needs x, y and yaw: " + name);
    }
  }

  void print_menu() const
  {
    for (const auto & [name, pose] : stations_) {
      RCLCPP_INFO(get_logger(), "%s: x=%.2f y=%.2f yaw=%.2f", name.c_str(), pose.x, pose.y, pose.yaw);
    }
  }

  void read_console()
  {
    char data[128];
    const ssize_t count = read(STDIN_FILENO, data, sizeof(data));
    if (count == 0) {
      console_timer_->cancel();
      return;
    }
    if (count < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) console_timer_->cancel();
      return;
    }
    for (ssize_t i = 0; i < count; ++i) {
      if (data[i] == '\n') {
        handle_command(console_buffer_);
        console_buffer_.clear();
      } else if (console_buffer_.size() < 256) {
        console_buffer_.push_back(data[i]);
      }
    }
  }

  void handle_command(const std::string & command)
  {
    if (command == "list") {print_menu(); return;}
    if (command == "cancel") {
      queued_auto_ = false;
      queued_station_.clear();
      auto_mode_ = false;
      manual_cancel_ = true;
      if (pause_timer_) pause_timer_->cancel();
      if (state_ == State::WAIT_SERVER || state_ == State::PAUSE) state_ = State::IDLE;
      request_cancel();
      return;
    }
    const bool auto_request = command == "auto";
    if (auto_request && route_.empty()) {
      RCLCPP_WARN(get_logger(), "auto_route is empty");
      return;
    }
    if (!auto_request && !stations_.count(command)) {
      RCLCPP_WARN(get_logger(), "Unknown mission command: %s", command.c_str());
      return;
    }
    queued_auto_ = auto_request;
    queued_station_ = auto_request ? "" : command;
    manual_cancel_ = false;
    auto_mode_ = false;
    if (pause_timer_) pause_timer_->cancel();
    if (state_ == State::WAIT_SERVER || state_ == State::PAUSE || state_ == State::IDLE) {
      state_ = State::IDLE;
      dispatch_queued();
    } else {
      request_cancel();
    }
  }

  void dispatch_queued()
  {
    if (queued_auto_) {
      queued_auto_ = false;
      queued_station_.clear();
      auto_mode_ = true;
      route_index_ = 0;
      retry_count_ = 0;
      send_goal(route_.front());
    } else if (!queued_station_.empty()) {
      auto name = queued_station_;
      queued_station_.clear();
      auto_mode_ = false;
      retry_count_ = 0;
      send_goal(name);
    }
  }

  void send_goal(const std::string & name)
  {
    current_station_ = name;
    if (!nav_client_->action_server_is_ready()) {
      state_ = State::WAIT_SERVER;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for Nav2 action server");
      return;
    }
    Navigate::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    const auto & target = stations_.at(name);
    goal.pose.pose.position.x = target.x;
    goal.pose.pose.position.y = target.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, target.yaw);
    goal.pose.pose.orientation.x = q.x();
    goal.pose.pose.orientation.y = q.y();
    goal.pose.pose.orientation.z = q.z();
    goal.pose.pose.orientation.w = q.w();
    const auto sequence = ++goal_sequence_;
    state_ = State::WAIT_GOAL;
    cancel_on_accept_ = false;
    goal_start_ = std::chrono::steady_clock::now();
    auto options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    options.goal_response_callback = [this, sequence](GoalHandle::SharedPtr handle) {
      if (sequence != goal_sequence_) return;
      if (!handle) {
        RCLCPP_WARN(get_logger(), "Nav2 rejected station %s", current_station_.c_str());
        finish_goal(false);
        return;
      }
      active_goal_ = handle;
      state_ = State::NAVIGATING;
      goal_start_ = std::chrono::steady_clock::now();
      if (cancel_on_accept_) request_cancel();
    };
    options.result_callback = [this, sequence](const GoalHandle::WrappedResult & result) {
      if (sequence != goal_sequence_) return;
      active_goal_.reset();
      finish_goal(result.code == rclcpp_action::ResultCode::SUCCEEDED);
    };
    nav_client_->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(), "Navigating to %s (attempt %d)", name.c_str(), retry_count_ + 1);
  }

  void request_cancel()
  {
    if (state_ == State::WAIT_GOAL) {
      cancel_on_accept_ = true;
    } else if (state_ == State::NAVIGATING && active_goal_) {
      state_ = State::CANCELLING;
      nav_client_->async_cancel_goal(active_goal_);
    }
  }

  void finish_goal(bool succeeded)
  {
    state_ = State::IDLE;
    cancel_on_accept_ = false;
    if (queued_auto_ || !queued_station_.empty()) {
      dispatch_queued();
      return;
    }
    if (manual_cancel_) {
      manual_cancel_ = false;
      RCLCPP_INFO(get_logger(), "Mission canceled");
      return;
    }
    if (succeeded) {
      RCLCPP_INFO(get_logger(), "Reached %s", current_station_.c_str());
      if (auto_mode_ && ++route_index_ < route_.size()) {
        state_ = State::PAUSE;
        const auto delay = std::chrono::duration<double>(pause_sec_);
        pause_timer_ = create_wall_timer(delay, [this]() {
          pause_timer_->cancel();
          retry_count_ = 0;
          send_goal(route_[route_index_]);
        });
      } else {
        auto_mode_ = false;
      }
      return;
    }
    if (retry_count_ < max_retries_) {
      ++retry_count_;
      RCLCPP_WARN(get_logger(), "Retrying %s", current_station_.c_str());
      send_goal(current_station_);
    } else {
      RCLCPP_ERROR(get_logger(), "Mission stopped at %s after %d attempts",
        current_station_.c_str(), retry_count_ + 1);
      auto_mode_ = false;
    }
  }

  void on_watchdog()
  {
    if (state_ == State::WAIT_SERVER && nav_client_->action_server_is_ready()) {
      send_goal(current_station_);
    }
    if ((state_ == State::NAVIGATING || state_ == State::WAIT_GOAL) &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_start_).count() > timeout_sec_) {
      RCLCPP_ERROR(get_logger(), "Goal timeout for %s; requesting cancellation", current_station_.c_str());
      request_cancel();
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<MissionManagerNode>(options);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "Mission manager startup failed: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
