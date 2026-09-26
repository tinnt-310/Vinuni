#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class GoalSenderNode : public rclcpp::Node {
public:
  using Nav2Pose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<Nav2Pose>;

  explicit GoalSenderNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("interactive_goal_sender", options), is_navigating_(false) {
    presets_ = {
      {"start", {-18.0,  0.0,  0.00}},
      
      {"o1", { -7.3,  6.0,  1.57}},
      {"o2", { -2.5,  6.0,  1.57}},
      {"o3", {  2.5,  6.0,  1.57}},
      {"o4", {  7.7,  6.0,  1.57}},
      {"o5", {-13.2,  6.0,  1.57}},
      
      {"o6", {-12.0, -6.7, -1.57}},
      {"o7", { -7.5, -6.7, -1.57}},
      {"o8", { -2.5, -6.7, -1.57}},
      {"o9", {  7.5, -6.7, -1.57}}
    };

    nav_client_ = rclcpp_action::create_client<Nav2Pose>(this, "navigate_to_pose");

    input_thread_ = std::thread(&GoalSenderNode::user_input_thread, this);
  }

  ~GoalSenderNode() {
    if (input_thread_.joinable()) {
      input_thread_.detach(); // Ngắt luồng an toàn khi node bị hủy
    }
  }

private:
  struct Pose2D { double x, y, yaw; };

  std::map<std::string, Pose2D> presets_;
  rclcpp_action::Client<Nav2Pose>::SharedPtr nav_client_;
  
  std::thread input_thread_;
  std::atomic<bool> is_navigating_; // Thay thế string state_ để chống Race Condition
  std::mutex print_mutex_;          // Khóa chống in đè terminal giữa log và bàn phím

  void user_input_thread() {
    RCLCPP_INFO(this->get_logger(), "⏳ Đang chờ kết nối tới Nav2 Action Server...");
    while (rclcpp::ok() && !nav_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "⚠️ Nav2 chưa sẵn sàng, đang thử lại...");
    }
    RCLCPP_INFO(this->get_logger(), "✅ Đã kết nối thành công với Nav2!");

    print_menu();

    while (rclcpp::ok()) {
      std::string input;
      {
        std::lock_guard<std::mutex> lock(print_mutex_);
        std::cout << "\n[ĐIỀU KHIỂN] Nhập mã trạm (VD: c1_o1), 'list' xem DS, 'quit' thoát: ";
      }
      
      std::cin >> input;

      if (!rclcpp::ok()) break;

      if (input == "quit" || input == "exit") {
        RCLCPP_INFO(this->get_logger(), "👋 Đang thoát chương trình...");
        rclcpp::shutdown();
        break;
      }

      if (input == "list" || input == "ls") {
        print_menu();
        continue;
      }

      auto it = presets_.find(input);
      if (it == presets_.end()) {
        std::lock_guard<std::mutex> lock(print_mutex_);
        std::cout << "[LỖI] Không tìm thấy trạm '" << input << "'!\n";
        continue;
      }

      // Kiểm tra trạng thái an toàn đa luồng
      if (is_navigating_.load()) {
        std::lock_guard<std::mutex> lock(print_mutex_);
        std::cout << "[CẢNH BÁO] Xe đang di chuyển! Hãy đợi hoàn thành goal hiện tại.\n";
        continue;
      }

      send_goal(input, it->second);
    }
  }

  void send_goal(const std::string &name, const Pose2D &pose) {
    auto goal_msg = Nav2Pose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.pose.position.x = pose.x;
    goal_msg.pose.pose.position.y = pose.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, pose.yaw);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();

    {
      std::lock_guard<std::mutex> lock(print_mutex_);
      RCLCPP_INFO(this->get_logger(), "🎯 Ra lệnh đi tới [%s]: X=%.2f, Y=%.2f, Yaw=%.2f rad", 
                  name.c_str(), pose.x, pose.y, pose.yaw);
    }

    is_navigating_.store(true); // Khóa trạng thái

    auto send_goal_options = rclcpp_action::Client<Nav2Pose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        std::bind(&GoalSenderNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(&GoalSenderNode::result_callback, this, std::placeholders::_1);

    nav_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleNav2::SharedPtr &goal_handle) {
    std::lock_guard<std::mutex> lock(print_mutex_);
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "❌ Goal bị Nav2 TỪ CHỐI (Có thể do kẹt vật cản)!");
      is_navigating_.store(false); // Mở khóa
    } else {
      RCLCPP_INFO(this->get_logger(), "✅ Nav2 chấp nhận lệnh. Xe đang di chuyển...");
    }
  }

  void result_callback(const GoalHandleNav2::WrappedResult &result) {
    std::lock_guard<std::mutex> lock(print_mutex_);
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "🎉 ĐÃ TỚI TRẠM THÀNH CÔNG!");
    } else {
      RCLCPP_WARN(this->get_logger(), "⚠️ Di chuyển thất bại hoặc bị HỦY!");
    }
    is_navigating_.store(false); // Mở khóa cho phép nhập lệnh tiếp
  }

  void print_menu() {
    std::lock_guard<std::mutex> lock(print_mutex_);
    std::cout << "\n================= DANH SÁCH BÃI ĐỖ =================\n";
    for (const auto &[key, val] : presets_) {
      std::cout << "  " << key << "\t→ (X: " << val.x << ",\tY: " << val.y << ",\tYaw: " << val.yaw << ")\n";
    }
    std::cout << "====================================================\n";
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_sim_time", true}});

  auto node = std::make_shared<GoalSenderNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}