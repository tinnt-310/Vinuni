#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <map>
#include <thread>
#include <vector>
#include <iostream>
#include <string>

struct Pose2D { double x, y, yaw; };

class AutoDockingNode : public rclcpp::Node {
public:
    using Nav2Pose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<Nav2Pose>;

    explicit AutoDockingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
    : Node("auto_docking_node", options) {
        
        nav_client_ = rclcpp_action::create_client<Nav2Pose>(this, "navigate_to_pose");

        // Tọa độ mục tiêu các trạm (x, y, yaw)
        stations_["0"] = {0.0, 0.0, 0.0};     // Home / Start
        stations_["1"] = {8.5, -0.2, 0.0};  // Trạm 1
        stations_["2"] = {8.5, 4.0, 0.0};   // Trạm 2
        stations_["3"] = {0.0, -8.0, 3.14};   // Trạm 3

        auto_route_ = {"1", "2", "3", "0"};

        // Thread Console Menu
        std::thread([this]() {
            while (rclcpp::ok()) {
                std::string input;
                std::cout << "\n========== EV NAV2 CONTROLLER ==========\n";
                std::cout << "  Nhập (1, 2, 3, 0) : Di chuyển trạm đơn\n";
                std::cout << "  Nhập 'a'         : Chạy AUTO (Start -> 1 -> 2 -> 3 -> Start)\n";
                std::cout << "========================================\nLệnh: ";
                std::cin >> input;

                if (state_ != "IDLE") {
                    std::cout << "[CẢNH BÁO] Xe đang bận chạy nhiệm vụ!\n";
                    continue;
                }

                if (input == "a" || input == "A") {
                    is_auto_mode_ = true;
                    auto_step_ = 0;
                    auto_start_time_ = this->get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "=== BẮT ĐẦU AUTO MISSION LOOP ===");
                    execute_next_auto_step();
                } else if (stations_.find(input) != stations_.end()) {
                    is_auto_mode_ = false;
                    start_mission(stations_[input], input);
                } else {
                    std::cout << "[LỖI] Trạm không hợp lệ!\n";
                }
            }
        }).detach();
    }

private:
    void execute_next_auto_step() {
        if (auto_step_ >= (int)auto_route_.size()) {
            double total_time = (this->get_clock()->now() - auto_start_time_).seconds();
            RCLCPP_INFO(this->get_logger(), "==========================================");
            RCLCPP_INFO(this->get_logger(), "=== AUTO MISSION HOÀN THÀNH XUẤT SẮC! ===");
            RCLCPP_INFO(this->get_logger(), "⏱️ TỔNG THỜI GIAN TOÀN VÒNG: %.2f GIÂY", total_time);
            RCLCPP_INFO(this->get_logger(), "==========================================");
            state_ = "IDLE";
            is_auto_mode_ = false;
            return;
        }

        std::string st_id = auto_route_[auto_step_];
        std::string label = (st_id == "0") ? "Start" : ("Trạm " + st_id);
        RCLCPP_INFO(this->get_logger(), "[AUTO %d/%lu] Di chuyển tới %s...", 
                    auto_step_ + 1, auto_route_.size(), label.c_str());
        start_mission(stations_[st_id], st_id);
    }

    void start_mission(const Pose2D& target_pose, const std::string& station_id) {
        state_ = "NAV2";
        current_station_id_ = station_id;
        leg_start_time_ = this->get_clock()->now();

        auto goal_msg = Nav2Pose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = target_pose.x;
        goal_msg.pose.pose.position.y = target_pose.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, target_pose.yaw); 
        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();

        auto send_goal_options = rclcpp_action::Client<Nav2Pose>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&AutoDockingNode::nav2_goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback = 
            std::bind(&AutoDockingNode::nav2_result_callback, this, std::placeholders::_1);
        
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void nav2_goal_response_callback(const GoalHandleNav2::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 TỪ CHỐI lệnh!");
            state_ = "IDLE";
            is_auto_mode_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Nav2 CHẤP NHẬN. Đang di chuyển...");
        }
    }

    void nav2_result_callback(const GoalHandleNav2::WrappedResult & result) {
        double leg_time = (this->get_clock()->now() - leg_start_time_).seconds();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            std::string label = (current_station_id_ == "0") ? "Start" : ("Trạm " + current_station_id_);
            RCLCPP_INFO(this->get_logger(), "✓ ĐÃ TỚI %s! ⏱️ Thời gian chặng: %.2f giây", label.c_str(), leg_time);

            if (is_auto_mode_) {
                state_ = "PAUSE";
                // Tạm dừng 1 giây giữa các trạm trước khi xuất phát chặng tiếp theo
                pause_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
                    pause_timer_->cancel();
                    auto_step_++;
                    execute_next_auto_step();
                });
            } else {
                state_ = "IDLE";
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Nav2 LỖI HOẶC BỊ HỦY! Dừng nhiệm vụ.");
            state_ = "IDLE";
            is_auto_mode_ = false;
        }
    }

    std::string state_ = "IDLE";
    bool is_auto_mode_ = false;
    std::string current_station_id_;
    std::map<std::string, Pose2D> stations_; 
    std::vector<std::string> auto_route_;
    int auto_step_ = 0;

    rclcpp::Time leg_start_time_;
    rclcpp::Time auto_start_time_;

    rclcpp_action::Client<Nav2Pose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr pause_timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"use_sim_time", true}});
    
    auto node = std::make_shared<AutoDockingNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}