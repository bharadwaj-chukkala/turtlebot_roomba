/**
 * @file walker.cpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-12-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "../include/Turtlebot_Roomba/walker.hpp"

Roomba::Roomba()
    : Node("walker"),
      left_dist_(0.0),
      center_dist_(0.0),
      right_dist_(0.0),
      state_(STOP) {

      // creates publisher to publish /cmd_vel topic
       auto pubTopicName = "cmd_vel";
       publisher_ = this->create_publisher<TWIST> (pubTopicName, 10);

      // creates subscriber to get/kobuki/laser/scan topic
       auto subTopicName = "/scan";
       auto subCallback = std::bind(&Roomba::laser_callback, this, _1);
       subscription_ = this->create_subscription<SCAN>
                    (subTopicName, 10, subCallback);

      // create a 10Hz timer for processing
       auto timerCallback = std::bind(&Roomba::timer_callback, this);
       timer_ = this->create_wall_timer(100ms, timerCallback);
}

void Roomba::laser_callback(const SCAN& msg) {
    scan_ = msg;
}

void Roomba::timer_callback() {
    // wait until the first scan data is read.
    if (scan_.header.stamp.sec == 0)
      return;

    auto message = TWIST();

    switch (state_) {
        case FORWARD:
        if (detect_obstacle()) {
            state_ = STOP;
            message.linear.x = 0.0;
            message.linear.y = 0.0;
            message.linear.z = 0.0;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = STOP");
        }
        break;

        case STOP:
        if (detect_obstacle()) {
            state_ = TURN;
            message.angular.z = 0.1;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = TURN");
        } else {
            state_ = FORWARD;
            message.linear.x = 0.2;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
        }
        break;

        case TURN:
        if (!detect_obstacle()) {
            state_ = FORWARD;
            message.linear.x = 0.2;
            publisher_->publish(message);
            RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
        }
        break;
    }
}

bool Roomba::detect_obstacle() {
    // For laser scans with non-zero minimium angle
    if (scan_.angle_min != 0) {
        // Index for the center laser scan ray
        auto ray_idx = static_cast<int>(
            (scan_.angle_max - scan_.angle_min)/(scan_.angle_increment) - 1);
        center_dist_ = scan_.ranges[ray_idx];
        left_dist_ = scan_.ranges[ray_idx - 30];
        right_dist_ = scan_.ranges[ray_idx + 30];
    } else {
        auto ray_idx = 0;
        center_dist_ = scan_.ranges[ray_idx];
        left_dist_ =
                    scan_.ranges[(scan_.angle_max/scan_.angle_increment) - 30];
        right_dist_ = scan_.ranges[ray_idx + 30];
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Distance: " << left_dist_ <<
                 " " << center_dist_ << " " << right_dist_);

    // Obstacle is detected if either of the laser scan rays picks up one.
    if (left_dist_ < 0.9 || center_dist_ < 0.9 || right_dist_ < 0.9) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Obstacle has been detected!");
        return true;
    }

    return false;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Roomba>());
  rclcpp::shutdown();
  return 0;
}
