/**
 * @file walker.hpp
 * @author Bharadwaj Chukkala (bchukkal@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-12-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using SCAN = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

// using a State Machine
typedef enum {
  FORWARD = 0,
  STOP,
  TURN,
} StateType;

/**
 * @brief Class to implement roomba vaccum cleaner algorithm for the TurtleBot3
 * 
 */
class Roomba : public rclcpp::Node {
 public:
    Roomba();

 private:
    /**
     * @brief Callback function to determine the state of the system and decide on the new input.
     * 
     */
    void timer_callback();

    /**
     * @brief Callback function to receive the incoming laser scan data
     * 
     */
    void laser_callback(const SCAN&);

    /**
     * @brief Member function to detect if an obstacle is present in front of the robot.
     * 
     * @return true If obstacle is detected
     * @return false If obstacle is not detected
     */
    bool detect_obstacle();

    rclcpp::Publisher<TWIST>::SharedPtr publisher_;
    rclcpp::Subscription<SCAN>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    SCAN scan_;  // To store incoming laser scan data
    float left_dist_;  // Distance measured by left ray of Laser
    float center_dist_;  // Distance measured by center ray of Laser
    float right_dist_;  // Distance measured by right ray of Laser
    StateType state_;
};
