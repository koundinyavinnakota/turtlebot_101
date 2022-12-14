/**
 * @file publish_details.cpp
 * @author Koundinya Vinnakota
 * @brief This the publisher and subscriber that controls the turtlebot
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
/**
 * @brief This is the turtlebot controller  class
 *
 */
class bot_control : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Details Publisher object
   *
   */
  bot_control() : Node("bot_control") {
    // subscription to the laser scan topic
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 1, std::bind(&bot_control::topic_callback, this, _1));

    // Creating a publisher for bot control
    botPublisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    RCLCPP_INFO(this->get_logger(), "Obstacle flag : %d", thresholdCroseed);
  }

 private:
 /**
  * @brief This is a subscriber call back function
  * 
  * @param intensities 
  */
  void topic_callback(
      const sensor_msgs::msg::LaserScan::ConstPtr &intensities) {
    
    detector(intensities);// function call for detector alogorithm
    auto message = geometry_msgs::msg::Twist();
    if (thresholdCroseed) {
      RCLCPP_INFO(this->get_logger(), "Danger turn around");
      message.linear.x = 0.0;
      message.angular.z = rotateAngle;

    } else {
      RCLCPP_INFO(this->get_logger(), "Safe to move forward");
      message.linear.x = 0.4;
      message.angular.z = 0.0;
    }
    botPublisher_->publish(message);// publishing to /cmd_vel
  }
  /**
   * @brief This is a function that detects obstacles
   * 
   * @param intensities 
   */
  void detector(const sensor_msgs::msg::LaserScan::ConstPtr &intensities) {
    for (auto intensity : intensities->ranges) {
      if (intensity <= thresholdIntensity) {
        thresholdCroseed = true;  /// changes flag if threshold crossed
        RCLCPP_INFO(this->get_logger(), "Danger");
        break;
      } else {
        thresholdCroseed = false;
        break;
      }
    }
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr botPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist mover_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  double thresholdIntensity = 0.9;// Threshold intensity - distance proximity
  double rotateAngle = 0.7;// Amount of rotation of turtlebot
  bool thresholdCroseed = false;// obstacle flag
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bot_control>());
  rclcpp::shutdown();
  return 0;
}
