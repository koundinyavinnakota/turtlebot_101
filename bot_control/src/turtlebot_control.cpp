/**
 * @file publish_details.cpp
 * @author Koundinya Vinnakota
 * @brief This the publisher and servie provider
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
// #include "std_msgs/msg/LaserScan.hpp"

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
      "/scan", 30, std::bind(&bot_control::topic_callback, this, _1));
    

    //Creating a publisher for bot control
    botPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 30);
    mover_.linear.x = 0;   /// Linear velocity in X direction 0
    mover_.linear.y = 0;   /// Linear velocity in Y direction 0
    mover_.linear.z = 0;   /// Linear velocity in Z direction 0
    mover_.angular.x = 0;  /// Angular velocity in X direction 0
    mover_.angular.y = 0;  /// Angular velocity in Y direction 0
    mover_.angular.z = 0;  /// Angular velocity in Z direction 0
    timer_ = this->create_wall_timer(500ms, std::bind(&bot_control::timer_callback, this));
    botPublisher_->publish(mover_);
    RCLCPP_INFO(this->get_logger(),"Obstacle flag : %d",thresholdCroseed);
  
}
 private:
  void timer_callback(){
    auto message = geometry_msgs::msg::Twist();
    if(!thresholdCroseed){
      RCLCPP_INFO(this->get_logger(),"Safe to move forward");
      message.linear.x = 0.5; 
      botPublisher_->publish(message);

    }
    else{
      RCLCPP_INFO(this->get_logger(),"Ooops... Danger... Time to turn");
      message.linear.x = 0; 
      message.linear.y = 0; 
      message.linear.z = 0; 
      message.angular.x = 0; 
      message.angular.y = 0; 
      message.angular.z = 0; 
      botPublisher_->publish(message);

      message.angular.z = rotateAngle;
      botPublisher_->publish(message);

    }
    


  }

  void topic_callback(const sensor_msgs::msg::LaserScan::ConstPtr & intensities){
    for (auto intensity : intensities->ranges) {
      if (intensity <= thresholdIntensity) {
        thresholdCroseed = true;  /// changes flag if threshold crossed
        RCLCPP_INFO(this->get_logger(),"Danger");
        break;
      } else {
        thresholdCroseed = false;
      }

    }
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr botPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist mover_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  double thresholdIntensity = 0.5;
  double rotateAngle = 0.75;
  bool thresholdCroseed = false;

  
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bot_control>());
  rclcpp::shutdown();
  return 0;
}

