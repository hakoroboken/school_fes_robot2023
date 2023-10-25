#include <chrono>
#include <functional>
#include <memory>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "mc_msgs/msg/data.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

struct motor_control_msg{
    std::uint8_t id;
    float motor_1;
    float motor_2;
    float motor_3;
    float motor_4;
};

template <typename T>
std::vector<uint8_t> serialize(const T& data) {
    std::vector<uint8_t> bytes(sizeof(data));
    std::memcpy(bytes.data(), &data, sizeof(data));
    return bytes;
}

class mdc_transporter_node : public rclcpp::Node
{
  public:
    mdc_transporter_node()
    : Node("mdc_transporter_node")
    {
      this->declare_parameter<double>("gain" , 0.1);
      this->get_parameter("gain" , param_gain);

      publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 10);
      subscription_ = this->create_subscription<mc_msgs::msg::Data>(
      "rc_command", 10, std::bind(&mdc_transporter_node::topic_callback, this, _1));

      timer_ = this->create_wall_timer(20ms , [this](){
        this->timer_callback();
      });

      mrm_ = this->create_wall_timer(500ms , [this](){
        this->mrm_callback();
      });

      target = motor_control_msg();
      histry = motor_control_msg();
    }

  private:
    void topic_callback(const mc_msgs::msg::Data sub_msg)
    {
      target.id = 0;
      target.motor_1 = sub_msg.motor_a;
      target.motor_2 = sub_msg.motor_b;
      target.motor_3 = sub_msg.motor_c;
      target.motor_4 = sub_msg.motor_d;

      mrm_flag = true;
    }

    void timer_callback(){
      // motor 1
      auto rt_ = motor_control_msg();
      auto vec = target.motor_1 - histry.motor_1;
      vec = std::sqrt(vec * vec);
      if(vec >param_gain){
        if(target.motor_1 > histry.motor_1){
          rt_.motor_1 = histry.motor_1 + param_gain;
        }else{
          rt_.motor_1 = histry.motor_1 - param_gain;
        }
      }else{
        rt_.motor_1 = target.motor_1;
      }
      histry.motor_1 = rt_.motor_1;

      // motor 2
      vec = target.motor_2 - histry.motor_2;
      vec = std::sqrt(vec * vec);
      if(vec >param_gain){
        if(target.motor_2 > histry.motor_2){
          rt_.motor_2 = histry.motor_2 + param_gain;
        }else{
          rt_.motor_2 = histry.motor_2 - param_gain;
        }
      }else{
        rt_.motor_2 = target.motor_2;
      }
      histry.motor_2 = rt_.motor_2;

      // motor 3
      vec = target.motor_3 - histry.motor_3;
      vec = std::sqrt(vec * vec);
      if(vec >param_gain){
        if(target.motor_3 > histry.motor_3){
          rt_.motor_3 = histry.motor_3 + param_gain;
        }else{
          rt_.motor_3 = histry.motor_3 - param_gain;
        }
      }else{
        rt_.motor_3 = target.motor_3;
      }
      histry.motor_3 = rt_.motor_3;

      // motor 4
      vec = target.motor_4 - histry.motor_4;
      vec = std::sqrt(vec * vec);
      if(vec >param_gain){
        if(target.motor_4 > histry.motor_4){
          rt_.motor_4 = histry.motor_4 + param_gain;
        }else{
          rt_.motor_4 = histry.motor_4 - param_gain;
        }
      }else{
        rt_.motor_4 = target.motor_4;
      }
      histry.motor_4 = rt_.motor_4;


      RCLCPP_INFO(this->get_logger(), "%lf,%lf,%lf,%lf" , rt_.motor_1,rt_.motor_2,rt_.motor_3,rt_.motor_4);


      auto msg = std_msgs::msg::UInt8MultiArray();
      msg.data = serialize(rt_);
      msg.data.insert(msg.data.begin(),'t');
      msg.data.insert(msg.data.begin(),'s');
      msg.data.push_back('e');
      msg.data.push_back('n');
      publisher_->publish(msg);

    }

    void mrm_callback(){
      if(mrm_flag){
        mrm_flag = false;
        return;
      }
      target = motor_control_msg();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr mrm_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<mc_msgs::msg::Data>::SharedPtr subscription_;

    motor_control_msg target;
    motor_control_msg histry;
    double param_gain = 0.0;

    bool mrm_flag = false;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mdc_transporter_node>());
  rclcpp::shutdown();
  return 0;
}