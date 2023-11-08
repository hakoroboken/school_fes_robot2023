/*
 * haya_topic_hz.cpp
 *
 * License: BSD-3-Clause
 *
 * Copyright (c) 2012-2023 Shoun Corporation <research.robosho@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of RT Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"
#include "haya_imu_msgs/msg/imu_data.hpp"

using std::placeholders::_1;

class HayaTopicHz : public rclcpp::Node {
public:
    HayaTopicHz(): Node("haya_topic_hz") {
        // Create the sensor QoS profile for subscriber
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        qos_profile.depth = 1;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        // Create the synchronous subscriber on topic '/imu_data' and tie it to the topic_callback
        haya_subscription_ = this->create_subscription<haya_imu_msgs::msg::ImuData>("/imu_data", qos, std::bind(&HayaTopicHz::topic_callback, this, _1));              
    }

private:
    // Frame counter to measure hz
    const uint32_t MAX_COUNT_HZ =1000;

    // A subscriber that listens to topic '/imu_data'
    rclcpp::Subscription<haya_imu_msgs::msg::ImuData>::SharedPtr haya_subscription_;

    /**
     * Actions to run every time a new message is received
     */
    void topic_callback(const haya_imu_msgs::msg::ImuData & imu_msg) {
        // The following is to measure hz
        static uint32_t count = 0;
        static rclcpp::Time start_time;

        if (++count == 1) {
            start_time = rclcpp::Clock().now();
        }
        else if (count == MAX_COUNT_HZ) { // 1000 frames here
            RCLCPP_INFO(this->get_logger(), "[Subs] hz: %5.1lf, window: %d", static_cast<uint32_t>(MAX_COUNT_HZ) / (rclcpp::Clock().now() - start_time).seconds(), count);
            count = 0;
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HayaTopicHz>());
    rclcpp::shutdown();
    return 0;
}
// haya_topic_hz.cpp