/*
 * haya_imu_node.cpp
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

#include "haya_imu_node.hpp"

/*
 * HayaImuNode() HayaImuNode class constractor
 */
HayaImuNode::HayaImuNode() : Node("haya_imu_node") {
    // Declare parameters, and takes default value
    declare_parameter("serial_port_name", "/dev/ttyACM0");
    declare_parameter("imu_topic", "/imu_data");
    declare_parameter("imu_frame", "imu_link");
    declare_parameter("imu_mode", 500);

    // Get parameters from config file, otherwise takes default value
    port_name_ = get_parameter("serial_port_name").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_frame_ = get_parameter("imu_frame").as_string();
    imu_mode_ = static_cast <int16_t>(get_parameter("imu_mode").as_int());

    // Set topic for publishing
    if (imu_mode_ == DEMONSTRATION_MODE) { // Demo mode
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    else { // ODR mode or Calibration mode      
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        qos_profile.depth = 1;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
        publish_imu_ = this->create_publisher<haya_imu_msgs::msg::ImuData>(imu_topic_, qos);
    }
    // Covariance matrix
    for (int32_t i; i < sizeof(angular_velocity_covariance_) / sizeof(double); i++) {
        imu_msg_.angular_velocity_covariance[i] = angular_velocity_covariance_[i];
        imu_msg_.linear_acceleration_covariance[i] = linear_acceleration_covariance_[i];
        imu_msg_.magnetic_field_covariance[i] = magnetic_field_covariance_[i];
    }

    // Create CRC table
    CreateCrcTable();

    // Try connecting serial port
     while (rclcpp::ok()) {
        if (!LiteSerial::Open()) { // Disconnected
            RCLCPP_INFO_ONCE(this->get_logger(), "[Open] Try opening %s, which can be changed in params.yaml", port_name_.c_str());
            rclcpp::Duration(1.0, 0); // Sleep for 1.0s, try modifying to shorter or longer value according to needs
        }
        else { // Connected
            return;
        }
     }
}

/*
 * ~HayaImuNode() HayaImuNode class constractor
*/
HayaImuNode::~HayaImuNode() {
    // Add if needed
}

/*
 * SendParams() Send parameters through serial port
 */
void HayaImuNode::SendParams() {
    int32_t response[2];

    // Check parameter imu_mode_ once
    if ((imu_mode_ < CALIBRATION_MODE) || (imu_mode_ > ODR_1KHZ_MODE)){ // Exception
        RCLCPP_WARN(this->get_logger(), "[Para] Parameter(imu_mode in params.yaml) be out of range, imu works in ODR_500Hz_MODE");
        imu_mode_ = ODR_500Hz_MODE;
    }
    
    // Send parameter to imu
    size_t len = LiteSerial::WriteBuff(reinterpret_cast <uint8_t*>(&imu_mode_), sizeof(imu_mode_));

    // Wait for response from imu
    while (rclcpp::ok()) {
        if (LiteSerial::ReadBuff(reinterpret_cast <uint8_t*>(response), sizeof(response)) < sizeof(response)) {
            RCLCPP_ERROR_ONCE(this->get_logger(), "[Para] Failed to receive response of parameter setup, check USB connection & parameters in params.yaml");
           continue;
        }
        else {
            //RCLCPP_INFO(this->get_logger(), "[Para] Response of imu parmas setup received: 0x%08x,  0x%08x", response[0], response[1]);
            break;
        }
    }

    // Check the hardware version of haya_imu once
    if ((((response[0] >> 8) & 0xff) != 0x03)) { // Valid version range: v3.1.0 - v3.f.f(max)
        RCLCPP_ERROR(this->get_logger(), "[Para] Before performing a launch file, turn off then turn on power, or reset imu once. Press Ctrl+c to try again");
        while (rclcpp::ok()); // Press Ctrl+c to try again
    }

    // Release information on device to ROS
    switch (imu_mode_) {
        case DEMONSTRATION_MODE:
            RCLCPP_INFO(this->get_logger(), "[Para] Serial: %s, Initial_Bias: %s, Mode: Demo @250Hz, Firmware: v%d.%d.%d, Product_SN: %08x",
                        port_name_.c_str(), is_calibrated_[(response[0] >> 16 ) &0x1].c_str(), (response[0] >> 8) & 0xf, (response[0] >> 4 ) & 0xf, 
                        response[0]&0xf, response[1]);
            break;

        case CALIBRATION_MODE:
            RCLCPP_INFO(this->get_logger(), "[Para] Serial: %s, Mode: Calibration @10Hz, Firmware: v%d.%d.%d, Product_SN: %08x",
                        port_name_.c_str(), (response[0] >> 8) & 0xf, (response[0] >> 4) & 0xf, response[0] & 0xf, response[1]);
            break;

        default: // ODR mode
            RCLCPP_INFO(this->get_logger(), "[Para] Serial: %s, Initial_Bias: %s, Mode: Normal Output @%dHz, Firmware: v%d.%d.%d, Product_SN: %08x",
                        port_name_.c_str(), is_calibrated_[(response[0]>>16)&0x1].c_str(), 
                        imu_mode_, (response[0] >> 8) & 0xf, (response[0] >> 4) & 0xf, response[0] & 0xf, response[1]);
    }
}

/*
 * ReadImuData() Read Imu Data
 */
void HayaImuNode::ReadImuData() {
    // Iteration to read imu data
    while (rclcpp::ok()) {
        // Read imu_data_ buffer from serial port
        if (LiteSerial::ReadBuff(reinterpret_cast <uint8_t*>(&imu_data_), sizeof(imu_data_)) < sizeof(imu_data_)) { // Disconnected
            RCLCPP_ERROR_ONCE(this->get_logger(), "[Read] USB disconnected, press Ctrl+c to stop the node, then reconnect haya_imu, then ros2 launch again");
            continue;
        }
        else { // Buffer read ok
            // CRC check
            if (imu_data_.crc16 == CalCrc(reinterpret_cast <uint8_t*>(&imu_data_), sizeof(imu_data_) - sizeof(uint16_t))) { // CRC be ok
                PublishImuMsg();
                //RCLCPP_INFO(this->get_logger(), "[Publ] header_stamp: %d, imu_stamp: %ld, mag_stamp: %ld, imu_data_.crc16: %04x", imu_msg_.header.stamp.nanosec, imu_data_.time_stamp_imu, imu_data_.time_stamp_magnetometer, imu_data_.crc16);
                //RCLCPP_INFO(this->get_logger(), "[Publ] nanosec: %ld", rclcpp::Clock().now().nanoseconds());
            }
            else { // CRC error occurs once
                RCLCPP_INFO(this->get_logger(), "[Read] Wrong imu data received once. If wrong data occurs ofen, check environment & parameters in params.yaml");
            }
        }
    }
}

/*
 * PublishImuData() Publish topic
 */
void HayaImuNode::PublishImuMsg() {
    if (imu_mode_ == DEMONSTRATION_MODE) { // Demo mode
        geometry_msgs::msg::TransformStamped t6, t9;
        t6.header.stamp = this->get_clock()->now();
        t6.header.frame_id = "world_6axis";
        t6.child_frame_id = "fusion_6axis";
        t9.header.stamp = t6.header.stamp;
        t9.header.frame_id = "world_9axis";
        t9.child_frame_id = "fusion_9axis";

        t6.transform.rotation.w = imu_data_.quaternion[0][0];
        t6.transform.rotation.x = imu_data_.quaternion[0][1];
        t6.transform.rotation.y = imu_data_.quaternion[0][2];
        t6.transform.rotation.z = imu_data_.quaternion[0][3];
        t9.transform.rotation.w = imu_data_.quaternion[1][0];
        t9.transform.rotation.x = imu_data_.quaternion[1][1];
        t9.transform.rotation.y = imu_data_.quaternion[1][2];
        t9.transform.rotation.z = imu_data_.quaternion[1][3];

        // Release information om publishing fusion data once
        RCLCPP_INFO_ONCE(this->get_logger(), "[Tfbr] Tf Broadcaster: 6-axis quaternion & 9-axis quaternion @ %dHz", imu_mode_);
        tf_broadcaster_->sendTransform(t6);
        tf_broadcaster_->sendTransform(t9);
    }
    else { // ODR mode or Calibration mode

        // Message header
        imu_msg_.header.stamp = this->get_clock()->now();
        //imu_msg_.header.stamp = rclcpp::Clock().now();
        imu_msg_.header.frame_id = imu_frame_;

        // Angular velocity
        imu_msg_.angular_velocity.x = imu_data_.angular_velocity[0];
        imu_msg_.angular_velocity.y = imu_data_.angular_velocity[1];
        imu_msg_.angular_velocity.z = imu_data_.angular_velocity[2];

        // Linear acceleration
        imu_msg_.linear_acceleration.x = imu_data_.linear_acceleration[0];
        imu_msg_.linear_acceleration.y = imu_data_.linear_acceleration[1];
        imu_msg_.linear_acceleration.z = imu_data_.linear_acceleration[2];

        // Magnetic field
        imu_msg_.magnetic_field.x = imu_data_.magnetic_field[0];
        imu_msg_.magnetic_field.y = imu_data_.magnetic_field[1];
        imu_msg_.magnetic_field.z = imu_data_.magnetic_field[2];

        // 6-axis fusion quaternion
        imu_msg_.orientation[0].w = imu_data_.quaternion[0][0];
        imu_msg_.orientation[0].x = imu_data_.quaternion[0][1];
        imu_msg_.orientation[0].y = imu_data_.quaternion[0][2];
        imu_msg_.orientation[0].z = imu_data_.quaternion[0][3];

        // 9-axis fusion quaternion
        imu_msg_.orientation[1].w = imu_data_.quaternion[1][0];
        imu_msg_.orientation[1].x = imu_data_.quaternion[1][1];
        imu_msg_.orientation[1].y = imu_data_.quaternion[1][2];
        imu_msg_.orientation[1].z = imu_data_.quaternion[1][3];

        // Combined calibration index, 0x00(worst) -> 0x3f(best)
        imu_msg_.calibration_index = imu_data_.calibration_index;

        // 6-axis imu temperature
        imu_msg_.temperature_imu = imu_data_.temperature_imu;

        // Get Euler angle, rotation sequence is Yaw -> Pitch -> Roll
        GetEulerYPR(&imu_msg_);

        // Publish imu topic
        publish_imu_->publish(imu_msg_);

        // Release information on publishing fusion data once
        RCLCPP_INFO_ONCE(this->get_logger(), "[Publ] Publishing Topic: %s (haya_imu_msgs::msg::ImuData) @ %dHz", imu_topic_.c_str(), imu_mode_); 
    }
}

/*
 * GetEulerYPR() Get Euler angle, refer to manual for more information
 */
void HayaImuNode::GetEulerYPR(haya_imu_msgs::msg::ImuData *p_msg) {
    geometry_msgs::msg::Quaternion geometry_quat;
    geometry_quat.x = p_msg->orientation[0].x;
    geometry_quat.y = p_msg->orientation[0].y;
    geometry_quat.z = p_msg->orientation[0].z;
    geometry_quat.w = p_msg->orientation[0].w;
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(geometry_quat, tf2_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);
    p_msg->euler_ypr.z = yaw;
    p_msg->euler_ypr.y = pitch;
    p_msg->euler_ypr.x = roll;
}

/*
 * Crc16Table()  Calculate CRC : CCITT16 Table
 */
void HayaImuNode::CreateCrcTable(void) { 
    int32_t i, j;
    uint16_t crc;
    for (i = 0; i < 256; i++) {
        crc = (i << 8);
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = polynom_ ^ (crc << 1);
            }
            else {
                crc <<= 1;
            }
        }
        crc_table_[i] = crc;
    }
}

/*
 * Crc16Table()  Calculate CRC: CCITT16
 */
uint16_t HayaImuNode::CalCrc(uint8_t *data, size_t len) {
    uint32_t crc = 0xffff, final = 0x0000;
    uint32_t temp;

    for (uint32_t i = 0; i < len; ++i) {
        temp = (*data++ ^ (crc >> 8)) & 0xff;
        crc = crc_table_[temp] ^ (crc << 8);
    }

    return static_cast<uint16_t>(crc ^ final);
}

// Serial port file descriptor
int32_t serial_fd = -1;

// Struct for config restoration
struct termios tio_back;

/*
 * sigint_handler() Ctrl+c handler
*/
void sigint_handler(int sig) {
    if (serial_fd >= 0) {
        // Flush input & output buffer
        tcflush(serial_fd, TCIFLUSH);
        tcflush(serial_fd, TCOFLUSH);

        // Store configuration
        tcsetattr(serial_fd, TCSANOW, &tio_back);

        // Release fd
        close(serial_fd);
    }
    rclcpp::shutdown();
}

/*
 * main() for haya_imu_node
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigInt);

    // Create an instance
    HayaImuNode imu_node;
    serial_fd = imu_node.GetSerialFd();
    tio_back = imu_node.GetTermio();

    // Set ctrl+c handler
    signal(SIGINT, sigint_handler);

    // Send parameters
    imu_node.SendParams();

    // Receive imu data
    imu_node.ReadImuData();

    // Shut down
    imu_node.Close();

    //RCPCPP_INFO(this->get_logger(), "[Close] haya_imu_node done\n");
    rclcpp::shutdown();
    return 0;
}
// haya_imu_node.cpp