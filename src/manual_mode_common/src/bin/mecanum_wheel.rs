// Copyright 2023 Hakoroboken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use safe_drive::{
    context::Context, 
    error::DynError, 
    logger::Logger,
    pr_info,
};

use ros2_rust_util::{get_str_parameter, get_f64_parameter};
use manual_mode_common::{check_pram_value_name, GetGamePadValue};

fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("mecanum_wheel_node", None, Default::default())?;

    // ---- ros2 topic ----
    let subscriber = node.create_subscriber::<remote_control_msgs::msg::Gamepad>("manual_input", None)?;
    let publisher_mecanum_wheel = node.create_publisher::<actuator_control_msgs::msg::MecanumWheel>("mecanum_wheel", None)?;

    // ---- ros2 param ----
    let param_x_name = get_str_parameter(node.get_name(), "x_name", "x.joystic.left");
    check_pram_value_name(param_x_name.as_str());
    let param_y_name = get_str_parameter(node.get_name(), "y_name", "y.joystic.left");
    check_pram_value_name(param_y_name.as_str());
    let param_rotation_name = get_str_parameter(node.get_name(), "rotation_name", "x.joystic.right");
    check_pram_value_name(param_rotation_name.as_str());
    let param_no_accel_power = get_f64_parameter(node.get_name(), "no_accel_power", 0.15) as f32;

    // ---- Create a logger ----
    let logger = Logger::new("mecanum_wheel_node");
    pr_info!(logger, "start node: {:?}", node.get_name());

    // ---- Subscriber ----
    let mut selector = ctx.create_selector()?;
    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            let mut send_msg = actuator_control_msgs::msg::MecanumWheel::new().unwrap();
            send_msg.vec_x = msg.get_value(param_x_name.as_str()) * ( param_no_accel_power + (1.0 - param_no_accel_power) * (1.0 - msg.get_value("trigger.left")));
            send_msg.vec_y = msg.get_value(param_y_name.as_str()) * ( param_no_accel_power + (1.0 - param_no_accel_power) * (1.0 - msg.get_value("trigger.left")));
            send_msg.rotation_power = msg.get_value(param_rotation_name.as_str()) * ( param_no_accel_power + (1.0 - param_no_accel_power) * (msg.get_value("trigger.left") - 1.0));
            send_msg.enable_servo = false;
            let _ = publisher_mecanum_wheel.send(&send_msg);
        }),
    );

    loop {
        selector.wait()?;
    }
}
