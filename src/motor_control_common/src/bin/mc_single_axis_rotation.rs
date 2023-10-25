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

use ros2_rust_util::get_bool_parameter;
use motor_control_common::Invert;

fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("motor_control_single_axis_rotation_node", None, Default::default())?;

     // ---- ros2 topic ----
    let subscriber = node.create_subscriber::<actuator_control_msgs::msg::SingleAxisRotation>("input_axis_rotation", None)?;
    let publisher = node.create_publisher::<actuator_control_msgs::msg::RawMotor>("raw_motor", None)?;

    // ---- ros2 param ----
    let param_reversal = get_bool_parameter(node.get_name(), "reversal", false);

    // ---- Create a logger ----
    let logger = Logger::new("motor_control_single_axis_rotation_node");
    pr_info!(logger, "start node: {:?}", node.get_name());

    // ---- Subscriber ----
    let mut selector = ctx.create_selector()?;
    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            let mut send_msg = actuator_control_msgs::msg::RawMotor::new().unwrap();
            send_msg.power = msg.power;
             if param_reversal {
                send_msg.power.invert();
            }
            send_msg.enable_servo = false;
            let _ = publisher.send(&send_msg);
        }),
    );

    loop {
        selector.wait()?;
    }
}