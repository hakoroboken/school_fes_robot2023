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

use ros2_rust_util::{get_str_parameter, get_bool_parameter};
use manual_mode_common::{GetGamePadValue, check_pram_button_name, Invert};

fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("lock_arm_node", None, Default::default())?;

    // ---- ros2 topic ----
    let subscriber = node.create_subscriber::<remote_control_msgs::msg::Gamepad>("manual_input", None)?;
    let publisher_lock_arm = node.create_publisher::<actuator_control_msgs::msg::LockArm>("lock_arm", None)?;

    // ---- ros2 param ----
    let param_button_name = get_str_parameter(node.get_name(), "get_button", "x");
    check_pram_button_name(param_button_name.as_str());
    let param_reversal = get_bool_parameter(node.get_name(), "reversal", false);
    
    // ---- Create a logger ----
    let logger = Logger::new("lock_arm_node");
    pr_info!(logger, "start node: {:?}", node.get_name());

    // ---- Subscriber ----
    let mut selector = ctx.create_selector()?;
    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            let mut send_msg = actuator_control_msgs::msg::LockArm::new().unwrap();
            send_msg.power = msg.get_button(param_button_name.as_str());
            if param_reversal {
                send_msg.power.invert();
            }
            let _ = publisher_lock_arm.send(&send_msg);
        }),
    );

    loop {
        selector.wait()?;
    }
}