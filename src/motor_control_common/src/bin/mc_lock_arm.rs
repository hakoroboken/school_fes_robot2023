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

fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("lock_arm_node", None, Default::default())?;

    // ---- ros2 topic ----
    let subscriber = node.create_subscriber::<actuator_control_msgs::msg::LockArm>("manual_input", None)?;
    let publisher_lock_arm = node.create_publisher::<actuator_control_msgs::msg::RawMotor>("lock_arm", None)?;
    
    // ---- Create a logger ----
    let logger = Logger::new("lock_arm_node");
    pr_info!(logger, "start node: {:?}", node.get_name());

    // ---- Subscriber ----
    let mut selector = ctx.create_selector()?;
    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            let mut send_msg = actuator_control_msgs::msg::RawMotor::new().unwrap();
            if msg.power {
                send_msg.power = 1.0
            }else{
                send_msg.power = -1.0
            }
            let _ = publisher_lock_arm.send(&send_msg);
        }),
    );

    loop {
        selector.wait()?;
    }
}