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

use safe_drive::{context::Context, error::DynError, logger::Logger, pr_info};

fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("mecanum_wheel_node", None, Default::default())?;

    // ---- ros2 topic ----
    let subscriber =
        node.create_subscriber::<actuator_control_msgs::msg::MecanumWheel>("manual_input", None)?;
    let publisher_fl =
        node.create_publisher::<actuator_control_msgs::msg::RawMotor>("raw_motor_fl", None)?;
    let publisher_fr =
        node.create_publisher::<actuator_control_msgs::msg::RawMotor>("raw_motor_fr", None)?;
    let publisher_ll =
        node.create_publisher::<actuator_control_msgs::msg::RawMotor>("raw_motor_rl", None)?;
    let publisher_lr =
        node.create_publisher::<actuator_control_msgs::msg::RawMotor>("raw_motor_rr", None)?;

    // ---- Create a logger ----
    let logger = Logger::new("mecanum_wheel_node");
    pr_info!(logger, "start node: {:?}", node.get_name());

    // ---- Subscriber ----
    let mut selector = ctx.create_selector()?;
    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            let mut send_msg_fl = actuator_control_msgs::msg::RawMotor::new().unwrap();
            let mut send_msg_fr = actuator_control_msgs::msg::RawMotor::new().unwrap();
            let mut send_msg_ll = actuator_control_msgs::msg::RawMotor::new().unwrap();
            let mut send_msg_lr = actuator_control_msgs::msg::RawMotor::new().unwrap();
            send_msg_fl.power =
                0.707106781 * msg.vec_x - 0.707106781 * msg.vec_y + msg.rotation_power;
            send_msg_fr.power =
                0.707106781 * msg.vec_x - 0.707106781 * msg.vec_y - msg.rotation_power;
            send_msg_ll.power =
                -0.707106781 * msg.vec_x - 0.707106781 * msg.vec_y + msg.rotation_power;
            send_msg_lr.power =
                -0.707106781 * msg.vec_x - 0.707106781 * msg.vec_y - msg.rotation_power;
            send_msg_fl.power *= -1.0;
            send_msg_fr.power *= -1.0;
            send_msg_ll.power *= -1.0;
            send_msg_lr.power *= -1.0;
            let _ = publisher_fl.send(&send_msg_fl);
            let _ = publisher_fr.send(&send_msg_fr);
            let _ = publisher_ll.send(&send_msg_ll);
            let _ = publisher_lr.send(&send_msg_lr);
        }),
    );

    loop {
        selector.wait()?;
    }
}

//            Front

// fl(motor-0) ---- fr(motor-1)
//     |                |
//     |      Robot     |
//     |                |
//     |                |
// rl(motor-3) ---- rr(motor-4)