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
use async_std::channel::unbounded;

use motor_control_common::merger_module;

#[async_std::main]
async fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("merger_node", None, Default::default())?;

    // ---- ros2 topic ----
    let subscriber_0 = node.create_subscriber::<actuator_control_msgs::msg::RawMotor>("raw_motor_0", None)?;
    let subscriber_1 = node.create_subscriber::<actuator_control_msgs::msg::RawMotor>("raw_motor_1", None)?;
    let subscriber_2 = node.create_subscriber::<actuator_control_msgs::msg::RawMotor>("raw_motor_2", None)?;
    let subscriber_3 = node.create_subscriber::<actuator_control_msgs::msg::RawMotor>("raw_motor_3", None)?;
    let publisher = node.create_publisher::<mc_msgs::msg::Data>("motor_contorl", None)?;

    // ---- IPC ----
    let (motor_data_send, motor_data_rcv) = unbounded();
    let (closer_send , closer_rcv) = unbounded();
    // ---- Create a logger ----
    let logger = Logger::new("lock_arm_node");
    pr_info!(logger, "start node: {:?}", node.get_name());

    // ---- Process Spawn ----
    let get_signaler = async_std::task::spawn(merger_module::get_signal(closer_send.clone()));
    let motor_subcliber_0 = async_std::task::spawn(
        merger_module::motor_data_subscriber(closer_rcv.clone(), motor_data_send.clone(), subscriber_0, 0));
    let motor_subcliber_1 = async_std::task::spawn(
        merger_module::motor_data_subscriber(closer_rcv.clone(), motor_data_send.clone(), subscriber_1, 1));
    let motor_subcliber_2 = async_std::task::spawn(
        merger_module::motor_data_subscriber(closer_rcv.clone(), motor_data_send.clone(), subscriber_2, 2));
    let motor_subcliber_3 = async_std::task::spawn(
        merger_module::motor_data_subscriber(closer_rcv.clone(), motor_data_send.clone(), subscriber_3, 3));
    let publisher = async_std::task::spawn(
        merger_module::motor_data_send(closer_rcv.clone(), motor_data_rcv, publisher));
    // ---- Process Run ----
    get_signaler.await?;
    motor_subcliber_0.await?;
    motor_subcliber_1.await?;
    motor_subcliber_2.await?;
    motor_subcliber_3.await?;
    publisher.await?;

    pr_info!(logger, "End node");
    Ok(())
}
