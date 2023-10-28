// Copyright 2023 Taiga Takano
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

use safe_smart_controller_gateway::network_module;
use async_std::channel::unbounded;
use async_std::net::UdpSocket;
use ros2_rust_util::{get_bool_parameter, get_i64_parameter};
use safe_drive::{context::Context, error::DynError, logger::Logger, pr_info};

#[async_std::main]
async fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("scgw", None, Default::default())?;

    let publisher = node.create_publisher::<scgw_msgs::msg::Data>("scgw_raw", None)?;

    // Create a logger.
    let logger = Logger::new("scgw");
    pr_info!(logger, "Start {:?}", node.get_name());

    let param_port = get_i64_parameter(node.get_name(), "port", 64201);
    let param_debug = get_bool_parameter(node.get_name(), "debug", false);
    let param_id = get_i64_parameter(node.get_name(), "id", 120);

    let (closer_send, closer_rcv) = unbounded();

    let (search_locker_send, search_locker_rcv) = unbounded();
    let (target_info_send, target_info_rcv) = unbounded();

    let main_udp_socket = UdpSocket::bind(format!("0.0.0.0:{param_port}")).await?;
    let search_app_socket = UdpSocket::bind("0.0.0.0:0").await?;
    let ping_socket = UdpSocket::bind("0.0.0.0:0").await?;

    let get_signaler = async_std::task::spawn(network_module::get_signal(closer_send));
    let main_udp_service_task = async_std::task::spawn(network_module::main_udp_service(
        main_udp_socket,
        closer_rcv.clone(),
        search_locker_send,
        target_info_send,
        publisher,
        param_debug.clone(),
    ));
    let search_app_task = async_std::task::spawn(network_module::search_app(
        search_app_socket,
        closer_rcv.clone(),
        search_locker_rcv,
        param_id.clone(),
        param_debug.clone(),
    ));
    let ping_task = async_std::task::spawn(network_module::ping_app(
        ping_socket,
        closer_rcv.clone(),
        target_info_rcv,
        param_id.clone(),
        param_debug.clone(),
    ));

    main_udp_service_task.await?;
    search_app_task.await?;
    ping_task.await?;
    get_signaler.await?;

    pr_info!(logger, "End node");
    Ok(())
}
