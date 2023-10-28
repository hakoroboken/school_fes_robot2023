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

// --- Network util ---
use network_module_util::key::{ConnectionBuffer, EnumKeyUtil, NodeConnectionKey, U8KeyUtil};
use network_module_util::net;

// --- ROS 2 Socket ---
use safe_drive::msg::U8Seq;
use safe_drive::topic::publisher::Publisher;
use safe_drive::{error::DynError, logger::Logger, pr_error, pr_info};
use scgw_msgs::msg::Data;

use std::os::unix::prelude::OsStringExt;
use std::time::Duration;

// --- UDP Socket ---
use async_std::net::UdpSocket;
use gethostname::gethostname;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};

// --- Inter Thread Commnunication
use async_std::channel::{Receiver, Sender};
use async_std::future::timeout;
use async_std::prelude::*;

// --- Get Signal ---
use signal_hook::consts::signal::*;
use signal_hook_async_std::Signals;

pub async fn main_udp_service(
    socket: UdpSocket,
    closer: Receiver<bool>,
    lock_search: Sender<bool>,
    ipaddr_send: Sender<SocketAddr>,
    publisher: Publisher<Data>,
    is_debug: bool,
) -> Result<(), DynError> {
    // --- Logger ---
    let logger = Logger::new("main_udp_service");

    // --- UDP Socket ---
    let mut connection_buffer = ConnectionBuffer {
        connection_key: NodeConnectionKey::UnknownKey,
        raw_buffer: [0; 2048],
        rcv_size: 0,
        taget_address: SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 8080),
    };

    // --- Get Signal ---
    let deadline = Duration::from_secs(1);

    pr_info!(logger, "Listening on {}", socket.local_addr()?);
    loop {
        // --- revive data ---
        match timeout(
            deadline,
            socket.recv_from(&mut connection_buffer.raw_buffer),
        )
        .await
        {
            Ok(recv_result) => match recv_result {
                Ok((recv, addr)) => {
                    //connection_buffer.raw_buffer[0] is header id. please read doc.
                    connection_buffer.connection_key =
                        connection_buffer.raw_buffer[0].convert_to_enumkey();
                    connection_buffer.rcv_size = recv;
                    connection_buffer.taget_address.set_ip(addr.ip());
                    connection_buffer.taget_address.set_port(64202);

                    if is_debug {
                        pr_info!(logger, "Key:{}", connection_buffer.connection_key);
                    }

                    match connection_buffer.connection_key {
                        NodeConnectionKey::SearchAppResponse => {
                            lock_search.send(true).await?;
                            ipaddr_send.send(connection_buffer.taget_address).await?;
                        }
                        NodeConnectionKey::PingResponse => {
                            lock_search.send(true).await?;
                            ipaddr_send.send(connection_buffer.taget_address).await?;
                        }
                        NodeConnectionKey::DataValue => {
                            let mut msg = scgw_msgs::msg::Data::new().unwrap();
                            msg.id = connection_buffer.raw_buffer[1]; //data packet id. is not node connection key

                            let data_size = connection_buffer.rcv_size - 2;
                            let mut data = U8Seq::new(data_size).unwrap();
                            data.as_slice_mut()
                                .copy_from_slice(&connection_buffer.raw_buffer[2..2 + data_size]);

                            msg.data = data;
                            publisher.send(&msg)?;
                        }
                        _ => {
                            pr_error!(logger, "Unknown ID:{}", connection_buffer.raw_buffer[0]);
                        }
                    }
                }
                Err(e) => {
                    pr_error!(logger, "An error occurred during receiving: {}", e);
                    return Ok(());
                }
            },
            Err(_) => {
                if closer.try_recv() == Ok(true) {
                    pr_info!(logger, "main_udp_service thread shutdown");
                    return Ok(());
                }
            }
        }
    }
}

pub async fn search_app(
    socket: UdpSocket,
    closer: Receiver<bool>,
    locker: Receiver<bool>,
    param_id: i64,
    is_debug: bool,
) -> Result<(), DynError> {
    let logger = Logger::new("search_app");
    let deadline = Duration::from_millis(500);

    socket.set_broadcast(true)?;

    // --- create packet ---
    let mut buffer: Vec<u8> = vec![NodeConnectionKey::SearchApp.convert_to_u8key()];
    buffer.push(param_id as u8);
    buffer.append(&mut net::get_ip());
    let ip_port: u16 = 64201;
    buffer.append(&mut ip_port.to_le_bytes().to_vec());
    buffer.append(&mut gethostname().into_vec());

    buffer.resize(24, 0);

    loop {
        match timeout(deadline, closer.recv()).await {
            Ok(_) => {
                pr_info!(logger, "search_app thread shutdown");
                return Ok(());
            }
            Err(_) => {
                if locker.try_recv() == Ok(true) {
                    if is_debug {
                        pr_info!(logger, "locked");
                    }
                    async_std::task::sleep(Duration::from_millis(1000)).await;
                    while locker.try_recv() == Ok(true) {}
                    continue;
                }
                if is_debug {
                    pr_info!(logger, "search");
                }
                socket
                    .send_to(
                        &buffer,
                        &SocketAddr::new(IpAddr::V4(Ipv4Addr::new(255, 255, 255, 255)), 64202),
                    )
                    .await?;
                async_std::task::sleep(Duration::from_millis(500)).await;
            }
        }
    }
}

pub async fn ping_app(
    socket: UdpSocket,
    closer: Receiver<bool>,
    target_info_rcv: Receiver<SocketAddr>,
    param_id: i64,
    is_debug: bool,
) -> Result<(), DynError> {
    let logger = Logger::new("ping_app");
    let deadline = Duration::from_secs(1);

    loop {
        match timeout(deadline, target_info_rcv.recv()).await {
            Ok(rcv_result) => match rcv_result {
                Ok(addr) => {
                    let mut buffer: Vec<u8> =
                        vec![NodeConnectionKey::PingRequest.convert_to_u8key()];
                    buffer.push(param_id as u8);
                    socket.send_to(&buffer, addr).await?;
                    if is_debug {
                        pr_info!(logger, "ping send to {}", addr);
                    }

                    async_std::task::sleep(Duration::from_millis(500)).await;
                }
                Err(e) => {
                    pr_error!(logger, "An error occurred during receiving: {}", e);
                    return Ok(());
                }
            },
            Err(_) => {
                if closer.try_recv() == Ok(true) {
                    pr_info!(logger, "ping_app thread shutdown");
                    return Ok(());
                }
            }
        }
    }
}

pub async fn get_signal(closer: Sender<bool>) -> Result<(), DynError> {
    let signals = Signals::new(&[SIGHUP, SIGTERM, SIGINT, SIGQUIT])?;
    let mut signals = signals.fuse();
    loop {
        if let Some(signal) = signals.next().await {
            match signal {
                SIGTERM | SIGINT | SIGQUIT => {
                    // Shutdown the system;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    async_std::task::sleep(Duration::from_millis(100)).await;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    return Ok(());
                }
                _ => unreachable!(),
            }
        }
    }
}
