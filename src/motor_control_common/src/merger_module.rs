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

// --- ROS2 ---
use safe_drive::topic::publisher::Publisher;
use safe_drive::topic::subscriber::Subscriber;
use safe_drive::{
    error::DynError,
    logger::Logger,
    pr_info
};

// --- Inter Thread Commnunication
use async_std::channel::{Sender , Receiver};
use async_std::future::timeout;
use async_std::prelude::*;
use std::time::Duration;

// --- Get Signal ---
use signal_hook::consts::signal::*;
use signal_hook_async_std::Signals;


// motor data
pub struct MotorDataForIPC {
    pub power: f32,
    pub id: u8,
}

pub async fn motor_data_subscriber(
    process_closer: Receiver<bool>,
    motor_data_send: Sender<MotorDataForIPC>,
    mut subscriber: Subscriber<actuator_control_msgs::msg::RawMotor>,
    motor_id: u8
)-> Result<(), DynError>{
    // --- Logger ---
    let logger = Logger::new("motor_data_subscriber");
    pr_info!(logger, "start motor_data_subscriber");

    // --- Get Signal ---
    let deadline = Duration::from_secs(1);

    loop {
        let mut power: f32 = 0.0;
        let mut i_add: bool = false;
        match timeout(deadline, subscriber.recv()).await {
            Ok(msg_result) => {
                let msg = msg_result?;
                power = msg.power.clone();
                i_add = true;
            },
            Err(_) => {
                if process_closer.try_recv() == Ok(true){
                    pr_info!(logger, "motor_data_subscriber thread shutdown");
                    return Ok(());
                }
            }
        }
        if i_add {
            motor_data_send.send(MotorDataForIPC{power: power, id: motor_id}).await?;
        }
    }
}

pub async fn motor_data_send(
    process_closer: Receiver<bool>,
    motor_data_rcv: Receiver<MotorDataForIPC>,
    publisher: Publisher<mc_msgs::msg::Data>,
)-> Result<(), DynError>{
    // --- Logger ---
    let logger = Logger::new("motor_data_send");
    pr_info!(logger, "start motor_data_send");

    // --- Get Signal ---
    let deadline = Duration::from_secs(1);

    // pub msg
    let mut pub_msg = mc_msgs::msg::Data::new().unwrap();
    let mut pub_ok = [false; 4];
    loop {
        match timeout(deadline, motor_data_rcv.recv()).await {
            Ok(sub_result) => match sub_result{
                Ok(data) => {
                    match data.id {
                        0 => {
                            pub_msg.motor_a = data.power;
                            pub_ok[0] = true;
                        }
                        1 => {
                            pub_msg.motor_b = data.power;
                            pub_ok[1] = true;
                        }
                        2 => {
                            pub_msg.motor_c = data.power;
                            pub_ok[2] = true;
                        }
                        3 => {
                            pub_msg.motor_d = data.power;
                            pub_ok[3] = true;
                        }
                        _ => {}
                    }

                    if pub_ok[0] && pub_ok[1] && pub_ok[2] && pub_ok[3] {
                        publisher.send(&pub_msg)?;
                        pub_ok[0] = false;
                        pub_ok[1] = false;
                        pub_ok[2] = false;
                        pub_ok[3] = false;
                    }
                }
                Err(_)=>{}
            },
            Err(_) => {
                if process_closer.try_recv() == Ok(true){
                    pr_info!(logger, "motor_data_send thread shutdown");
                    return Ok(());
                }
            }
        }
    }
}

pub async fn get_signal(closer: Sender<bool> ) -> Result<(), DynError>{
    let signals = Signals::new(&[SIGHUP, SIGTERM, SIGINT, SIGQUIT])?;
    let mut signals = signals.fuse();
    loop {
        if let Some(signal) = signals.next().await{
            match signal {
                SIGTERM | SIGINT | SIGQUIT => {
                    // Shutdown the system;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    async_std::task::sleep(Duration::from_millis(100)).await;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    return  Ok(());
                },
                _ => unreachable!(),
            }
        }
    }
}