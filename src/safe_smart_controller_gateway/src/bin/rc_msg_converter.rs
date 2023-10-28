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

use safe_drive::{
    context::Context, 
    error::DynError, 
    logger::Logger,
    pr_info,
    pr_error,
};

use std::ptr;

fn deserializex<T: Sized>(buf: &[u8]) -> T {
    let obj_ptr = buf.as_ptr() as *const T;
    unsafe { ptr::read(obj_ptr) }
}

fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("rc_msg_converter", None, Default::default())?;

    let subscriber = node.create_subscriber::<scgw_msgs::msg::Data>("scgw_raw", None)?;
    let publisher = node.create_publisher::<remote_control_msgs::msg::Gamepad>("gamepad", None)?;
    let mut selector = ctx.create_selector()?;

    // Create a logger.
    let logger = Logger::new("scgw");
    pr_info!(logger, "start node: {:?}", node.get_name());

    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            if msg.id != 128 {
                return;  
            }

            let u8_data = msg.data.as_slice();

            if u8_data.len() < 38{
                pr_error!(logger, "msg is small {:?}", u8_data.len());
                return;
            }

            let mut pub_msg = remote_control_msgs::msg::Gamepad::new().unwrap();

            pub_msg.left_joystic = deserializex(&u8_data[0..8]);
            pub_msg.right_joystic = deserializex(&u8_data[9..17]);
            pub_msg.left_trigger = deserializex(&u8_data[18..22]);
            pub_msg.right_trigger = deserializex(&u8_data[23..27]);
            pub_msg.dpad = deserializex(&u8_data[28..31]);
            pub_msg.button = deserializex(&u8_data[32..35]);
            pub_msg.left_shoulder_button = deserializex(&u8_data[36..36]);
            pub_msg.right_shoulder_button = deserializex(&u8_data[37..37]);

            let _ = publisher.send(&pub_msg);
        }),
    );

    loop {
        selector.wait()?;
    }
}