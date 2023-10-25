use std::io::prelude::*;
use std::time::Duration;

use ros2_rust_util::{get_i64_parameter, get_str_parameter};
use safe_drive::{context::Context, topic::{subscriber::Subscriber, publisher::Publisher}, error::DynError, logger::Logger, pr_info, pr_error};
use safe_drive::msg::U8Seq;
use serialport::SerialPort;

#[async_std::main]
async fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("safe_serial_bridge", None, Default::default())?;
    let subscriber = node.create_subscriber::<std_msgs::msg::UInt8MultiArray>("serial_write", None)?;
    let publisher = node.create_publisher::<std_msgs::msg::UInt8MultiArray>("serial_read", None)?;

    // ---- Create a logger ----
    let logger = Logger::new("safe_serial_bridge");
    pr_info!(logger, "Start {:?}", node.get_name());

    // ---- Parameter ----
    let port_name_read = get_str_parameter(node.get_name(), "port", "/dev/cu.usbmodem2103");
    let baud_late = get_i64_parameter(node.get_name(), "baud_late", 115200);

    // --- Serial ----
    let serial_port = serialport::new(port_name_read, baud_late as u32)
        .timeout(Duration::from_millis(100))
        .open()?;

    let subscriber_task = async_std::task::spawn(thread_subscriber(subscriber, serial_port.try_clone()?));
    let publisher_task = async_std::task::spawn(thread_publisher(publisher,serial_port));

    subscriber_task.await?;
    publisher_task.await?;
    Ok(())
}


pub async fn thread_subscriber(
    mut subscriber: Subscriber<std_msgs::msg::UInt8MultiArray>,
    mut serialport: Box<dyn SerialPort>
) -> Result<(), DynError> {
    let logger = Logger::new(subscriber.get_topic_name());

    loop {
        // Receive a message
        let msg = subscriber.recv().await?;
        match serialport.write(msg.data.as_slice()) {
            Ok(_) => {
                if let Err(e) = std::io::stdout().flush() {
                    pr_error!(logger, "Failed to flush stdout: {:?}" , e)
                }
            },
            Err(e) => pr_error!(logger, "{:?}" , e),
        }
    }
}

pub async fn thread_publisher(
    publisher: Publisher<std_msgs::msg::UInt8MultiArray>,
    mut serialport: Box<dyn SerialPort>
)-> Result<(), DynError>{
    let logger = Logger::new(publisher.get_topic_name());
    let mut buf: Vec<u8> = vec![0; 1000];

    loop {
        // Receive a message
        match serialport.read(buf.as_mut_slice()) {
            Ok(t) => {
                let bytes = &buf[..t];
                let mut msg = std_msgs::msg::UInt8MultiArray::new().unwrap();
                let mut data = U8Seq::new(t).unwrap();
                data.as_slice_mut().copy_from_slice(bytes);
                msg.data = data;
                publisher.send(&msg)?;
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => (),
            Err(e) => pr_error!(logger, "{:?}", e),
        }
    }
}