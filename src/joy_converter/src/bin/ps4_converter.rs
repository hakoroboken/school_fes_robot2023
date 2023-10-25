use safe_drive::{
    error::DynError,
    logger::Logger,
    pr_info,
    context::Context,
    msg::common_interfaces::sensor_msgs,
};

fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;

    let node = ctx.create_node("ps4_converter", None, Default::default())?;

    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("/joy", None)?;

    let publisher = node.create_publisher::<remote_control_msgs::msg::Gamepad>("/scgw/gamepad", None)?;

    let log = Logger::new(node.get_name());

    let mut selector = ctx.create_selector()?;
    let mut send_msg = remote_control_msgs::msg::Gamepad::new().unwrap();

    selector.add_subscriber(
        subscriber, 
    Box::new(move |msg| {
        send_msg.left_joystic.x = *msg.axes.as_slice().get(0).unwrap();
        send_msg.left_joystic.y = *msg.axes.as_slice().get(1).unwrap();
        send_msg.right_joystic.x = *msg.axes.as_slice().get(4).unwrap();

        if *msg.axes.as_slice().get(6).unwrap() > 0.0
        {
            send_msg.dpad.up = true;
        }
        else if *msg.axes.as_slice().get(8).unwrap() < 0.0
        {
            send_msg.dpad.down = true;
        }
        else {
            send_msg.dpad.up = false;
            send_msg.dpad.down = false;
        }

        let _ = publisher.send(&send_msg);
    }),
    );

    loop {
        pr_info!(log, "start {}", node.get_name());
        selector.wait()?;
    }
}