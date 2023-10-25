use safe_drive::{
    context::Context, 
    error::DynError, 
    logger::Logger,
    pr_info,
};

fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("mc_catch_arm", None, Default::default())?;

    // ---- ros2 topic ----
    let subscriber = node.create_subscriber::<actuator_control_msgs::msg::LockArm>("/input", None)?;
    let publisher = node.create_publisher::<actuator_control_msgs::msg::RawMotor>("/output", None)?;
    
    // ---- Create a logger ----
    let logger = Logger::new("mc_catch_arm");
    pr_info!(logger, "start node: {:?}", node.get_name());

    // ---- Subscriber ----
    let mut selector = ctx.create_selector()?;

    let mut total_move = 0.0;
    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            let mut send_msg = actuator_control_msgs::msg::RawMotor::new().unwrap();
            send_msg.power = 0.0;
            if msg.power {
                send_msg.power = 1.0;
                total_move += 1.0;
            }
            else
            {
                if total_move > 0.0
                {
                    send_msg.power = -1.0;
                    total_move -= 1.0;
                    let _ = publisher.send(&send_msg);
                }
                else {
                    total_move = 0.0;
                }
            }

            let _ = publisher.send(&send_msg);
        }),
    );

    loop {
        selector.wait()?;
    }
}