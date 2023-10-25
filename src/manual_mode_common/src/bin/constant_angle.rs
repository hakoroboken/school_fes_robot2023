use safe_drive::{
    context::Context, 
    error::DynError, 
    logger::Logger,
    pr_info,
};

use ros2_rust_util::{get_str_parameter, get_bool_parameter, get_f64_parameter};
use manual_mode_common::{check_pram_value_name, GetGamePadValue , Invert};

fn main() -> Result<(), DynError> {
    
    let ctx = Context::new()?;
    let node = ctx.create_node("single_axis_node", None, Default::default())?;

    let subscriber = node.create_subscriber::<remote_control_msgs::msg::Gamepad>("/input", None)?;
    let publisher_single_axis = node.create_publisher::<actuator_control_msgs::msg::SingleAxis>("/output", None)?;

    let logger = Logger::new("single_axis_node");
    pr_info!(logger, "start node: {:?}", node.get_name());

    let mut selector = ctx.create_selector()?;

    // Parameters
    let param_value_name = get_str_parameter(node.get_name(), "get_value", "trigger.right");
    check_pram_value_name(param_value_name.as_str());
    let param_reversal = get_bool_parameter(node.get_name(), "reversal", false);

    let param_start_value = get_f64_parameter(node.get_name(), "start_value", -1.0) as f32;
    let param_end_value = get_f64_parameter(node.get_name(), "end_value", 1.0) as f32;

    let mut now_power = param_start_value;

    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            let mut send_msg = actuator_control_msgs::msg::SingleAxis::new().unwrap();
            send_msg.power = 0.0;

            if msg.get_value(param_value_name.as_str()) > now_power
            {
                now_power = msg.get_value(param_value_name.as_str());
                send_msg.power = 1.0;
            }


            if param_reversal {
                send_msg.power.invert();
            }
            
            let _ = publisher_single_axis.send(&send_msg);

            if now_power == param_end_value
            {
                now_power = param_start_value;
            }
        }),
    );

    loop {
        selector.wait()?;
    }
}