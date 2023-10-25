use manual_mode_common::GetGamePadValue;
use safe_drive::{
    context::Context,
    error::DynError,
};

use ros2_rust_util::{get_str_parameter, get_f64_parameter};

use manual_mode_common::check_pram_button_name;

fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;

    let node = ctx.create_node("two_button", None, Default::default())?;

    let mut selector  = ctx.create_selector()?;

    let subscriber = node.create_subscriber::<remote_control_msgs::msg::Gamepad>("/input", None)?;

    let publisher = node.create_publisher::<actuator_control_msgs::msg::SingleAxis>("/output", None)?;

    //Parameter
    let posi_button = get_str_parameter(node.get_name(), "posi_button", "b");
    check_pram_button_name(posi_button.as_str());

    let nega_button = get_str_parameter(node.get_name(), "nega_button", "a");
    check_pram_button_name(nega_button.as_str());

    let plus_power = get_f64_parameter(node.get_name(), "plus_power", 1.0) as f32;

    let minus_power = get_f64_parameter(node.get_name(), "minus_power", 1.0) as f32;

    let mut send_msg = actuator_control_msgs::msg::SingleAxis::new().unwrap();

    selector.add_subscriber(
        subscriber, 
        Box::new(move |msg| {
            if msg.get_button(posi_button.as_str())
            {
                send_msg.power = 1.0 * plus_power;
            }
            else if msg.get_button(nega_button.as_str())
            {
                send_msg.power = -1.0 * minus_power;
            }
            else if !msg.get_button(posi_button.as_str()) || msg.get_button(nega_button.as_str()) {
                send_msg.power = 0.0;
            }

            let _ = publisher.send(&send_msg);
        })
    );

    loop {
        selector.wait()?;
    }
}