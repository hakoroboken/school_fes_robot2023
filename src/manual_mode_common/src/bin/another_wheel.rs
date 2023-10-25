use safe_drive::{
    context::Context,
    error::DynError, 
};

use ros2_rust_util::{get_str_parameter, get_f64_parameter};
use manual_mode_common::{check_pram_button_name, check_pram_value_name, GetGamePadValue};

fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;

    let node = ctx.create_node("another_wheel", None, Default::default())?;

    let publisher = node.create_publisher::<actuator_control_msgs::msg::MecanumWheel>("/output", None)?;

    let subscriber = node.create_subscriber::<remote_control_msgs::msg::Gamepad>("/input", None)?;

    let mut selector = ctx.create_selector()?;

    // parameters
    let rotation_name_param = get_str_parameter(node.get_name(), "rotation_name", "x.joystic.right");
    check_pram_value_name(rotation_name_param.as_str());

    let right_name_param = get_str_parameter(node.get_name(), "right_name", "right");
    check_pram_button_name(right_name_param.as_str());

    let left_name_param = get_str_parameter(node.get_name(), "left_name", "left");
    check_pram_button_name(left_name_param.as_str());

    let front_name_param = get_str_parameter(node.get_name(), "front_name", "up");
    check_pram_button_name(front_name_param.as_str());

    let back_name_param = get_str_parameter(node.get_name(), "back_name", "down");
    check_pram_button_name(back_name_param.as_str());

    let move_no_accel_power = get_f64_parameter(node.get_name(), "move_no_accel_power", 0.15) as f32;
    let rotation_power = get_f64_parameter(node.get_name(), "rotation_power", 0.15) as f32;


    selector.add_subscriber(
        subscriber, 
    Box::new(move |msg| {
            let mut send_msg = actuator_control_msgs::msg::MecanumWheel::new().unwrap();

            if msg.get_button(right_name_param.as_str())
            {
                send_msg.vec_x = 1.0 * (move_no_accel_power + (1.0 - move_no_accel_power) * (1.0 - msg.get_value("trigger.left")));
            }
            if msg.get_button(left_name_param.as_str())
            {
                send_msg.vec_x = -1.0 * (move_no_accel_power + (1.0 - move_no_accel_power) * (1.0 - msg.get_value("trigger.left")));
            }
            if msg.get_button(front_name_param.as_str())
            {
                send_msg.vec_y = 1.0 * (move_no_accel_power + (1.0 - move_no_accel_power) * (1.0 - msg.get_value("trigger.left")));
            }
            if msg.get_button(back_name_param.as_str())
            {
                send_msg.vec_y = -1.0 * (move_no_accel_power + (1.0 - move_no_accel_power) * (1.0 - msg.get_value("trigger.left")));
            }

            send_msg.rotation_power = msg.get_value(rotation_name_param.as_str()) * (move_no_accel_power + (1.0 - move_no_accel_power) * (msg.get_value("trigger.left") - 1.0)) * rotation_power;

            let _ = publisher.send(&send_msg);

        }),
    );

    loop {
        selector.wait()?;
    }
}