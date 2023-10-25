use safe_drive::{
    context::Context,
    error::DynError, 
};

use ros2_rust_util::get_f64_parameter;

fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;

    let node = ctx.create_node("mc_another_wheel", None, Default::default())?;

    let subscriber = node.create_subscriber::<actuator_control_msgs::msg::MecanumWheel>("/input", None)?;

    let publisher_fl = node.create_publisher::<actuator_control_msgs::msg::RawMotor>("/output_fl", None)?;
    let publisher_fr = node.create_publisher::<actuator_control_msgs::msg::RawMotor>("/output_fr", None)?;
    let publisher_ll = node.create_publisher::<actuator_control_msgs::msg::RawMotor>("/output_ll", None)?;
    let publisher_lr = node.create_publisher::<actuator_control_msgs::msg::RawMotor>("/output_lr", None)?;

    let mut selector = ctx.create_selector()?;

    // parameters
    let fl_power_rate = get_f64_parameter(node.get_name(), "fl_power", 0.5) as f32;

    let fr_power_rate = get_f64_parameter(node.get_name(), "fr_power", 0.5) as f32;

    let ll_power_rate = get_f64_parameter(node.get_name(), "ll_power", 0.5) as f32; 

    let lr_power_rate = get_f64_parameter(node.get_name(), "lr_power", 0.5) as f32;

    selector.add_subscriber(
        subscriber, 
        Box::new(move |msg| {
            let mut send_msg_fl = actuator_control_msgs::msg::RawMotor::new().unwrap();
            let mut send_msg_fr = actuator_control_msgs::msg::RawMotor::new().unwrap();
            let mut send_msg_ll = actuator_control_msgs::msg::RawMotor::new().unwrap();
            let mut send_msg_lr = actuator_control_msgs::msg::RawMotor::new().unwrap();
            send_msg_fl.power =
                0.707106781 * msg.vec_x - 0.707106781 * msg.vec_y + msg.rotation_power;
            send_msg_fr.power =
                0.707106781 * msg.vec_x - 0.707106781 * msg.vec_y - msg.rotation_power;
            send_msg_ll.power =
                -0.707106781 * msg.vec_x - 0.707106781 * msg.vec_y + msg.rotation_power;
            send_msg_lr.power =
                -0.707106781 * msg.vec_x - 0.707106781 * msg.vec_y - msg.rotation_power;
            send_msg_fl.power *= -1.0 * (2.0 * fl_power_rate);
            send_msg_fr.power *= -1.0 * (2.0 * fr_power_rate);
            send_msg_ll.power *= -1.0 * (2.0 * ll_power_rate);
            send_msg_lr.power *= -1.0 * (2.0 * lr_power_rate);
            let _ = publisher_fl.send(&send_msg_fl);
            let _ = publisher_fr.send(&send_msg_fr);
            let _ = publisher_ll.send(&send_msg_ll);
            let _ = publisher_lr.send(&send_msg_lr);
        }),
    );

    loop {
        selector.wait()?;
    }
}
