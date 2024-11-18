use bevy::{
    input::gamepad::GamepadEvent,
    prelude::{EventReader, GamepadAxisType, Query},
};

use crate::Controller;

pub fn gamepad_input_events(
    mut evr_gamepad: EventReader<GamepadEvent>,
    mut controller: Query<&mut Controller>,
) {
    let mut controller = controller.single_mut();
    for ev in evr_gamepad.read() {
        let &GamepadEvent::Axis(ax) = &ev else {
            continue;
        };

        let ax_val = ax.value as f64;

        match ax.axis_type {
            GamepadAxisType::LeftZ => controller.throttle = ax_val,
            GamepadAxisType::RightStickX => {
                controller.yaw = if ax_val > -0.96 { ax_val } else { -1. }
            }
            GamepadAxisType::LeftStickX => controller.roll = ax_val,
            GamepadAxisType::LeftStickY => controller.pitch = -ax_val,
            _ => {}
        }
    }
}
