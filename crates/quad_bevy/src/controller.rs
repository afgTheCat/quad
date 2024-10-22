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
            GamepadAxisType::LeftZ => controller.set_throttle(ax_val),
            // GamepadAxisType::RightStickX => controller.set_yaw(ax_val),
            // GamepadAxisType::LeftStickX => controller.set_roll(ax_val),
            GamepadAxisType::LeftStickY => controller.set_pitch(ax_val),
            _ => {}
        }
    }
}
