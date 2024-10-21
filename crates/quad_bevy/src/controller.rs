use bevy::{
    input::gamepad::GamepadEvent,
    prelude::{Component, EventReader},
};

#[derive(Component)]
struct Controller {}

fn gamepad_input_events(mut evr_gamepad: EventReader<GamepadEvent>) {
    for ev in evr_gamepad.read() {
        match ev {
            GamepadEvent::Axis(ev_axis) => {
                println!(
                    "Axis {:?} on gamepad {:?} is now at {:?}",
                    ev_axis.axis_type, ev_axis.gamepad, ev_axis.value
                );
            }
            GamepadEvent::Button(ev_button) => {
                // The "value" of a button is typically `0.0` or `1.0`, but it
                // is a `f32` because some gamepads may have buttons that are
                // pressure-sensitive or otherwise analog somehow.
                println!(
                    "Button {:?} on gamepad {:?} is now at {:?}",
                    ev_button.button_type, ev_button.gamepad, ev_button.value
                );
            }
            _ => {
                // we don't care about other events here (connect/disconnect)
            }
        }
    }
}
