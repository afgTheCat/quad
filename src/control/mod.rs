use bevy::{
    input::{gamepad::GamepadEvent, keyboard::KeyboardInput},
    prelude::*,
};

#[derive(Debug, Resource)]
struct ControllerPosition(Vec4);

impl ControllerPosition {
    fn set_throttle(&mut self, value: f32) {
        self.0[0] = value;
    }

    fn set_yaw(&mut self, value: f32) {
        self.0[1] = value;
    }

    fn set_pitch(&mut self, value: f32) {
        self.0[2] = value;
    }

    fn set_roll(&mut self, value: f32) {
        self.0[3] = value;
    }
}

fn handle_controller_event(
    controller: &mut ResMut<ControllerPosition>,
    gamepar_event: &GamepadEvent,
) {
    match gamepar_event {
        GamepadEvent::Axis(axis) => {
            let value = axis.value;
            match axis.axis_type {
                GamepadAxisType::RightStickY => {
                    controller.set_throttle(value);
                }
                GamepadAxisType::RightStickX => {
                    controller.set_yaw(value);
                }
                GamepadAxisType::LeftStickY => {
                    controller.set_pitch(value);
                }
                GamepadAxisType::LeftStickX => {
                    controller.set_roll(value);
                }
                _ => {}
            }
        }
        GamepadEvent::Button(but) => {
            println!("{:?}", but.button_type);
            println!("{:?}", but.value);
        }
        _ => {}
    }
    println!("{:?}", controller);
}

pub fn gamepad_events(
    mut controller_pos: ResMut<ControllerPosition>,
    mut evr_gpd: EventReader<GamepadEvent>,
) {
    // let mut axis = Vec4::new(0., 0., 0., 0.);
    for ev in evr_gpd.read() {
        handle_controller_event(&mut controller_pos, ev);
    }
}

fn handle_keyboard_event2(mut camera_query: Query<&mut Camera>, toggle: bool) {
    let mut camera = camera_query.single_mut();
    if toggle {
        camera.is_active = !camera.is_active;
    }
}

pub fn handle_keyboard_events(
    camera_query: Query<&mut Camera>,
    mut evr_kbd: EventReader<KeyboardInput>,
) {
    let mut toggle = false;
    for ev in evr_kbd.read() {
        if let KeyCode::Space = ev.key_code {
            toggle = true;
        }
    }
    handle_keyboard_event2(camera_query, toggle);
}
