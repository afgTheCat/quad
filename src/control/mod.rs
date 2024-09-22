use bevy::{
    input::{keyboard::KeyboardInput, ButtonState},
    prelude::*,
};

pub fn handle_keyboard_events(
    mut camera_query: Query<(&mut Camera, &mut Transform)>,
    mut evr_kbd: EventReader<KeyboardInput>,
) {
    let mut moving_dir = Vec3::new(0., 0., 0.);
    for ev in evr_kbd.read() {
        match (ev.key_code, ev.state) {
            (KeyCode::ArrowLeft, ButtonState::Pressed) => moving_dir[0] -= 1.,
            (KeyCode::ArrowRight, ButtonState::Pressed) => moving_dir[0] += 1.,
            (KeyCode::ArrowUp, ButtonState::Pressed) => moving_dir[2] -= 1.,
            (KeyCode::ArrowDown, ButtonState::Pressed) => moving_dir[2] += 1.,
            _ => {}
        }
    }

    let (_, mut camera_transform) = camera_query.single_mut();
    camera_transform.translation += moving_dir;
}
