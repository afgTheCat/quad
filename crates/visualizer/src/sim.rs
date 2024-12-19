use crate::{initial_simulation_frame, ntb_mat3, ntb_vec3, ui::SimData, Simulaton, DB};
use bevy::{
    asset::Handle,
    color::palettes::css::RED,
    input::{gamepad::GamepadEvent, keyboard::KeyboardInput, ButtonState},
    math::{Quat, Vec3},
    prelude::{
        Deref, DerefMut, EventReader, GamepadAxisType, Gizmos, KeyCode, Query, Res, ResMut,
        Resource, Transform,
    },
    reflect::List,
    scene::Scene,
    time::Time,
};
use bevy_panorbit_camera::PanOrbitCamera;
use flight_controller::Channels;
use nalgebra::Vector3;
use uuid::Uuid;

/// Acts as storage for the controller inputs. Controller inputs are used as setpoints for the
/// controller. We are storing them since it's not guaranteed that a new inpout will be sent on
/// each frame.
/// TODO: do we even need this? I assume that betaflight will handle storing the inputs.
#[derive(Resource, Deref, DerefMut, Default)]
pub struct PlayerControllerInput(Channels);

impl PlayerControllerInput {
    fn to_channels(&self) -> Channels {
        self.0
    }
}

/// Handles the input.
pub fn handle_input(
    mut evr_gamepad: EventReader<GamepadEvent>,
    mut evr_kbd: EventReader<KeyboardInput>, // TODO: just for debugging
    mut controller_input: ResMut<PlayerControllerInput>,
) {
    for ev in evr_gamepad.read() {
        let &GamepadEvent::Axis(ax) = &ev else {
            continue;
        };

        let ax_val = ax.value as f64;

        match ax.axis_type {
            GamepadAxisType::LeftZ => controller_input.throttle = ax_val,
            GamepadAxisType::RightStickX => {
                controller_input.yaw = if ax_val > -0.96 { ax_val } else { -1. }
            }
            GamepadAxisType::LeftStickX => controller_input.roll = ax_val,
            GamepadAxisType::LeftStickY => controller_input.pitch = -ax_val,
            _ => {}
        }
    }

    for ev in evr_kbd.read() {
        if let (KeyCode::Space, ButtonState::Pressed) = (ev.key_code, ev.state) {
            controller_input.throttle = 1.;
        }

        if let (KeyCode::Space, ButtonState::Released) = (ev.key_code, ev.state) {
            controller_input.throttle = -1.;
        }

        if let (KeyCode::ArrowLeft, ButtonState::Pressed) = (ev.key_code, ev.state) {
            controller_input.yaw -= 0.01;
        }

        if let (KeyCode::ArrowRight, ButtonState::Pressed) = (ev.key_code, ev.state) {
            controller_input.yaw += 0.01;
        }
    }
}

/// The simulation loop.
pub fn sim_loop(
    mut gizmos: Gizmos,
    timer: Res<Time>,
    mut sim_data: ResMut<SimData>,
    mut simulation: ResMut<Simulaton>,
    controller_input: Res<PlayerControllerInput>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    mut scene_query: Query<(&mut Transform, &Handle<Scene>)>,
) {
    let (mut tranform, _) = scene_query.single_mut();
    let mut camera = camera_query.single_mut();
    let debug_info = simulation.simulate_delta(timer.delta(), controller_input.to_channels());

    let drone_translation = ntb_vec3(debug_info.position);
    let drone_rotation = Quat::from_mat3(&ntb_mat3(debug_info.rotation));
    tranform.translation = drone_translation;
    tranform.rotation = drone_rotation;

    let down_dir = debug_info.rotation * Vector3::new(0., -1., -0.);
    let current_frame = &simulation.drone.current_frame;
    for motor_index in 0..4 {
        let motor_pos = drone_translation
            + ntb_vec3(debug_info.rotation * current_frame.rotors_state.0[motor_index].motor_pos);
        gizmos.arrow(
            motor_pos,
            motor_pos
                + ntb_vec3(down_dir * current_frame.rotors_state.0[motor_index].effective_thrust),
            RED,
        );
    }

    sim_data.set_sim_info(debug_info);
    camera.target_focus = drone_translation;
}

pub fn enter_simulation(mut simulation: ResMut<Simulaton>) {
    let simulation_id = Uuid::new_v4().to_string();
    simulation.init(simulation_id);
}

pub fn exit_simulation(
    mut simulation: ResMut<Simulaton>,
    mut sim_data: ResMut<SimData>,
    db: Res<DB>,
    mut scene_query: Query<(&mut Transform, &Handle<Scene>)>,
    mut camera_query: Query<&mut PanOrbitCamera>,
) {
    let (mut tranform, _) = scene_query.single_mut();
    let mut camera = camera_query.single_mut();

    tranform.rotation = Quat::IDENTITY;
    tranform.translation = Vec3::ZERO;

    let simulation_id = simulation
        .logger
        .simulation_id
        .as_ref()
        .unwrap()
        .to_string();

    // write data
    db.write_flight_logs(&simulation_id, &simulation.logger.data);

    // insert siumulation id
    sim_data.simulation_ids.push(simulation_id);

    let initial_frame = initial_simulation_frame();
    simulation.reset(initial_frame);

    camera.target_focus = tranform.translation;
}
