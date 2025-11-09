use crate::{ntb_mat3, ntb_vec3, Context};
use bevy::{
    asset::Handle,
    color::palettes::css::RED,
    input::{gamepad::GamepadEvent, keyboard::KeyboardInput, ButtonState},
    math::{Quat, Vec3},
    prelude::{
        Commands, Deref, DerefMut, EventReader, GamepadAxisType, Gizmos, KeyCode, Query, Res,
        ResMut, Resource, Transform,
    },
    scene::Scene,
    time::Time,
};
use bevy_panorbit_camera::PanOrbitCamera;
use flight_controller::Channels;
use nalgebra::Vector3;
use simulator::SimulationObservation;
use simulator::Simulator;

// Since we currently only support a single simulation, we should use a resource for the drone and
// all the auxulary information. In the future, if we include a multi drone setup/collisions and
// other things, it might make sense to have entities/components
#[derive(Resource, Deref, DerefMut)]
pub struct Simulation(Simulator);

#[derive(Resource, Default, Debug)]
pub struct SimulationData {
    pub channels: Channels,
    pub sim_info: SimulationObservation,
}

/// Handles the input.
pub fn handle_input(
    mut evr_gamepad: EventReader<GamepadEvent>,
    mut evr_kbd: EventReader<KeyboardInput>, // TODO: just for debugging
    mut sim_data: ResMut<SimulationData>,
) {
    for ev in evr_gamepad.read() {
        let &GamepadEvent::Axis(ax) = &ev else {
            continue;
        };

        let ax_val = ax.value as f64;

        match ax.axis_type {
            GamepadAxisType::LeftZ => sim_data.channels.throttle = ax_val,
            GamepadAxisType::RightStickX => {
                sim_data.channels.yaw = if ax_val > -0.96 { ax_val } else { -1. }
            }
            GamepadAxisType::LeftStickX => sim_data.channels.roll = ax_val,
            GamepadAxisType::LeftStickY => sim_data.channels.pitch = -ax_val,
            _ => {}
        }
    }

    for ev in evr_kbd.read() {
        if let (KeyCode::Space, ButtonState::Pressed) = (ev.key_code, ev.state) {
            sim_data.channels.throttle = 1.;
        }

        if let (KeyCode::Space, ButtonState::Released) = (ev.key_code, ev.state) {
            sim_data.channels.throttle = -1.;
        }

        if let (KeyCode::ArrowLeft, ButtonState::Pressed) = (ev.key_code, ev.state) {
            sim_data.channels.yaw -= 0.01;
        }

        if let (KeyCode::ArrowRight, ButtonState::Pressed) = (ev.key_code, ev.state) {
            sim_data.channels.yaw += 0.01;
        }
    }
}

/// The simulation loop.
pub fn sim_loop(
    mut gizmos: Gizmos,
    timer: Res<Time>,
    mut simulation: ResMut<Simulation>,
    mut sim_data: ResMut<SimulationData>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    mut scene_query: Query<(&mut Transform, &Handle<Scene>)>,
) {
    let (mut tranform, _) = scene_query.single_mut();
    let mut camera = camera_query.single_mut();
    let debug_info = simulation.simulate_delta(timer.delta(), sim_data.channels);

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

    sim_data.sim_info = debug_info;
    camera.target_focus = drone_translation;
}

// TODO: set it up according to the menu
pub fn enter_simulation(mut commands: Commands, mut context: ResMut<Context>) {
    let mut simulation = context.try_load_simulator().unwrap();
    simulation.init();
    commands.insert_resource(Simulation(simulation));
    commands.insert_resource(SimulationData::default());
}

pub fn exit_simulation(
    mut scene_query: Query<(&mut Transform, &Handle<Scene>)>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    mut commands: Commands,
) {
    // reset transform
    let (mut tranform, _) = scene_query.single_mut();
    let mut camera = camera_query.single_mut();
    tranform.rotation = Quat::IDENTITY;
    tranform.translation = Vec3::ZERO;
    camera.target_focus = tranform.translation;

    // Write the logs, should be a trait eventually
    // simulation.write_remaining_logs();

    // Remove the simulation
    commands.remove_resource::<Simulation>();
    commands.remove_resource::<SimulationData>();
}
