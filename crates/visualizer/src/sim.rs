use std::{io::Write, sync::Arc, time::Duration};

use bevy::{
    asset::{Assets, Handle},
    color::palettes::css::RED,
    gltf::{Gltf, GltfNode},
    input::{gamepad::GamepadEvent, keyboard::KeyboardInput, ButtonState},
    math::{Quat, Vec3, VectorSpace},
    prelude::{
        Commands, Deref, DerefMut, EventReader, GamepadAxisType, Gizmos, KeyCode, NextState, Query,
        Res, ResMut, Resource, Transform,
    },
    scene::{ron::de::Position, Scene, SceneBundle},
    time::Time,
};
// use bevy_egui::{egui::Window as EguiWindow, EguiContexts};
use bevy_panorbit_camera::PanOrbitCamera;
// use egui_extras::{Column, TableBuilder};
use flight_controller::{controllers::bf_controller::BFController, Channels, FlightController};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use simulator::{
    logger::SimLogger,
    low_pass_filter::LowPassFilter,
    sample_curve::{SampleCurve, SamplePoint},
    BatteryModel, BatteryState, Drone, DroneFrameState, DroneModel, GyroModel, GyroState,
    MotorInput, RotorModel, RotorState, RotorsState, SimulationFrame, Simulator,
};

use crate::{
    ntb_mat3, ntb_vec3, ui::UiData, DroneAsset, Simulaton, VisualizerState, PROP_BLADE_MESH_NAMES,
};

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

/// When the drone asset is loaded, sets up the `Simulation` and sets the new `SimState` to
/// `SimState::Running`. It will also handle setting up the `DebugUiContent`, the
/// `PlayerControllerInput` and the spawns the scene.
pub fn setup_drone_simulation(
    mut commands: Commands,
    drone_asset: Res<DroneAsset>,
    gltf_assets: Res<Assets<Gltf>>,
    simulation: ResMut<Simulaton>,
    mut next_state: ResMut<NextState<VisualizerState>>,
) {
    let Some(gltf) = gltf_assets.get(&drone_asset.0) else {
        return;
    };

    // Insert the scene bundle
    commands.spawn(SceneBundle {
        scene: gltf.scenes[0].clone(),
        ..Default::default()
    });

    simulation.init();

    // Set next state
    next_state.set(VisualizerState::Simulation);
}

/// The simulation loop.
pub fn sim_loop(
    mut gizmos: Gizmos,
    timer: Res<Time>,
    mut ui_info: ResMut<UiData>,
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

    ui_info.set_sim_info(debug_info);
    camera.target_focus = drone_translation;
}

pub fn reset_drone_simulation(
    mut simulation: ResMut<Simulaton>,
    mut scene_query: Query<(&mut Transform, &Handle<Scene>)>,
) {
    let rotors_state =
        RotorsState(
            PROP_BLADE_MESH_NAMES.map(|(name, rotor_dir, position)| RotorState {
                current: 0.,
                rpm: 0.,
                motor_torque: 0.,
                effective_thrust: 0.,
                pwm: 0.,
                rotor_dir,
                motor_pos: Vector3::new(position.x, position.y, position.z),
                pwm_low_pass_filter: LowPassFilter::default(),
            }),
        );

    let battery_state = BatteryState {
        capacity: 850.,
        bat_voltage: 4.2,
        bat_voltage_sag: 4.2,
        amperage: 0.,
        m_ah_drawn: 0.,
    };

    let drone_state = DroneFrameState {
        position: Vector3::zeros(),
        rotation: Rotation3::identity(), // stargin position
        linear_velocity: Vector3::zeros(),
        angular_velocity: Vector3::new(0., 0., 0.),
        acceleration: Vector3::zeros(),
    };

    let gyro_state = GyroState {
        rotation: UnitQuaternion::identity(),
        acceleration: Vector3::zeros(),
        angular_velocity: Vector3::zeros(),
        low_pass_filters: [
            LowPassFilter::default(),
            LowPassFilter::default(),
            LowPassFilter::default(),
        ],
    };

    let initial_frame = SimulationFrame {
        battery_state,
        rotors_state,
        drone_state,
        gyro_state,
    };

    simulation.reset(initial_frame);
    let (mut tranform, _) = scene_query.single_mut();
    tranform.rotation = Quat::IDENTITY;
    tranform.translation = Vec3::ZERO;
}
