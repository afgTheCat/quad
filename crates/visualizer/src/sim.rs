use std::{sync::Arc, time::Duration};

use bevy::{
    asset::{Assets, Handle},
    color::palettes::css::RED,
    gltf::{Gltf, GltfNode},
    input::{gamepad::GamepadEvent, keyboard::KeyboardInput, ButtonState},
    math::Quat,
    prelude::{
        Commands, Deref, DerefMut, EventReader, GamepadAxisType, Gizmos, KeyCode, NextState, Query,
        Res, ResMut, Resource, Transform,
    },
    scene::{Scene, SceneBundle},
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
    MotorInput, RotorModel, RotorState, RotorsState, SimulationDebugInfo, SimulationFrame,
    Simulator,
};

use crate::{ntb_mat3, ntb_vec3, ui::UiData, DroneAsset, VisualizerState, PROP_BLADE_MESH_NAMES};

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

// Since we currently only support a single simulation, we should use a resource for the drone and
// all the auxulary information. In the future, if we include a multi drone setup/collisions and
// other things, it might make sense to have entities/components
#[derive(Resource, Deref, DerefMut)]
pub struct Simulaton(Simulator);

/// Holds all the relevant data that the we wish to show.
// #[derive(Resource, Deref, Default)]
// pub struct DebugUiContent(SimulationDebugInfo);

/// When the drone asset is loaded, sets up the `Simulation` and sets the new `SimState` to
/// `SimState::Running`. It will also handle setting up the `DebugUiContent`, the
/// `PlayerControllerInput` and the spawns the scene.
pub fn setup_drone_simulation(
    mut commands: Commands,
    drone_asset: Res<DroneAsset>,
    gltf_assets: Res<Assets<Gltf>>,
    gltf_node_assets: Res<Assets<GltfNode>>,
    mut next_state: ResMut<NextState<VisualizerState>>,
) {
    let Some(gltf) = gltf_assets.get(&drone_asset.0) else {
        return;
    };
    let flight_controller = Arc::new(BFController::new());
    flight_controller.init();

    let rotors_state = RotorsState(PROP_BLADE_MESH_NAMES.map(|(name, rotor_dir)| {
        let node_id = gltf.named_nodes[name].id();
        let prop_asset_node = gltf_node_assets.get(node_id).unwrap().clone();
        let position = prop_asset_node.transform.translation.as_dvec3();
        // guess its fine
        RotorState {
            current: 0.,
            rpm: 0.,
            motor_torque: 0.,
            effective_thrust: 0.,
            pwm: 0.,
            rotor_dir,
            motor_pos: Vector3::new(position.x, position.y, position.z),
        }
    }));

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
    };

    let initial_frame = SimulationFrame {
        battery_state,
        rotors_state,
        drone_state,
        gyro_state,
    };

    let bat_voltage_curve = SampleCurve::new(vec![
        SamplePoint::new(-0.06, 4.4),
        SamplePoint::new(0.0, 4.2),
        SamplePoint::new(0.01, 4.05),
        SamplePoint::new(0.04, 3.97),
        SamplePoint::new(0.30, 3.82),
        SamplePoint::new(0.40, 3.7),
        SamplePoint::new(1.0, 3.49),
        SamplePoint::new(1.01, 3.4),
        SamplePoint::new(1.03, 3.3),
        SamplePoint::new(1.06, 3.0),
        SamplePoint::new(1.08, 0.0),
    ]);

    let battery_model = BatteryModel {
        quad_bat_capacity: 850.,
        bat_voltage_curve,
        quad_bat_cell_count: 4,
        quad_bat_capacity_charged: 850.,
        max_voltage_sag: 1.4,
    };

    let rotor_model = RotorModel {
        prop_max_rpm: 36000.0,
        pwm_low_pass_filter: [
            LowPassFilter::default(),
            LowPassFilter::default(),
            LowPassFilter::default(),
            LowPassFilter::default(),
        ],
        motor_kv: 3200., // kv
        motor_r: 0.13,   // resistence
        motor_io: 0.23,  // idle current
        prop_thrust_factor: Vector3::new(-5e-05, -0.0025, 4.75),
        prop_torque_factor: 0.0056,
        prop_a_factor: 7.43e-10,
        prop_inertia: 3.5e-07,
    };

    let drone_model = DroneModel {
        frame_drag_area: Vector3::new(0.0082, 0.0077, 0.0082),
        frame_drag_constant: 1.45,
        mass: 0.2972,
        inv_tensor: Matrix3::from_diagonal(&Vector3::new(750., 5150.0, 750.0)),
    };

    let gyro_model = GyroModel {
        low_pass_filter: [
            LowPassFilter::default(),
            LowPassFilter::default(),
            LowPassFilter::default(),
        ],
    };

    let drone = Drone {
        current_frame: initial_frame.clone(),
        next_frame: initial_frame.clone(),
        battery_model,
        rotor_model,
        drone_model,
        gyro_model,
    };

    let logger = SimLogger::new(&"thing.csv", MotorInput::default());

    let simulation = Simulaton(Simulator {
        drone,
        flight_controller: flight_controller.clone(),
        time_accu: Duration::default(),
        time: Duration::new(0, 0),
        dt: Duration::from_nanos(5000), // TODO: update this
        fc_time_accu: Duration::default(),
        logger,
    });

    commands.insert_resource(simulation);

    // Insert the simulation debug info
    // commands.insert_resource(DebugUiContent::default());

    // Insert the player controller input
    commands.insert_resource(PlayerControllerInput::default());

    // Insert the scene bundle
    commands.spawn(SceneBundle {
        scene: gltf.scenes[0].clone(),
        ..Default::default()
    });

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
