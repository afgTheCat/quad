use crate::{
    drone::get_base_model,
    ntb_mat3, ntb_vec3,
    state::{Controller, Logger, SelectionConfig, VisualizerData},
    DB,
};
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
use flight_controller::{
    controllers::{
        bf_controller::BFController, bf_controller2::BFController2, res_controller::ResController,
    },
    Channels, FlightController,
};
use nalgebra::Vector3;
use simulator::loggers::{Logger as LoggerTrait, RerunLogger};
use simulator::{loggers::DBLogger, BatteryUpdate, MotorInput, Simulator};
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};
use uuid::Uuid;

// Since we currently only support a single simulation, we should use a resource for the drone and
// all the auxulary information. In the future, if we include a multi drone setup/collisions and
// other things, it might make sense to have entities/components
#[derive(Resource, Deref, DerefMut)]
pub struct Simulaton(Simulator);

/// Acts as storage for the controller inputs. Controller inputs are used as setpoints for the
/// controller. We are storing them since it's not guaranteed that a new inpout will be sent on
/// each frame.
/// TODO: do we even need this? I assume that betaflight will handle storing the inputs.
#[derive(Resource, Deref, DerefMut, Default, Debug)]
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
    mut sim_data: ResMut<VisualizerData>,
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

// TODO: set it up according to the menu
pub fn enter_simulation(mut commands: Commands, sim_data: ResMut<VisualizerData>, db: Res<DB>) {
    let simulation_id = Uuid::new_v4().to_string();
    let drone = get_base_model();
    let SelectionConfig::Simulation {
        logger: Some(logger),
        controller: Some(controller),
    } = &sim_data.selection_config
    else {
        unreachable!()
    };

    let flight_controller: Arc<dyn FlightController> = match controller {
        // Controller::Betafligt => Arc::new(BFController::new()),
        Controller::Betafligt => Arc::new(BFController2::new()),
        Controller::Reservoir(res_id) => Arc::new(ResController::from_db(&db, &res_id)),
    };
    let battery_state = &drone.current_frame.battery_state;

    let logger: Arc<Mutex<dyn LoggerTrait>> = match logger {
        Logger::DBLogger => Arc::new(Mutex::new(DBLogger::new(
            simulation_id,
            MotorInput::default(),
            BatteryUpdate {
                bat_voltage_sag: battery_state.bat_voltage_sag,
                bat_voltage: battery_state.bat_voltage,
                amperage: battery_state.amperage,
                m_ah_drawn: battery_state.m_ah_drawn,
                cell_count: drone.battery_model.quad_bat_cell_count,
            },
            drone.current_frame.gyro_state.gyro_update(),
            Channels::default(),
            db.clone(),
        ))),
        Logger::Rerun => Arc::new(Mutex::new(RerunLogger::new(simulation_id))),
    };
    let mut simulation = Simulaton(Simulator {
        drone: drone.clone(),
        flight_controller: flight_controller.clone(),
        time_accu: Duration::default(),
        time: Duration::new(0, 0),
        dt: Duration::from_nanos(5000), // TODO: update this
        fc_time_accu: Duration::default(),
        logger,
    });

    simulation.init();
    commands.insert_resource(simulation);
    commands.insert_resource(PlayerControllerInput::default());
}

pub fn exit_simulation(
    simulation: ResMut<Simulaton>,
    db: Res<DB>,
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
    simulation.write_remaining_logs();

    // Remove the simulation
    commands.remove_resource::<Simulaton>();
    commands.remove_resource::<PlayerControllerInput>();
}
