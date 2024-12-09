use bevy::{
    asset::{Assets, Handle},
    color::palettes::css::RED,
    gltf::{Gltf, GltfNode},
    math::Quat,
    prelude::{
        Commands, Deref, DerefMut, Gizmos, NextState, Query, Res, ResMut, Resource, Transform,
    },
    scene::{Scene, SceneBundle},
    time::Time,
};
use bevy_panorbit_camera::PanOrbitCamera;
use csv::Reader;
use flight_controller::MotorInput;
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use simulator::{
    low_pass_filter::LowPassFilter, BatteryModel, BatteryState, Drone, DroneFrameState, DroneModel,
    GyroModel, GyroState, Replayer, RotorModel, RotorState, RotorsState, SampleCurve, SamplePoint,
    SimulationFrame,
};
use std::time::Duration;

use crate::{ntb_mat3, ntb_vec3, DroneAsset, VisualizerState, PROP_BLADE_MESH_NAMES};

#[derive(Resource, Deref, DerefMut)]
pub struct Replay(Replayer);

pub fn setup_drone_replay(
    mut commands: Commands,
    drone_asset: Res<DroneAsset>,
    gltf_assets: Res<Assets<Gltf>>,
    gltf_node_assets: Res<Assets<GltfNode>>,
    mut next_state: ResMut<NextState<VisualizerState>>,
) {
    let Some(gltf) = gltf_assets.get(&drone_asset.0) else {
        return;
    };

    // TODO: this whole drone setup should be serialized etc
    let rotors_state = RotorsState(PROP_BLADE_MESH_NAMES.map(|(name, rotor_dir, position)| {
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
            pwm_low_pass_filter: LowPassFilter::default(),
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

    let gyro_model = GyroModel {};

    let drone = Drone {
        current_frame: initial_frame.clone(),
        next_frame: initial_frame.clone(),
        battery_model,
        rotor_model,
        drone_model,
        gyro_model,
    };

    // TODO: suboptimal
    let data = Reader::from_path(&"thing.csv")
        .unwrap()
        .records()
        .map(|record| {
            let csv = record
                .unwrap()
                .into_iter()
                .map(|r| r.to_string())
                .collect::<Vec<_>>();
            let time_range = Duration::from_secs_f64(csv[0].parse().unwrap())
                ..Duration::from_secs_f64(csv[1].parse().unwrap());
            let motor_input = MotorInput {
                input: [
                    csv[2].parse().unwrap(),
                    csv[3].parse().unwrap(),
                    csv[4].parse().unwrap(),
                    csv[5].parse().unwrap(),
                ],
            };
            (time_range, motor_input)
        })
        .collect();

    let replay = Replay(Replayer {
        drone,
        time: Duration::new(0, 0),
        time_accu: Duration::new(0, 0),
        time_steps: data,
        replay_index: 0,
        dt: Duration::from_nanos(5000), // TODO: update this
    });

    commands.insert_resource(replay);

    commands.spawn(SceneBundle {
        scene: gltf.scenes[0].clone(),
        ..Default::default()
    });

    next_state.set(VisualizerState::Replay);
}

pub fn replay_loop(
    mut gizmos: Gizmos,
    timer: Res<Time>,
    mut replay: ResMut<Replay>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    mut scene_query: Query<(&mut Transform, &Handle<Scene>)>,
) {
    let (mut tranform, _) = scene_query.single_mut();
    let mut camera = camera_query.single_mut();

    let debug_info = replay.replay_delta(timer.delta());
    let drone_translation = ntb_vec3(debug_info.position);
    let drone_rotation = Quat::from_mat3(&ntb_mat3(debug_info.rotation));
    tranform.translation = drone_translation;
    tranform.rotation = drone_rotation;

    let down_dir = debug_info.rotation * Vector3::new(0., -1., -0.);
    let current_frame = &replay.drone.current_frame;
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
    camera.target_focus = drone_translation;
}
