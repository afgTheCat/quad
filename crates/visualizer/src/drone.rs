use bevy::math::DVec3;
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use simulator::{
    low_pass_filter::LowPassFilter, BatteryModel, BatteryState, Drone, DroneFrameState, DroneModel,
    GyroModel, GyroState, RotorModel, RotorState, RotorsState, SampleCurve, SamplePoint,
    SimulationFrame,
};

// TODO: we should not rely on the mesh names for the simulation but the other way around, it would
// be nice to load and adjust the correct mesh to the simulations configuration
pub const PROP_BLADE_MESH_NAMES: [(&str, f64, DVec3); 4] = [
    (
        "prop_blade.001",
        -1.,
        DVec3::new(
            0.14055216312408447,
            0.013523973524570465,
            0.11647607386112213,
        ),
    ),
    (
        "prop_blade.002",
        1.,
        DVec3::new(
            0.14055214822292328,
            0.013523973524570465,
            -0.11647609621286392,
        ),
    ),
    (
        "prop_blade.003",
        1.,
        DVec3::new(
            -0.14055216312408447,
            0.013523973524570465,
            0.11647608131170273,
        ),
    ),
    (
        "prop_blade.004",
        -1.,
        DVec3::new(
            -0.14055216312408447,
            0.013523973524570465,
            -0.11647607386112213,
        ),
    ),
];

fn initial_simulation_frame() -> SimulationFrame {
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

    SimulationFrame {
        battery_state,
        rotors_state,
        drone_state,
        gyro_state,
    }
}

pub fn get_base_model() -> Drone {
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
    let initial_frame = initial_simulation_frame();

    Drone {
        current_frame: initial_frame.clone(),
        next_frame: initial_frame.clone(),
        battery_model: battery_model.clone(),
        rotor_model,
        drone_model,
        gyro_model,
    }
}
