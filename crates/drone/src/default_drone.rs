use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

use crate::{
    BatteryModel, BatteryState, Drone, DroneFrameState, DroneModel, GyroModel, GyroState,
    LowPassFilter, RotorModel, RotorState, RotorsState, SampleCurve, SamplePoint, SimulationFrame,
};

pub const PROP_BLADE_MESH_NAMES: [(&str, f64, Vector3<f64>); 4] = [
    (
        "prop_blade.001",
        -1.,
        Vector3::new(
            0.14055216312408447,
            0.013523973524570465,
            0.11647607386112213,
        ),
    ),
    (
        "prop_blade.002",
        1.,
        Vector3::new(
            0.14055214822292328,
            0.013523973524570465,
            -0.11647609621286392,
        ),
    ),
    (
        "prop_blade.003",
        1.,
        Vector3::new(
            -0.14055216312408447,
            0.013523973524570465,
            0.11647608131170273,
        ),
    ),
    (
        "prop_blade.004",
        -1.,
        Vector3::new(
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
        drone_frame_state: drone_state,
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

pub fn default_7in_4s_drone() -> Drone {
    let rotors_state = {
        let arm = 0.15_f64; // 15 cm
        let positions = [
            Vector3::new(arm, 0.0, arm),   // front-right
            Vector3::new(-arm, 0.0, arm),  // front-left
            Vector3::new(-arm, 0.0, -arm), // back-left
            Vector3::new(arm, 0.0, -arm),  // back-right
        ];
        let dirs = [1.0, -1.0, 1.0, -1.0];

        RotorsState([
            RotorState {
                current: 0.0,
                rpm: 0.0,
                motor_torque: 0.0,
                effective_thrust: 0.0,
                pwm: 0.0,
                rotor_dir: dirs[0],
                motor_pos: positions[0],
                pwm_low_pass_filter: LowPassFilter::new(0.0, 0.0),
            },
            RotorState {
                current: 0.0,
                rpm: 0.0,
                motor_torque: 0.0,
                effective_thrust: 0.0,
                pwm: 0.0,
                rotor_dir: dirs[1],
                motor_pos: positions[1],
                pwm_low_pass_filter: LowPassFilter::new(0.0, 0.0),
            },
            RotorState {
                current: 0.0,
                rpm: 0.0,
                motor_torque: 0.0,
                effective_thrust: 0.0,
                pwm: 0.0,
                rotor_dir: dirs[2],
                motor_pos: positions[2],
                pwm_low_pass_filter: LowPassFilter::new(0.0, 0.0),
            },
            RotorState {
                current: 0.0,
                rpm: 0.0,
                motor_torque: 0.0,
                effective_thrust: 0.0,
                pwm: 0.0,
                rotor_dir: dirs[3],
                motor_pos: positions[3],
                pwm_low_pass_filter: LowPassFilter::new(0.0, 0.0),
            },
        ])
    };

    let drone_frame_state = DroneFrameState {
        position: Vector3::zeros(),
        rotation: Rotation3::identity(),
        linear_velocity: Vector3::zeros(),
        angular_velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };

    let gyro_state = GyroState {
        rotation: UnitQuaternion::default(),
        acceleration: Vector3::zeros(),
        angular_velocity: Vector3::zeros(),
        low_pass_filters: [
            LowPassFilter::new(0.0, 0.0),
            LowPassFilter::new(0.0, 0.0),
            LowPassFilter::new(0.0, 0.0),
        ],
    };

    let current_frame = SimulationFrame {
        battery_state: BatteryState {
            capacity: 3000.0,  // mAh remaining (start full)
            bat_voltage: 16.8, // pack open-circuit at full charge
            bat_voltage_sag: 16.5,
            amperage: 0.0,
            m_ah_drawn: 0.0,
        },
        rotors_state,
        drone_frame_state,
        gyro_state,
    };
    let next_frame = current_frame.clone();

    let bat_curve = SampleCurve {
        sample_points: vec![
            SamplePoint {
                discharge: 0.00,
                voltage: 4.20,
            },
            SamplePoint {
                discharge: 0.20,
                voltage: 3.95,
            },
            SamplePoint {
                discharge: 0.50,
                voltage: 3.70,
            },
            SamplePoint {
                discharge: 0.80,
                voltage: 3.50,
            },
            SamplePoint {
                discharge: 1.00,
                voltage: 3.30,
            },
        ],
        min_discharge_point: SamplePoint {
            discharge: 1.0,
            voltage: 3.30,
        },
        max_discharge_point: SamplePoint {
            discharge: 0.0,
            voltage: 4.20,
        },
    };

    let battery_model = BatteryModel {
        quad_bat_capacity: 3000.0,
        bat_voltage_curve: bat_curve,
        quad_bat_cell_count: 4,
        quad_bat_capacity_charged: 3000.0,
        max_voltage_sag: 0.7,
    };

    let prop_a = 1.1e-8_f64;
    let prop_max_rpm = 26_000.0;
    let prop_f_at_v0 = prop_a * prop_max_rpm * prop_max_rpm;

    let rotor_model = RotorModel {
        prop_max_rpm,
        pwm_low_pass_filter: [
            LowPassFilter::new(0.0, 0.0),
            LowPassFilter::new(0.0, 0.0),
            LowPassFilter::new(0.0, 0.0),
            LowPassFilter::new(0.0, 0.0),
        ],
        motor_kv: 1700.0,
        motor_r: 0.12,
        motor_io: 1.0,
        prop_thrust_factor: Vector3::new(0.0, 0.0, prop_f_at_v0),
        prop_torque_factor: 0.025,
        prop_a_factor: prop_a,
        prop_inertia: 5.0e-5,
    };

    let mass = 0.9_f64;
    let l = 0.15_f64;
    let i_xx = mass * l * l;
    let i_yy = 2.0 * mass * l * l;
    let i_zz = mass * l * l;

    let inv_tensor = Matrix3::from_diagonal(&Vector3::new(1.0 / i_xx, 1.0 / i_yy, 1.0 / i_zz));

    let drone_model = DroneModel {
        frame_drag_area: Vector3::new(0.02, 0.03, 0.02),
        frame_drag_constant: 1.1,
        mass,
        inv_tensor,
    };

    let gyro_model = GyroModel {};

    Drone {
        current_frame,
        next_frame,
        battery_model,
        rotor_model,
        drone_model,
        gyro_model,
    }
}
