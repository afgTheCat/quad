use bevy::math::DVec3;
use db::{
    simulation_frame::{DBLowPassFilter, DBRotorState},
    AscentDb,
};
use nalgebra::{Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector3};
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

fn db_to_rotor_state(db_rotor_state: DBRotorState, pwm_state: DBLowPassFilter) -> RotorState {
    RotorState {
        current: db_rotor_state.current,
        rpm: db_rotor_state.rpm,
        motor_torque: db_rotor_state.motor_torque,
        effective_thrust: db_rotor_state.effective_thrust,
        pwm: db_rotor_state.pwm,
        rotor_dir: db_rotor_state.rotor_dir,
        motor_pos: Vector3::new(
            db_rotor_state.motor_pos_x,
            db_rotor_state.motor_pos_y,
            db_rotor_state.motor_pos_z,
        ),
        pwm_low_pass_filter: LowPassFilter {
            output: pwm_state.output,
            e_pow: pwm_state.e_pow,
        },
    }
}

// TODO: finish this eventually
fn initial_simulation_frame2(db: &AscentDb) -> SimulationFrame {
    let (
        db_sim_frame,
        rotor_1_state,
        pwm_filter_1_state,
        rotor_2_state,
        pwm_filter_2_state,
        rotor_3_state,
        pwm_filter_3_state,
        rotor_4_state,
        pwm_filter_4_state,
        gyro_filter_1,
        gyro_filter_2,
        gyro_filter_3,
    ) = db.select_simulation_frame(1).unwrap();
    let rotor1 = db_to_rotor_state(rotor_1_state, pwm_filter_1_state);
    let rotor2 = db_to_rotor_state(rotor_2_state, pwm_filter_2_state);
    let rotor3 = db_to_rotor_state(rotor_3_state, pwm_filter_3_state);
    let rotor4 = db_to_rotor_state(rotor_4_state, pwm_filter_4_state);
    let battery_state = BatteryState {
        capacity: db_sim_frame.capacity,
        bat_voltage: db_sim_frame.bat_voltage,
        bat_voltage_sag: db_sim_frame.bat_voltage_sag,
        amperage: db_sim_frame.amperage,
        m_ah_drawn: db_sim_frame.m_ah_drawn,
    };
    let drone_state = DroneFrameState {
        position: Vector3::new(
            db_sim_frame.position_x,
            db_sim_frame.position_y,
            db_sim_frame.position_z,
        ),
        rotation: Rotation3::from(UnitQuaternion::new_normalize(Quaternion::new(
            db_sim_frame.rotation_w,
            db_sim_frame.rotation_x,
            db_sim_frame.rotation_y,
            db_sim_frame.rotation_z,
        ))),
        linear_velocity: Vector3::new(
            db_sim_frame.linear_velocity_x,
            db_sim_frame.linear_velocity_y,
            db_sim_frame.linear_velocity_z,
        ),
        angular_velocity: Vector3::new(
            db_sim_frame.angular_velocity_x,
            db_sim_frame.angular_velocity_y,
            db_sim_frame.angular_velocity_z,
        ),
        acceleration: Vector3::new(
            db_sim_frame.acceleration_x,
            db_sim_frame.acceleration_y,
            db_sim_frame.acceleration_z,
        ),
    };

    let gyro_state = GyroState {
        rotation: UnitQuaternion::new_normalize(Quaternion::new(
            db_sim_frame.gyro_rotation_w,
            db_sim_frame.gyro_rotation_x,
            db_sim_frame.gyro_rotation_y,
            db_sim_frame.gyro_rotation_z,
        )),
        acceleration: Vector3::new(
            db_sim_frame.gyro_acceleration_x,
            db_sim_frame.gyro_rotation_y,
            db_sim_frame.gyro_rotation_z,
        ),
        angular_velocity: Vector3::new(
            db_sim_frame.gyro_angular_velocity_x,
            db_sim_frame.gyro_angular_velocity_y,
            db_sim_frame.gyro_angular_velocity_z,
        ),
        low_pass_filters: [
            LowPassFilter {
                output: gyro_filter_1.output,
                e_pow: gyro_filter_1.e_pow,
            },
            LowPassFilter {
                output: gyro_filter_2.output,
                e_pow: gyro_filter_2.e_pow,
            },
            LowPassFilter {
                output: gyro_filter_3.output,
                e_pow: gyro_filter_3.e_pow,
            },
        ],
    };

    SimulationFrame {
        battery_state,
        drone_state,
        rotors_state: RotorsState([rotor1, rotor2, rotor3, rotor4]),
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

// TODO: make this come from the db
pub fn get_base_model2(db: &AscentDb) -> Drone {
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
    let initial_frame = initial_simulation_frame2(db);

    Drone {
        current_frame: initial_frame.clone(),
        next_frame: initial_frame.clone(),
        battery_model: battery_model.clone(),
        rotor_model,
        drone_model,
        gyro_model,
    }
}

#[cfg(test)]
mod test {
    use db::AscentDb;

    #[test]
    fn get_frame_test() {
        let db = AscentDb::new("/home/gabor/ascent/quad/data.sqlite");
        let sim_frame = db.select_simulation_frame(1);
        println!("{:?}", sim_frame);
    }
}
