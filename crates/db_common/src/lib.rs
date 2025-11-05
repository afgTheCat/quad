pub mod queries;

pub struct DBDroneModel {
    pub id: i64,
    pub quad_bat_capacity: f64,
    pub quad_bat_cell_count: i64,
    pub quad_bat_capacity_charged: f64,
    pub max_voltage_sag: f64,
    pub prop_max_rpm: f64,
    pub motor_1_lpf: i64,
    pub motor_2_lpf: i64,
    pub motor_3_lpf: i64,
    pub motor_4_lpf: i64,
    pub motor_kv: f64,
    pub motor_r: f64,
    pub motor_io: f64,
    pub prop_thrust_factor1: f64,
    pub prop_thrust_factor2: f64,
    pub prop_thrust_factor3: f64,
    pub prop_torque_factor: f64,
    pub prop_a_factor: f64,
    pub prop_inertia: f64,
    pub frame_drag_area1: f64,
    pub frame_drag_area2: f64,
    pub frame_drag_area3: f64,
    pub frame_drag_constant: f64,
    pub mass: f64,
    pub inv_tensor_diag1: f64,
    pub inv_tensor_diag2: f64,
    pub inv_tensor_diag3: f64,
}

pub struct DBSamplePoint {
    pub id: i64,
    pub drone_model_id: i64,
    pub discharge: f64,
    pub voltage: f64,
}

pub struct DBLowPassFilter {
    pub id: i64,
    pub output: f64,
    pub e_pow: f64,
}

pub struct DBRotorState {
    pub id: i64,
    pub current: f64,
    pub rpm: f64,
    pub motor_torque: f64,
    pub effective_thrust: f64,
    pub pwm: f64,
    pub rotor_dir: f64,
    pub motor_pos_x: f64,
    pub motor_pos_y: f64,
    pub motor_pos_z: f64,
    pub pwm_low_pass_filter: i64, // references low pass filter
}

pub struct DBSimulationFrame {
    pub id: i64,
    pub capacity: f64,
    pub bat_voltage: f64,
    pub bat_voltage_sag: f64,
    pub amperage: f64,
    pub m_ah_drawn: f64,
    pub rotor_1_state: i64, // references rotor state
    pub rotor_2_state: i64, // references rotor state
    pub rotor_3_state: i64, // references rotor state
    pub rotor_4_state: i64, // references rotor state

    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,

    pub rotation_x: f64,
    pub rotation_y: f64,
    pub rotation_z: f64,
    pub rotation_w: f64,

    pub linear_velocity_x: f64,
    pub linear_velocity_y: f64,
    pub linear_velocity_z: f64,

    pub angular_velocity_x: f64,
    pub angular_velocity_y: f64,
    pub angular_velocity_z: f64,

    pub acceleration_x: f64,
    pub acceleration_y: f64,
    pub acceleration_z: f64,

    pub gyro_rotation_x: f64,
    pub gyro_rotation_y: f64,
    pub gyro_rotation_z: f64,
    pub gyro_rotation_w: f64,

    pub gyro_acceleration_x: f64,
    pub gyro_acceleration_y: f64,
    pub gyro_acceleration_z: f64,

    pub gyro_angular_velocity_x: f64,
    pub gyro_angular_velocity_y: f64,
    pub gyro_angular_velocity_z: f64,

    pub gyro_low_pass_filter_1: i64,
    pub gyro_low_pass_filter_2: i64,
    pub gyro_low_pass_filter_3: i64,
}

pub struct DBRcModel {
    pub id: i64,
    pub rc_id: String,
    pub n_internal_units: i64,
    pub input_scaling: f64,
    pub internal_weights: String,
    pub input_weights: Option<String>,
    pub alpha: f64,
    pub readout_coeff: Option<String>,
    pub readout_intercept: Option<String>,
}

#[derive(Debug, Clone)]
pub struct DBFlightLog {
    pub id: i64,
    pub simulation_id: String,
    pub start_seconds: f64,
    pub end_seconds: f64, // TODO: do we need this? probbaly not
    pub motor_input_1: f64,
    pub motor_input_2: f64,
    pub motor_input_3: f64,
    pub motor_input_4: f64,
    pub battery_voltage_sag: f64,
    pub battery_voltage: f64,
    pub amperage: f64,
    pub mah_drawn: f64,
    pub cell_count: i64,
    pub rot_quat_x: f64,
    pub rot_quat_y: f64,
    pub rot_quat_z: f64,
    pub rot_quat_w: f64,
    pub linear_acceleration_x: f64,
    pub linear_acceleration_y: f64,
    pub linear_acceleration_z: f64,
    pub angular_velocity_x: f64,
    pub angular_velocity_y: f64,
    pub angular_velocity_z: f64,
    pub throttle: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

pub struct DBNewFlightLog {
    pub simulation_id: String,
    pub start_seconds: f64,
    pub end_seconds: f64, // TODO: do we need this? probbaly not
    pub motor_input_1: f64,
    pub motor_input_2: f64,
    pub motor_input_3: f64,
    pub motor_input_4: f64,
    pub battery_voltage_sag: f64,
    pub battery_voltage: f64,
    pub amperage: f64,
    pub mah_drawn: f64,
    pub cell_count: i64,
    pub rot_quat_x: f64,
    pub rot_quat_y: f64,
    pub rot_quat_z: f64,
    pub rot_quat_w: f64,
    pub linear_acceleration_x: f64,
    pub linear_acceleration_y: f64,
    pub linear_acceleration_z: f64,
    pub angular_velocity_x: f64,
    pub angular_velocity_y: f64,
    pub angular_velocity_z: f64,
    pub throttle: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

pub struct NewDBRcModel {
    pub rc_id: String,
    pub n_internal_units: i64,
    pub input_scaling: f64,
    pub internal_weights: String,
    pub input_weights: Option<String>,
    pub alpha: f64,
    pub readout_coeff: Option<String>,
    pub readout_intercept: Option<String>,
}
