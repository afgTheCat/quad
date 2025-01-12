// TODO: clean this up

use db::{simulation::DBNewFlightLog, AscentDb};
// use db::FlightLogEvent;
use flight_controller::{BatteryUpdate, Channels, GyroUpdate, MotorInput};
use std::time::Duration;

pub struct SimLogger {
    current_time_step: Duration,
    current_input: MotorInput,
    current_battery_update: BatteryUpdate,
    current_gyro: GyroUpdate,
    current_channels: Channels,
    pub simulation_id: Option<String>,
    pub data: Vec<DBNewFlightLog>, // start_seconds, end_seconds, motor_input_1, motor_input_2, motor_input_3, motor_input_4
}

impl SimLogger {
    pub fn new(
        current_input: MotorInput,
        current_battery_update: BatteryUpdate,
        current_gyro: GyroUpdate,
        current_channels: Channels,
    ) -> Self {
        let current_time_step = Duration::new(0, 0);
        Self {
            current_time_step,
            current_input,
            current_battery_update,
            current_gyro,
            current_channels,
            simulation_id: None,
            data: vec![],
        }
    }

    pub fn init(&mut self, simulation_id: String) {
        self.simulation_id = Some(simulation_id)
    }

    pub fn insert_data(
        &mut self,
        next_time_step: Duration,
        next_input: MotorInput,
        next_battery_input: BatteryUpdate,
        next_gyro_udpate: GyroUpdate,
        next_channels: Channels,
    ) {
        self.data.push(DBNewFlightLog {
            simulation_id: self.simulation_id.clone().unwrap(),
            start_seconds: self.current_time_step.as_secs_f64(),
            end_seconds: self.current_time_step.as_secs_f64(),
            motor_input_1: self.current_input.input[0],
            motor_input_2: self.current_input.input[1],
            motor_input_3: self.current_input.input[2],
            motor_input_4: self.current_input.input[3],
            battery_voltage_sag: self.current_battery_update.bat_voltage_sag,
            battery_voltage: self.current_battery_update.bat_voltage,
            amperage: self.current_battery_update.amperage,
            mah_drawn: self.current_battery_update.m_ah_drawn,
            cell_count: self.current_battery_update.cell_count as i64,
            rot_quat_x: self.current_gyro.rotation[0],
            rot_quat_y: self.current_gyro.rotation[1],
            rot_quat_z: self.current_gyro.rotation[2],
            rot_quat_w: self.current_gyro.rotation[3],
            linear_acceleration_x: self.current_gyro.linear_acc[0],
            linear_acceleration_y: self.current_gyro.linear_acc[1],
            linear_acceleration_z: self.current_gyro.linear_acc[2],
            angular_velocity_x: self.current_gyro.angular_velocity[0],
            angular_velocity_y: self.current_gyro.angular_velocity[1],
            angular_velocity_z: self.current_gyro.angular_velocity[2],
            throttle: self.current_channels.throttle,
            roll: self.current_channels.roll,
            pitch: self.current_channels.pitch,
            yaw: self.current_channels.yaw,
        });

        self.current_time_step = next_time_step;
        self.current_input = next_input;
        self.current_battery_update = next_battery_input;
        self.current_gyro = next_gyro_udpate;
        self.current_channels = next_channels;
    }

    pub fn deinit(&mut self) {
        self.simulation_id = None
    }

    pub fn write_logs(&self, db: &AscentDb) {
        db.write_flight_logs(&self.simulation_id.as_ref().unwrap(), &self.data);
    }
}
