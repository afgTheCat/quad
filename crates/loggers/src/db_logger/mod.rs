use db::{AscentDb, simulation::DBNewFlightLog};
use std::{sync::Arc, time::Duration};

use crate::{Logger, SnapShot};

pub struct DBLogger {
    pub data: Vec<DBNewFlightLog>,
    pub db: Arc<AscentDb>,
    pub simulation_id: String,
    pub last_time_step: f64,
}

impl DBLogger {
    pub fn new(db: Arc<AscentDb>, simulation_id: String) -> Self {
        Self {
            data: vec![],
            db,
            simulation_id,
            last_time_step: 0.,
        }
    }

    fn push_to_data(&mut self, duration: Duration, snapshot: SnapShot) {
        self.data.push(DBNewFlightLog {
            simulation_id: self.simulation_id.clone(),
            // TODO: this is stupid I think
            start_seconds: self.last_time_step,
            end_seconds: duration.as_secs_f64(),
            motor_input_1: snapshot.motor_input[0],
            motor_input_2: snapshot.motor_input[1],
            motor_input_3: snapshot.motor_input[2],
            motor_input_4: snapshot.motor_input[3],
            battery_voltage_sag: snapshot.battery_update.bat_voltage_sag,
            battery_voltage: snapshot.battery_update.bat_voltage,
            amperage: snapshot.battery_update.amperage,
            mah_drawn: snapshot.battery_update.m_ah_drawn,
            cell_count: snapshot.battery_update.cell_count as i64,
            rot_quat_x: snapshot.gyro_update.rotation[0],
            rot_quat_y: snapshot.gyro_update.rotation[1],
            rot_quat_z: snapshot.gyro_update.rotation[2],
            rot_quat_w: snapshot.gyro_update.rotation[3],
            linear_acceleration_x: snapshot.gyro_update.linear_acc[0],
            linear_acceleration_y: snapshot.gyro_update.linear_acc[1],
            linear_acceleration_z: snapshot.gyro_update.linear_acc[3],
            angular_velocity_x: snapshot.gyro_update.angular_velocity[0],
            angular_velocity_y: snapshot.gyro_update.angular_velocity[1],
            angular_velocity_z: snapshot.gyro_update.angular_velocity[2],
            throttle: snapshot.channels.throttle,
            roll: snapshot.channels.roll,
            pitch: snapshot.channels.pitch,
            yaw: snapshot.channels.yaw,
        });
    }
}

impl Logger for DBLogger {
    fn log_time_stamp(&mut self, duration: Duration, snapshot: SnapShot) {
        self.push_to_data(duration, snapshot);
        self.last_time_step = duration.as_secs_f64();
    }
}

impl Drop for DBLogger {
    fn drop(&mut self) {
        self.db.write_flight_logs(&self.simulation_id, &self.data);
    }
}
