use crate::{Logger, SnapShot};
use db_common::DBNewFlightLog;
use db_common::queries::TestingDB;
use sqlx::Connection;
use sqlx::SqliteConnection;
use sqlx::query;
use std::sync::Arc;
use std::sync::Mutex;
use std::time::Duration;

pub struct DBLogger2 {
    pub data: Vec<DBNewFlightLog>,
    pub simulation_id: String,
    pub last_time_step: f64,
    pub db: Arc<Mutex<TestingDB>>,
}

impl DBLogger2 {
    pub fn new(db: Arc<Mutex<TestingDB>>, simulation_id: String) -> Self {
        Self {
            data: vec![],
            simulation_id,
            last_time_step: 0.,
            db,
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
            linear_acceleration_z: snapshot.gyro_update.linear_acc[2],
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

impl Drop for DBLogger2 {
    fn drop(&mut self) {
        let mut db = self.db.lock().unwrap();
        db.write_flight_logs(&self.simulation_id, &self.data);
    }
}

impl Logger for DBLogger2 {
    fn log_time_stamp(&mut self, duration: Duration, snapshot: SnapShot) {
        self.push_to_data(duration, snapshot);
        self.last_time_step = duration.as_secs_f64();
    }
}

pub struct DBLogger {
    pub data: Vec<DBNewFlightLog>,
    pub simulation_id: String,
    pub last_time_step: f64,
    pub conn: SqliteConnection,
}

impl DBLogger {
    pub fn new(simulation_id: String) -> Self {
        // TODO: get the locaction frfr
        let conn = smol::block_on(async {
            SqliteConnection::connect("sqlite://crates/db_common/schema.sqlite")
                .await
                .unwrap()
        });
        Self {
            data: vec![],
            simulation_id,
            last_time_step: 0.,
            conn,
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
            linear_acceleration_z: snapshot.gyro_update.linear_acc[2],
            angular_velocity_x: snapshot.gyro_update.angular_velocity[0],
            angular_velocity_y: snapshot.gyro_update.angular_velocity[1],
            angular_velocity_z: snapshot.gyro_update.angular_velocity[2],
            throttle: snapshot.channels.throttle,
            roll: snapshot.channels.roll,
            pitch: snapshot.channels.pitch,
            yaw: snapshot.channels.yaw,
        });
    }

    async fn write_flight_logs_async(&mut self) {
        let mut trx = self.conn.begin().await.unwrap();
        for flight_log in self.data.iter() {
            let query = query!(
                r#"
                    INSERT INTO flight_log (
                        simulation_id, start_seconds, end_seconds, motor_input_1, motor_input_2,
                        motor_input_3, motor_input_4, battery_voltage_sag, battery_voltage, amperage,
                        mah_drawn, cell_count, rot_quat_x, rot_quat_y, rot_quat_z, rot_quat_w,
                        linear_acceleration_x, linear_acceleration_y, linear_acceleration_z,
                        angular_velocity_x, angular_velocity_y, angular_velocity_z, throttle, roll, pitch, yaw
                    ) VALUES (
                        ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?
                    )"#,
                self.simulation_id,
                flight_log.start_seconds,
                flight_log.end_seconds,
                flight_log.motor_input_1,
                flight_log.motor_input_2,
                flight_log.motor_input_3,
                flight_log.motor_input_4,
                flight_log.battery_voltage_sag,
                flight_log.battery_voltage,
                flight_log.amperage,
                flight_log.mah_drawn,
                flight_log.cell_count,
                flight_log.rot_quat_x,
                flight_log.rot_quat_y,
                flight_log.rot_quat_z,
                flight_log.rot_quat_w,
                flight_log.linear_acceleration_x,
                flight_log.linear_acceleration_y,
                flight_log.linear_acceleration_z,
                flight_log.angular_velocity_x,
                flight_log.angular_velocity_y,
                flight_log.angular_velocity_z,
                flight_log.throttle,
                flight_log.roll,
                flight_log.pitch,
                flight_log.yaw,
            );
            query.execute(&mut *trx).await.unwrap();
        }
        trx.commit().await.unwrap();
    }

    fn write_flight_logs(&mut self) {
        smol::block_on(async { self.write_flight_logs_async().await })
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
        self.write_flight_logs();
    }
}
