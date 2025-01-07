use crate::schema::flight_log;
use crate::AscentDb2;
use derive_more::derive::Deref;
use diesel::ExpressionMethods;
use diesel::{
    prelude::{Insertable, Queryable},
    query_dsl::methods::{DistinctDsl, FilterDsl, OrderDsl, SelectDsl},
    RunQueryDsl, Selectable,
};
use flight_controller::{BatteryUpdate, Channels, GyroUpdate, MotorInput};
use nalgebra::DVector;
use rusqlite::params;
use std::ops::DerefMut;
use std::{ops::Range, time::Duration};

#[derive(Debug, Clone)]
pub struct FlightLogEvent {
    pub range: Range<Duration>,
    pub motor_input: MotorInput,
    pub battery_update: BatteryUpdate,
    pub gyro_update: GyroUpdate,
    pub channels: Channels,
}

impl FlightLogEvent {
    pub fn to_rc_input(&self) -> DVector<f64> {
        let FlightLogEvent {
            battery_update:
                BatteryUpdate {
                    bat_voltage_sag,
                    bat_voltage,
                    amperage,
                    m_ah_drawn,
                    ..
                },
            gyro_update:
                GyroUpdate {
                    rotation,
                    linear_acc,
                    angular_velocity,
                },
            channels:
                Channels {
                    throttle,
                    roll,
                    pitch,
                    yaw,
                },
            ..
        } = self;
        DVector::from_row_slice(&[
            *bat_voltage_sag,
            *bat_voltage,
            *amperage,
            *m_ah_drawn,
            rotation[0],
            rotation[1],
            rotation[2],
            rotation[3],
            linear_acc[0],
            linear_acc[1],
            linear_acc[2],
            angular_velocity[0],
            angular_velocity[1],
            angular_velocity[2],
            *throttle,
            *roll,
            *yaw,
            *pitch,
        ])
    }

    pub fn to_rc_output(&self) -> DVector<f64> {
        let FlightLogEvent {
            motor_input: MotorInput { input },
            ..
        } = self;
        DVector::from_row_slice(input)
    }
}

#[derive(Debug, Deref, Default, Clone)]
pub struct FlightLog(pub Vec<FlightLogEvent>);

impl FlightLog {
    pub fn len(&self) -> usize {
        self.0.len()
    }
}

#[derive(Queryable, Selectable, Insertable)]
#[diesel(table_name = crate::schema::flight_log)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
pub struct DBFlightLog {
    id: i64,
    simulation_id: String,
    start_seconds: f64,
    end_seconds: f64,
    motor_input_1: f64,
    motor_input_2: f64,
    motor_input_3: f64,
    motor_input_4: f64,
    battery_voltage_sag: f64,
    battery_voltage: f64,
    amperage: f64,
    mah_drawn: f64,
    cell_count: i64,
    rot_quat_x: f64,
    rot_quat_y: f64,
    rot_quat_z: f64,
    rot_quat_w: f64,
    linear_acceleration_x: f64,
    linear_acceleration_y: f64,
    linear_acceleration_z: f64,
    angular_velocity_x: f64,
    angular_velocity_y: f64,
    angular_velocity_z: f64,
    throttle: f64,
    roll: f64,
    pitch: f64,
    yaw: f64,
}

#[derive(Insertable, Default)]
#[diesel(table_name = crate::schema::flight_log)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
pub struct DBNewFlightLog {
    simulation_id: String,
    start_seconds: f64,
    end_seconds: f64,
    motor_input_1: f64,
    motor_input_2: f64,
    motor_input_3: f64,
    motor_input_4: f64,
    battery_voltage_sag: f64,
    battery_voltage: f64,
    amperage: f64,
    mah_drawn: f64,
    cell_count: i64,
    rot_quat_x: f64,
    rot_quat_y: f64,
    rot_quat_z: f64,
    rot_quat_w: f64,
    linear_acceleration_x: f64,
    linear_acceleration_y: f64,
    linear_acceleration_z: f64,
    angular_velocity_x: f64,
    angular_velocity_y: f64,
    angular_velocity_z: f64,
    throttle: f64,
    roll: f64,
    pitch: f64,
    yaw: f64,
}

const INSERT_FLIGHT_LOGS_QUERY: &str = "
    INSERT INTO flight_log (
        simulation_id, start_seconds, end_seconds, motor_input_1, motor_input_2,
        motor_input_3, motor_input_4, battery_voltage_sag, battery_voltage, amperage,
        mah_drawn, cell_count, rot_quat_x, rot_quat_y, rot_quat_z, rot_quat_w,
        linear_acceleration_x, linear_acceleration_y, linear_acceleration_z,
        angular_velocity_x, angular_velocity_y, angular_velocity_z, throttle, roll, pitch, yaw
    ) VALUES (
        ?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12, ?13, ?14, ?15, ?16, ?17, ?18, ?19, ?20, ?21, ?22, ?23, ?24, ?25, ?26
    )
";

impl AscentDb2 {
    // We use rusqlite here because diesel is just too slow. What makes this fast is that we are
    // preparing a repetetive statement and use it in a transaction. I don't see a faster way as
    // things stand now
    pub fn write_flight_logs(&self, simulation_id: &str, flight_logs: &[FlightLogEvent]) {
        let mut conn = self.rusqlite_conn.lock().unwrap();
        // should be fast enough, but in the future we can do batch insert
        let tx = conn.transaction().unwrap();
        {
            let mut statement = tx.prepare(INSERT_FLIGHT_LOGS_QUERY).unwrap();
            for flight_log in flight_logs {
                statement
                    .execute(params![
                        &simulation_id,
                        flight_log.range.start.as_secs_f64(),
                        flight_log.range.end.as_secs_f64(),
                        flight_log.motor_input[0],
                        flight_log.motor_input[1],
                        flight_log.motor_input[2],
                        flight_log.motor_input[3],
                        flight_log.battery_update.bat_voltage_sag,
                        flight_log.battery_update.bat_voltage,
                        flight_log.battery_update.amperage,
                        flight_log.battery_update.m_ah_drawn,
                        flight_log.battery_update.cell_count,
                        flight_log.gyro_update.rotation[0],
                        flight_log.gyro_update.rotation[1],
                        flight_log.gyro_update.rotation[2],
                        flight_log.gyro_update.rotation[3],
                        flight_log.gyro_update.linear_acc[0],
                        flight_log.gyro_update.linear_acc[1],
                        flight_log.gyro_update.linear_acc[2],
                        flight_log.gyro_update.angular_velocity[0],
                        flight_log.gyro_update.angular_velocity[1],
                        flight_log.gyro_update.angular_velocity[2],
                        flight_log.channels.throttle,
                        flight_log.channels.roll,
                        flight_log.channels.pitch,
                        flight_log.channels.yaw,
                    ])
                    .unwrap();
            }
        }
        tx.commit().unwrap();
    }

    pub fn get_all_simulation_ids(&self) -> Vec<String> {
        use self::flight_log::dsl::*;
        let mut conn = self.diesel_conn.lock().unwrap();

        flight_log
            .select(simulation_id)
            .distinct()
            .load::<String>(conn.deref_mut())
            .unwrap()
    }

    pub fn get_simulation_data(&self, sim_id: &str) -> FlightLog {
        use self::flight_log::dsl::*;
        let mut conn = self.diesel_conn.lock().unwrap();

        let events = flight_log
            .filter(simulation_id.eq(sim_id))
            .order(start_seconds.asc())
            .load::<DBFlightLog>(conn.deref_mut())
            .unwrap()
            .iter()
            .map(|x: &DBFlightLog| FlightLogEvent {
                range: Duration::from_secs_f64(x.start_seconds)
                    ..Duration::from_secs_f64(x.end_seconds),
                motor_input: MotorInput {
                    input: [
                        x.motor_input_1,
                        x.motor_input_2,
                        x.motor_input_3,
                        x.motor_input_4,
                    ],
                },
                battery_update: BatteryUpdate {
                    bat_voltage_sag: x.battery_voltage_sag,
                    bat_voltage: x.battery_voltage,
                    amperage: x.amperage,
                    m_ah_drawn: x.mah_drawn,
                    cell_count: x.cell_count as u8,
                },
                gyro_update: GyroUpdate {
                    rotation: [x.rot_quat_x, x.rot_quat_y, x.rot_quat_z, x.rot_quat_z],
                    linear_acc: [
                        x.linear_acceleration_x,
                        x.linear_acceleration_y,
                        x.linear_acceleration_z,
                    ],
                    angular_velocity: [
                        x.angular_velocity_x,
                        x.angular_velocity_y,
                        x.angular_velocity_z,
                    ],
                },
                channels: Channels {
                    throttle: x.throttle,
                    roll: x.roll,
                    pitch: x.pitch,
                    yaw: x.yaw,
                },
            })
            .collect();
        FlightLog(events)
    }
}
