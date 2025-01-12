use crate::schema::flight_log;
use crate::AscentDb;
use diesel::ExpressionMethods;
use diesel::{
    prelude::{Insertable, Queryable},
    query_dsl::methods::{DistinctDsl, FilterDsl, OrderDsl, SelectDsl},
    RunQueryDsl, Selectable,
};
// use flight_controller::{BatteryUpdate, Channels, GyroUpdate, MotorInput};
// use nalgebra::DVector;
use rusqlite::params;
use std::ops::DerefMut;

#[derive(Queryable, Selectable, Insertable, Clone)]
#[diesel(table_name = crate::schema::flight_log)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
pub struct DBFlightLog {
    pub id: i64,
    pub simulation_id: String,
    pub start_seconds: f64,
    pub end_seconds: f64,
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

#[derive(Insertable, Default)]
#[diesel(table_name = crate::schema::flight_log)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
pub struct DBNewFlightLog {
    pub simulation_id: String,
    pub start_seconds: f64,
    pub end_seconds: f64,
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

impl AscentDb {
    // We use rusqlite here because diesel is just too slow. What makes this fast is that we are
    // preparing a repetetive statement and use it in a transaction. I don't see a faster way as
    // things stand now
    pub fn write_flight_logs(&self, simulation_id: &str, flight_logs: &[DBNewFlightLog]) {
        let mut conn = self.rusqlite_conn.lock().unwrap();
        // should be fast enough, but in the future we can do batch insert
        let tx = conn.transaction().unwrap();
        {
            let mut statement = tx.prepare(INSERT_FLIGHT_LOGS_QUERY).unwrap();
            for flight_log in flight_logs {
                statement
                    .execute(params![
                        &simulation_id,
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
                    ])
                    .unwrap();
            }
        }
        tx.commit().unwrap();
    }

    pub fn select_simulation_ids(&self) -> Vec<String> {
        use self::flight_log::dsl::*;
        let mut conn = self.diesel_conn.lock().unwrap();

        flight_log
            .select(simulation_id)
            .distinct()
            .load::<String>(conn.deref_mut())
            .unwrap()
    }

    pub fn get_simulation_data(&self, sim_id: &str) -> Vec<DBFlightLog> {
        use self::flight_log::dsl::*;
        let mut conn = self.diesel_conn.lock().unwrap();

        flight_log
            .filter(simulation_id.eq(sim_id))
            .order(start_seconds.asc())
            .load::<DBFlightLog>(conn.deref_mut())
            .unwrap()
    }
}
