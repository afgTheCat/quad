use sqlx::{Connection, SqliteConnection, query, query_as};

use crate::{
    DBDroneModel, DBFlightLog, DBLowPassFilter, DBNewFlightLog, DBRcModel, DBRotorState,
    DBSamplePoint, DBSimulationFrame, NewDBRcModel,
};

#[derive(Debug)]
pub struct TestingDB {
    conn: SqliteConnection,
}

impl Default for TestingDB {
    fn default() -> Self {
        let conn = smol::block_on(async {
            SqliteConnection::connect("sqlite://crates/db_common/schema.sqlite")
                .await
                .unwrap()
        });
        Self { conn }
    }
}

impl TestingDB {
    async fn fetch_lpf_async(&mut self, id: i64) -> DBLowPassFilter {
        let query = query_as!(
            DBLowPassFilter,
            r#"SELECT * FROM low_pass_filter WHERE low_pass_filter.id = ?"#,
            id
        );
        query.fetch_one(&mut self.conn).await.unwrap()
    }

    pub fn fetch_lpf(&mut self, id: i64) -> DBLowPassFilter {
        smol::block_on(async { self.fetch_lpf_async(id).await })
    }

    async fn fetch_db_rc_data(&mut self, model_id: &str) -> DBRcModel {
        let query = query_as!(
            DBRcModel,
            r#"SELECT * from rc_model where rc_id = ?"#,
            model_id
        );
        query.fetch_one(&mut self.conn).await.unwrap()
    }

    pub fn fetch_rc_data(&mut self, model_id: &str) -> DBRcModel {
        smol::block_on(async { self.fetch_db_rc_data(model_id).await })
    }

    async fn fetch_sample_points_async(&mut self, config_id: i64) -> Vec<DBSamplePoint> {
        let query = query_as!(
            DBSamplePoint,
            r#"
                SELECT id, drone_model_id, discharge, voltage 
                FROM sample_point WHERE drone_model_id = ? order by discharge asc
            "#,
            config_id
        );
        query.fetch_all(&mut self.conn).await.unwrap()
    }

    pub fn fetch_sample_points(&mut self, config_id: i64) -> Vec<DBSamplePoint> {
        smol::block_on(async { self.fetch_sample_points_async(config_id).await })
    }

    async fn fetch_drone_model_async(&mut self, drone_id: i64) -> DBDroneModel {
        let query = query_as!(
            DBDroneModel,
            r#"
                SELECT *
                FROM drone_model 
                WHERE drone_model.id = ?
            "#,
            drone_id
        );
        query.fetch_one(&mut self.conn).await.unwrap()
    }

    pub fn fetch_drone_model(&mut self, drone_id: i64) -> DBDroneModel {
        smol::block_on(async { self.fetch_drone_model_async(drone_id).await })
    }

    async fn fetch_simulation_frame_async(&mut self, config_id: i64) -> DBSimulationFrame {
        let query = query_as!(
            DBSimulationFrame,
            r#"SELECT * from simulation_frame WHERE simulation_frame.id = ?"#,
            config_id
        );
        query.fetch_one(&mut self.conn).await.unwrap()
    }

    pub fn fetch_simulation_frame(&mut self, config_id: i64) -> DBSimulationFrame {
        smol::block_on(async { self.fetch_simulation_frame_async(config_id).await })
    }

    async fn fetch_rotor_state_async(&mut self, id: i64) -> DBRotorState {
        let query = query_as!(
            DBRotorState,
            r#"SELECT * FROM rotor_state WHERE rotor_state.id = ?"#,
            id
        );
        query.fetch_one(&mut self.conn).await.unwrap()
    }

    pub fn fetch_rotor_state(&mut self, id: i64) -> DBRotorState {
        smol::block_on(async { self.fetch_rotor_state_async(id).await })
    }

    async fn load_replay_ids_async(&mut self) -> Vec<String> {
        struct SimulationId {
            simulation_id: String, // TODO: this should be string
        }
        let query = query_as!(
            SimulationId,
            r#"SELECT DISTINCT simulation_id from flight_log"#
        );
        let res = query.fetch_all(&mut self.conn).await.unwrap();
        res.iter().map(|x| x.simulation_id.clone()).collect()
    }

    pub fn fetch_replay_ids(&mut self) -> Vec<String> {
        smol::block_on(async { self.load_replay_ids_async().await })
    }

    async fn fetch_res_controller_ids_async(&mut self) -> Vec<String> {
        struct RcId {
            rc_id: String,
        }
        let query = query_as!(RcId, r#"SELECT rc_id from rc_model"#);
        let res = query.fetch_all(&mut self.conn).await.unwrap();
        res.iter().map(|x| x.rc_id.clone()).collect()
    }

    pub fn fetch_res_controller_ids(&mut self) -> Vec<String> {
        smol::block_on(async { self.fetch_res_controller_ids_async().await })
    }

    async fn fetch_flight_logs_async(&mut self, sim_id: &str) -> Vec<DBFlightLog> {
        let query = query_as!(
            DBFlightLog,
            r#"SELECT * from flight_log WHERE simulation_id = ? ORDER BY start_seconds asc"#,
            sim_id
        );
        query.fetch_all(&mut self.conn).await.unwrap()
    }

    pub fn fetch_flight_logs(&mut self, sim_id: &str) -> Vec<DBFlightLog> {
        smol::block_on(async { self.fetch_flight_logs_async(sim_id).await })
    }

    async fn insert_reservoir_async(&mut self, res: NewDBRcModel) {
        let q = query!(
            r#"
                    INSERT INTO rc_model (rc_id, n_internal_units, input_scaling, internal_weights, input_weights, alpha, readout_coeff, readout_intercept)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            "#,
            res.rc_id,
            res.n_internal_units,
            res.input_scaling,
            res.internal_weights,
            res.input_weights,
            res.alpha,
            res.readout_coeff,
            res.readout_intercept
        );
        q.execute(&mut self.conn).await.unwrap();
    }

    pub fn insert_reservoir(&mut self, res: NewDBRcModel) {
        smol::block_on(async { self.insert_reservoir_async(res).await })
    }

    async fn load_db_rc_data_async(&mut self, model_id: &str) -> DBRcModel {
        let query = query_as!(
            DBRcModel,
            r#"SELECT * from rc_model where rc_id = ?"#,
            model_id
        );
        query.fetch_one(&mut self.conn).await.unwrap()
    }

    pub fn load_db_rc_data(&mut self, model_id: &str) -> DBRcModel {
        smol::block_on(async { self.load_db_rc_data_async(model_id).await })
    }

    async fn write_flight_logs_async(&mut self, simulation_id: &str, data: &[DBNewFlightLog]) {
        let mut trx = self.conn.begin().await.unwrap();
        for flight_log in data.iter() {
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
                simulation_id,
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

    pub fn write_flight_logs(&mut self, simulation_id: &str, data: &[DBNewFlightLog]) {
        smol::block_on(async { self.write_flight_logs_async(simulation_id, data).await })
    }
}
