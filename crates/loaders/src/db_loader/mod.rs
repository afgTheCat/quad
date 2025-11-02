mod schema;

use std::sync::Mutex;

use drone::{
    BatteryModel, BatteryState, Drone, DroneFrameState, DroneModel, GyroModel, GyroState,
    LowPassFilter, RotorModel, RotorState, RotorsState, SampleCurve, SamplePoint, SimulationFrame,
};
use flight_controller::controllers::res_controller::ResController;
use loggers::FlightLog;
use nalgebra::{Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector3};
use res::drone::DroneRc;
use sqlx::{Connection, Sqlite, Transaction, query_as};

use crate::{
    DataAccessLayer,
    db_loader::schema::{
        DBDroneModel, DBLowPassFilter, DBRcData, DBRotorState, DBSamplePoint, DBSimulationFrame,
    },
};

pub struct DBLoader {
    conn: sqlx::SqliteConnection,
}

impl DataAccessLayer for DBLoader {
    fn load_drone(&mut self, config_id: &str) -> Drone {
        smol::block_on(async { self.load_drone_async(1).await })
    }

    fn load_simulation(&mut self, config_id: &str) -> simulator::Simulator {
        let drone = self.load_drone(config_id);
        simulator::Simulator::default_from_drone(drone)
    }

    fn load_replay(&mut self, sim_id: &str) -> FlightLog {
        todo!()
    }

    fn get_replay_ids(&mut self) -> Vec<String> {
        smol::block_on(async { self.load_replay_ids_async().await })
    }

    fn get_reservoir_controller_ids(&mut self) -> Vec<String> {
        smol::block_on(async { self.load_res_controller_async().await })
    }

    fn load_res_controller(&mut self, controller_id: &str) -> ResController {
        let rc_data = smol::block_on(async { self.load_db_rc_data(controller_id).await });
        let drone_rc: DroneRc = {
            // DroneRc::from_db(rc_data)
            todo!()
        };
        ResController {
            model: Mutex::new(drone_rc),
        }
    }
}

async fn fetch_lpf(trx: &mut Transaction<'_, Sqlite>, id: i64) -> DBLowPassFilter {
    let query = query_as!(
        DBLowPassFilter,
        r#"SELECT * FROM low_pass_filter WHERE low_pass_filter.id = ?"#,
        id
    );
    query.fetch_one(trx.as_mut()).await.unwrap()
}

async fn fetch_rotor_state(trx: &mut Transaction<'_, Sqlite>, id: i64) -> DBRotorState {
    let query = query_as!(
        DBRotorState,
        r#"SELECT * FROM rotor_state WHERE rotor_state.id = ?"#,
        id
    );
    query.fetch_one(trx.as_mut()).await.unwrap()
}

async fn select_drone_model_async(
    trx: &mut Transaction<'_, Sqlite>,
    drone_id: i64,
) -> (
    DBDroneModel,
    DBLowPassFilter,
    DBLowPassFilter,
    DBLowPassFilter,
    DBLowPassFilter,
) {
    let query = query_as!(
        DBDroneModel,
        r#"
                    SELECT *
                    FROM drone_model 
                    WHERE drone_model.id = ?
            "#,
        drone_id
    );
    let drone_model = query.fetch_one(trx.as_mut()).await.unwrap();
    let lpf1 = fetch_lpf(trx, drone_model.motor_1_lpf).await;
    let lpf2 = fetch_lpf(trx, drone_model.motor_2_lpf).await;
    let lpf3 = fetch_lpf(trx, drone_model.motor_3_lpf).await;
    let lpf4 = fetch_lpf(trx, drone_model.motor_4_lpf).await;
    (drone_model, lpf1, lpf2, lpf3, lpf4)
}

async fn select_sample_points(
    trx: &mut Transaction<'_, Sqlite>,
    config_id: i64,
) -> Vec<DBSamplePoint> {
    let query = query_as!(
        DBSamplePoint,
        r#"
            SELECT id, drone_model_id, discharge, voltage 
            FROM sample_point WHERE drone_model_id = ? order by discharge asc
        "#,
        config_id
    );
    query.fetch_all(trx.as_mut()).await.unwrap()
}

pub fn db_to_rotor_state(db_rotor_state: DBRotorState, pwm_state: DBLowPassFilter) -> RotorState {
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

impl DBLoader {
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

    async fn load_res_controller_async(&mut self) -> Vec<String> {
        struct RcId {
            rc_id: String,
        }
        let query = query_as!(RcId, r#"SELECT rc_id from rc_model"#);
        let res = query.fetch_all(&mut self.conn).await.unwrap();
        res.iter().map(|x| x.rc_id.clone()).collect()
    }

    async fn load_db_rc_data(&mut self, model_id: &str) -> DBRcData {
        let query = query_as!(
            DBRcData,
            r#"SELECT * from rc_model where rc_id = ?"#,
            model_id
        );
        query.fetch_one(&mut self.conn).await.unwrap()
    }

    async fn load_drone_async(&mut self, config_id: i64) -> Drone {
        let mut trx = self.conn.begin().await.unwrap();
        let query = query_as!(
            DBSimulationFrame,
            r#"SELECT * from simulation_frame WHERE simulation_frame.id = ?"#,
            config_id
        );
        let simulation_frame = query.fetch_one(trx.as_mut()).await.unwrap();
        let rotor1_state = fetch_rotor_state(&mut trx, simulation_frame.rotor_1_state).await;
        let pwm_filter_1_state = fetch_lpf(&mut trx, rotor1_state.pwm_low_pass_filter).await;
        let rotor2_state = fetch_rotor_state(&mut trx, simulation_frame.rotor_2_state).await;
        let pwm_filter_2_state = fetch_lpf(&mut trx, rotor2_state.pwm_low_pass_filter).await;
        let rotor3_state = fetch_rotor_state(&mut trx, simulation_frame.rotor_3_state).await;
        let pwm_filter_3_state = fetch_lpf(&mut trx, rotor3_state.pwm_low_pass_filter).await;
        let rotor4_state = fetch_rotor_state(&mut trx, simulation_frame.rotor_4_state).await;
        let pwm_fitler_4_state = fetch_lpf(&mut trx, rotor4_state.pwm_low_pass_filter).await;
        let gyro_filter_1 = fetch_lpf(&mut trx, simulation_frame.gyro_low_pass_filter_1).await;
        let gyro_filter_2 = fetch_lpf(&mut trx, simulation_frame.gyro_low_pass_filter_2).await;
        let gyro_filter_3 = fetch_lpf(&mut trx, simulation_frame.gyro_low_pass_filter_3).await;
        let rotor1 = db_to_rotor_state(rotor1_state, pwm_filter_1_state);
        let rotor2 = db_to_rotor_state(rotor2_state, pwm_filter_2_state);
        let rotor3 = db_to_rotor_state(rotor3_state, pwm_filter_3_state);
        let rotor4 = db_to_rotor_state(rotor4_state, pwm_fitler_4_state);
        let battery_state = BatteryState {
            capacity: simulation_frame.capacity,
            bat_voltage: simulation_frame.bat_voltage,
            bat_voltage_sag: simulation_frame.bat_voltage_sag,
            amperage: simulation_frame.amperage,
            m_ah_drawn: simulation_frame.m_ah_drawn,
        };
        let drone_state = DroneFrameState {
            position: Vector3::new(
                simulation_frame.position_x,
                simulation_frame.position_y,
                simulation_frame.position_z,
            ),
            rotation: Rotation3::from(UnitQuaternion::new_normalize(Quaternion::new(
                simulation_frame.rotation_w,
                simulation_frame.rotation_x,
                simulation_frame.rotation_y,
                simulation_frame.rotation_z,
            ))),
            linear_velocity: Vector3::new(
                simulation_frame.linear_velocity_x,
                simulation_frame.linear_velocity_y,
                simulation_frame.linear_velocity_z,
            ),
            angular_velocity: Vector3::new(
                simulation_frame.angular_velocity_x,
                simulation_frame.angular_velocity_y,
                simulation_frame.angular_velocity_z,
            ),
            acceleration: Vector3::new(
                simulation_frame.acceleration_x,
                simulation_frame.acceleration_y,
                simulation_frame.acceleration_z,
            ),
        };
        let gyro_state = GyroState {
            rotation: UnitQuaternion::new_normalize(Quaternion::new(
                simulation_frame.gyro_rotation_w,
                simulation_frame.gyro_rotation_x,
                simulation_frame.gyro_rotation_y,
                simulation_frame.gyro_rotation_z,
            )),
            acceleration: Vector3::new(
                simulation_frame.gyro_acceleration_x,
                simulation_frame.gyro_rotation_y,
                simulation_frame.gyro_rotation_z,
            ),
            angular_velocity: Vector3::new(
                simulation_frame.gyro_angular_velocity_x,
                simulation_frame.gyro_angular_velocity_y,
                simulation_frame.gyro_angular_velocity_z,
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
        let current_frame = SimulationFrame {
            battery_state,
            drone_frame_state: drone_state,
            rotors_state: RotorsState([rotor1, rotor2, rotor3, rotor4]),
            gyro_state,
        };

        let next_frame = current_frame.clone();
        let (
            drone_model,
            motor_low_pass_filter_1,
            motor_low_pass_filter_2,
            motor_low_pass_filter_3,
            motor_low_pass_filter_4,
        ) = select_drone_model_async(&mut trx, config_id).await;
        let sample_points = select_sample_points(&mut trx, config_id).await;

        let bat_voltage_curve = SampleCurve::new(
            sample_points
                .iter()
                .map(|sp| SamplePoint::new(sp.discharge, sp.voltage))
                .collect(),
        );

        let battery_model = BatteryModel {
            quad_bat_capacity: drone_model.quad_bat_capacity,
            bat_voltage_curve,
            quad_bat_cell_count: drone_model.quad_bat_cell_count as u64,
            quad_bat_capacity_charged: drone_model.quad_bat_capacity_charged,
            max_voltage_sag: drone_model.max_voltage_sag,
        };

        let rotor_model = RotorModel {
            prop_max_rpm: drone_model.prop_max_rpm,
            pwm_low_pass_filter: [
                LowPassFilter::new(
                    motor_low_pass_filter_1.output,
                    motor_low_pass_filter_1.e_pow,
                ),
                LowPassFilter::new(
                    motor_low_pass_filter_2.output,
                    motor_low_pass_filter_2.e_pow,
                ),
                LowPassFilter::new(
                    motor_low_pass_filter_3.output,
                    motor_low_pass_filter_3.e_pow,
                ),
                LowPassFilter::new(
                    motor_low_pass_filter_4.output,
                    motor_low_pass_filter_4.e_pow,
                ),
            ],
            motor_kv: drone_model.motor_kv, // kv
            motor_r: drone_model.motor_r,   // resistence
            motor_io: drone_model.motor_io, // idle current
            prop_thrust_factor: Vector3::new(
                drone_model.prop_thrust_factor1,
                drone_model.prop_thrust_factor2,
                drone_model.prop_thrust_factor3,
            ),
            prop_torque_factor: drone_model.prop_torque_factor,
            prop_a_factor: drone_model.prop_a_factor,
            prop_inertia: drone_model.prop_inertia,
        };

        let drone_model = DroneModel {
            frame_drag_area: Vector3::new(
                drone_model.frame_drag_area1,
                drone_model.frame_drag_area2,
                drone_model.frame_drag_area3,
            ),
            frame_drag_constant: drone_model.frame_drag_constant,
            mass: drone_model.mass,
            inv_tensor: Matrix3::from_diagonal(&Vector3::new(
                drone_model.inv_tensor_diag1,
                drone_model.inv_tensor_diag2,
                drone_model.inv_tensor_diag3,
            )),
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
}
