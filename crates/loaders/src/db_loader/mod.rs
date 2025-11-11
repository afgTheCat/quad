use crate::LoaderTrait;
use db_common::{
    DBDroneModel, DBLowPassFilter, DBRotorState, DBSamplePoint, DBSimulationFrame, NewDBRcModel,
    queries::TestingDB,
};
use drone::{
    BatteryModel, BatteryState, Drone, DroneFrameState, DroneModel, GyroModel, GyroState,
    LowPassFilter, RotorModel, RotorState, RotorsState, SampleCurve, SamplePoint, SimulationFrame,
};
use flight_controller::{Channels, controllers::res_controller::ResController};
use loggers::{FlightLog, SnapShot};
use nalgebra::{Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector3};
use res::drone::DroneRc;
use simulator::{BatteryUpdate, GyroUpdate, MotorInput};
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

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

#[derive(Debug, Default)]
pub struct DBLoader {
    db: Arc<Mutex<TestingDB>>,
}

pub struct DBParts {
    pub frame: DBSimulationFrame,
    pub rotor1_state: DBRotorState,
    pub pwm_filter_1: DBLowPassFilter,
    pub rotor2_state: DBRotorState,
    pub pwm_filter_2: DBLowPassFilter,
    pub rotor3_state: DBRotorState,
    pub pwm_filter_3: DBLowPassFilter,
    pub rotor4_state: DBRotorState,
    pub pwm_filter_4: DBLowPassFilter,
    pub gyro_filter1: DBLowPassFilter,
    pub gyro_filter2: DBLowPassFilter,
    pub gyro_filter3: DBLowPassFilter,
    pub drone_model: DBDroneModel,
    pub motor_lpf1: DBLowPassFilter,
    pub motor_lpf2: DBLowPassFilter,
    pub motor_lpf3: DBLowPassFilter,
    pub motor_lpf4: DBLowPassFilter,
    pub sample_points: Vec<DBSamplePoint>,
}

impl DBParts {
    // TODO: this is big and ugly, and everything in between. We may finish this, but not right
    // now!
    fn from_drone(simulation_id: String, drone: &Drone) -> Self {
        let rotor1_pwm_filter = DBLowPassFilter {
            id: todo!(),
            output: todo!(),
            e_pow: todo!(),
        };
        let rotor1_state = DBRotorState {
            id: 0,
            current: drone.current_frame.rotors_state[0].current,
            rpm: drone.current_frame.rotors_state[0].rpm,
            motor_torque: drone.current_frame.rotors_state[0].motor_torque,
            effective_thrust: drone.current_frame.rotors_state[0].effective_thrust,
            pwm: drone.current_frame.rotors_state[0].pwm,
            rotor_dir: drone.current_frame.rotors_state[0].rotor_dir,
            motor_pos_x: drone.current_frame.rotors_state[0].motor_pos.x,
            motor_pos_y: drone.current_frame.rotors_state[0].motor_pos.x,
            motor_pos_z: drone.current_frame.rotors_state[0].motor_pos.x,
            pwm_low_pass_filter: 0,
        };
        let db_frame = DBSimulationFrame {
            id: simulation_id,
            capacity: drone.current_frame.battery_state.capacity,
            bat_voltage: drone.current_frame.battery_state.bat_voltage,
            bat_voltage_sag: drone.current_frame.battery_state.bat_voltage_sag,
            amperage: drone.current_frame.battery_state.amperage,
            m_ah_drawn: drone.current_frame.battery_state.m_ah_drawn,
            rotor_1_state: todo!(),
            rotor_2_state: todo!(),
            rotor_3_state: todo!(),
            rotor_4_state: todo!(),
            position_x: todo!(),
            position_y: todo!(),
            position_z: todo!(),
            rotation_x: todo!(),
            rotation_y: todo!(),
            rotation_z: todo!(),
            rotation_w: todo!(),
            linear_velocity_x: todo!(),
            linear_velocity_y: todo!(),
            linear_velocity_z: todo!(),
            angular_velocity_x: todo!(),
            angular_velocity_y: todo!(),
            angular_velocity_z: todo!(),
            acceleration_x: todo!(),
            acceleration_y: todo!(),
            acceleration_z: todo!(),
            gyro_rotation_x: todo!(),
            gyro_rotation_y: todo!(),
            gyro_rotation_z: todo!(),
            gyro_rotation_w: todo!(),
            gyro_acceleration_x: todo!(),
            gyro_acceleration_y: todo!(),
            gyro_acceleration_z: todo!(),
            gyro_angular_velocity_x: todo!(),
            gyro_angular_velocity_y: todo!(),
            gyro_angular_velocity_z: todo!(),
            gyro_low_pass_filter_1: todo!(),
            gyro_low_pass_filter_2: todo!(),
            gyro_low_pass_filter_3: todo!(),
        };
        todo!()
    }
}

// pub fn drone_to_db_parts(drone: &Drone) {}

impl LoaderTrait for DBLoader {
    fn load_drone(&mut self, config_id: &str) -> Drone {
        let mut db = self.db.lock().unwrap();
        let frame = db.fetch_simulation_frame(config_id);
        let rotor1_state = db.fetch_rotor_state(frame.rotor_1_state);
        let pwm_filter_1_state = db.fetch_lpf(rotor1_state.pwm_low_pass_filter);
        let rotor2_state = db.fetch_rotor_state(frame.rotor_2_state);
        let pwm_filter_2_state = db.fetch_lpf(rotor2_state.pwm_low_pass_filter);
        let rotor3_state = db.fetch_rotor_state(frame.rotor_3_state);
        let pwm_filter_3_state = db.fetch_lpf(rotor3_state.pwm_low_pass_filter);
        let rotor4_state = db.fetch_rotor_state(frame.rotor_4_state);
        let pwm_fitler_4_state = db.fetch_lpf(rotor4_state.pwm_low_pass_filter);
        let gyro_filter_1 = db.fetch_lpf(frame.gyro_low_pass_filter_1);
        let gyro_filter_2 = db.fetch_lpf(frame.gyro_low_pass_filter_2);
        let gyro_filter_3 = db.fetch_lpf(frame.gyro_low_pass_filter_3);
        let rotor1 = db_to_rotor_state(rotor1_state, pwm_filter_1_state);
        let rotor2 = db_to_rotor_state(rotor2_state, pwm_filter_2_state);
        let rotor3 = db_to_rotor_state(rotor3_state, pwm_filter_3_state);
        let rotor4 = db_to_rotor_state(rotor4_state, pwm_fitler_4_state);
        let battery_state = BatteryState {
            capacity: frame.capacity,
            bat_voltage: frame.bat_voltage,
            bat_voltage_sag: frame.bat_voltage_sag,
            amperage: frame.amperage,
            m_ah_drawn: frame.m_ah_drawn,
        };
        let drone_state = DroneFrameState {
            position: Vector3::new(frame.position_x, frame.position_y, frame.position_z),
            rotation: Rotation3::from(UnitQuaternion::new_normalize(Quaternion::new(
                frame.rotation_w,
                frame.rotation_x,
                frame.rotation_y,
                frame.rotation_z,
            ))),
            linear_velocity: Vector3::new(
                frame.linear_velocity_x,
                frame.linear_velocity_y,
                frame.linear_velocity_z,
            ),
            angular_velocity: Vector3::new(
                frame.angular_velocity_x,
                frame.angular_velocity_y,
                frame.angular_velocity_z,
            ),
            acceleration: Vector3::new(
                frame.acceleration_x,
                frame.acceleration_y,
                frame.acceleration_z,
            ),
        };
        let gyro_state = GyroState {
            rotation: UnitQuaternion::new_normalize(Quaternion::new(
                frame.gyro_rotation_w,
                frame.gyro_rotation_x,
                frame.gyro_rotation_y,
                frame.gyro_rotation_z,
            )),
            acceleration: Vector3::new(
                frame.gyro_acceleration_x,
                frame.gyro_rotation_y,
                frame.gyro_rotation_z,
            ),
            angular_velocity: Vector3::new(
                frame.gyro_angular_velocity_x,
                frame.gyro_angular_velocity_y,
                frame.gyro_angular_velocity_z,
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
        let drone_model = db.fetch_drone_model(config_id);
        let motor_lpf1 = db.fetch_lpf(drone_model.motor_1_lpf);
        let motor_lpf2 = db.fetch_lpf(drone_model.motor_2_lpf);
        let motor_lpf3 = db.fetch_lpf(drone_model.motor_3_lpf);
        let motor_lpf4 = db.fetch_lpf(drone_model.motor_4_lpf);
        // TODO: change the config id
        let sample_points = db.fetch_sample_points(config_id);
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
                LowPassFilter::new(motor_lpf1.output, motor_lpf1.e_pow),
                LowPassFilter::new(motor_lpf2.output, motor_lpf2.e_pow),
                LowPassFilter::new(motor_lpf3.output, motor_lpf3.e_pow),
                LowPassFilter::new(motor_lpf4.output, motor_lpf4.e_pow),
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

    fn get_replay_ids(&mut self) -> Vec<String> {
        let mut db = self.db.lock().unwrap();
        db.fetch_replay_ids()
    }

    fn load_simulation(&mut self, config_id: &str) -> simulator::Simulator {
        let drone = self.load_drone(config_id);
        simulator::Simulator::default_from_drone(drone)
    }

    fn get_reservoir_controller_ids(&mut self) -> Vec<String> {
        let mut db = self.db.lock().unwrap();
        db.fetch_res_controller_ids()
    }

    fn load_replay(&mut self, sim_id: &str) -> FlightLog {
        let mut db = self.db.lock().unwrap();
        let db_flight_logs = db.fetch_flight_logs(sim_id);
        let snapshots = db_flight_logs
            .into_iter()
            .map(|fl| SnapShot {
                duration: Duration::from_secs_f64(fl.end_seconds - fl.start_seconds),
                motor_input: MotorInput {
                    input: [
                        fl.motor_input_1,
                        fl.motor_input_2,
                        fl.motor_input_3,
                        fl.motor_input_4,
                    ],
                },
                battery_update: BatteryUpdate {
                    bat_voltage_sag: fl.battery_voltage_sag,
                    bat_voltage: fl.battery_voltage,
                    amperage: fl.amperage,
                    m_ah_drawn: fl.mah_drawn,
                    cell_count: fl.cell_count as u64,
                },
                gyro_update: GyroUpdate {
                    rotation: [fl.rot_quat_w, fl.rot_quat_x, fl.rot_quat_y, fl.rot_quat_z],
                    angular_velocity: [
                        fl.angular_velocity_x,
                        fl.angular_velocity_y,
                        fl.angular_velocity_z,
                    ],
                    linear_acc: [
                        fl.linear_acceleration_x,
                        fl.linear_acceleration_y,
                        fl.linear_acceleration_z,
                    ],
                },
                channels: Channels {
                    throttle: fl.throttle,
                    yaw: fl.yaw,
                    roll: fl.roll,
                    pitch: fl.pitch,
                },
            })
            .collect();
        FlightLog {
            simulation_id: sim_id.to_owned(),
            steps: snapshots,
        }
    }

    fn insert_reservoir(&mut self, res: NewDBRcModel) {
        let mut db = self.db.lock().unwrap();
        db.insert_reservoir(res);
    }

    fn load_res_controller(&mut self, controller_id: &str) -> ResController {
        let mut db = self.db.lock().unwrap();
        let rc_data = db.load_db_rc_data(controller_id);
        let drone_rc = DroneRc::from_db(rc_data);
        ResController {
            model: Mutex::new(drone_rc),
        }
    }
}

#[cfg(test)]
mod test {
    use crate::db_loader::DBParts;
    use drone::default_drone::default_7in_4s_drone;

    #[test]
    fn save_default_config_to_db() {
        let default_drone = default_7in_4s_drone();
        let DBParts {
            frame,
            rotor1_state,
            pwm_filter_1,
            rotor2_state,
            pwm_filter_2,
            rotor3_state,
            pwm_filter_3,
            rotor4_state,
            pwm_filter_4,
            gyro_filter1,
            gyro_filter2,
            gyro_filter3,
            drone_model,
            motor_lpf1,
            motor_lpf2,
            motor_lpf3,
            motor_lpf4,
            sample_points,
        } = DBParts::from_drone(format!("7in_4s"), &default_drone);
    }
}
