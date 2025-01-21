// This trait is only here as an API proposal. We also have an implementation. NOTE:  HIGHTLY IN PROGRESS

use crate::{
    loggers::{EmptyLogger, Logger},
    low_pass_filter::LowPassFilter,
    BatteryModel, BatteryState, Drone, DroneFrameState, DroneModel, GyroModel, GyroState,
    RotorModel, RotorState, RotorsState, SampleCurve, SamplePoint, SimulationFrame, Simulator,
};
use db::{
    simulation_frame::{DBLowPassFilter, DBRotorState},
    AscentDb,
};
use flight_controller::{controllers::bf_controller2::BFController, FlightController};
use nalgebra::{Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector3};
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

// This is the prmary API that we wish to use. Loading a drone is straight forward, however using
// the appropriate logger is a question to be awnsered
pub trait SimulationLoader: Send + Sync {
    fn load_simulation(&self, drone_id: i64) -> Simulator;
    fn load_drone(&self, drone_id: i64) -> Drone;
}

pub struct SimLoader {
    db: Arc<AscentDb>,
}

impl SimLoader {
    pub fn new(db: Arc<AscentDb>) -> Self {
        Self { db }
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

impl SimulationLoader for SimLoader {
    fn load_drone(&self, drone_id: i64) -> Drone {
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
        ) = self.db.select_simulation_frame(drone_id).unwrap();
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

        let current_frame = SimulationFrame {
            battery_state,
            drone_state,
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
        ) = self.db.select_drone_model(drone_id).unwrap();
        let sample_points = self.db.select_sample_points(drone_id);

        let bat_voltage_curve = SampleCurve::new(
            sample_points
                .iter()
                .map(|sp| SamplePoint::new(sp.discharge, sp.voltage))
                .collect(),
        );

        let battery_model = BatteryModel {
            quad_bat_capacity: drone_model.quad_bat_capacity,
            bat_voltage_curve,
            quad_bat_cell_count: drone_model.quad_bat_cell_count as u8,
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

    // This should be a configuration id. Suppose I want to load in some custom logger. How should
    // that work?
    fn load_simulation(&self, drone_id: i64) -> Simulator {
        let drone = self.load_drone(drone_id);
        let flight_controller: Arc<dyn FlightController> = Arc::new(BFController::new());
        let logger: Arc<Mutex<dyn Logger>> = Arc::new(Mutex::new(EmptyLogger::new()));
        let time = Duration::default();
        let dt = Duration::from_nanos(5000);
        let fc_time_accu = Duration::default();
        let time_accu = Duration::default();
        Simulator {
            drone,
            flight_controller,
            logger,
            time,
            dt,
            fc_time_accu,
            time_accu,
        }
    }
}
