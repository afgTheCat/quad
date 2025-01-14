use crate::AscentDb;
use diesel::{
    prelude::{Insertable, Queryable},
    Selectable,
};

// select * from simulation_frame left join rotor_state on rotor_1_state = rotor_state.id

#[derive(Clone, Queryable, Selectable, Insertable)]
#[diesel(table_name = crate::schema::low_pass_filter)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
pub struct DBLowPassFilter {
    pub id: i64,
    pub output: f64,
    pub e_pow: f64,
}

#[derive(Clone, Queryable, Selectable, Insertable)]
#[diesel(table_name = crate::schema::rotor_state)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
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

#[derive(Clone, Queryable, Selectable, Insertable)]
#[diesel(table_name = crate::schema::simulation_frame)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
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
}

impl AscentDb {
    pub fn select_simulation_frame(&self) {}
}
