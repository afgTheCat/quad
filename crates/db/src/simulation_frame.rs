use crate::AscentDb;
use diesel::{
    prelude::{Insertable, Queryable},
    ExpressionMethods, JoinOnDsl, QueryDsl, RunQueryDsl, Selectable,
};
use std::ops::DerefMut;

#[derive(Clone, Queryable, Selectable, Insertable, Debug)]
#[diesel(table_name = crate::schema::low_pass_filter)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
pub struct DBLowPassFilter {
    pub id: i64,
    pub output: f64,
    pub e_pow: f64,
}

#[derive(Clone, Queryable, Selectable, Insertable, Debug)]
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

#[derive(Clone, Queryable, Selectable, Insertable, Debug)]
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

    pub gyro_rotation_x: f64,
    pub gyro_rotation_y: f64,
    pub gyro_rotation_z: f64,
    pub gyro_rotation_w: f64,

    pub gyro_acceleration_x: f64,
    pub gyro_acceleration_y: f64,
    pub gyro_acceleration_z: f64,

    pub gyro_angular_velocity_x: f64,
    pub gyro_angular_velocity_y: f64,
    pub gyro_angular_velocity_z: f64,

    pub gyro_low_pass_filter_1: i64,
    pub gyro_low_pass_filter_2: i64,
    pub gyro_low_pass_filter_3: i64,
}

impl AscentDb {
    // WOW, this is just ugly. We really wanna move towards a better db/config storage
    pub fn select_simulation_frame(
        &self,
        simulation_id: i64,
    ) -> Option<(
        DBSimulationFrame,
        DBRotorState,
        DBLowPassFilter,
        DBRotorState,
        DBLowPassFilter,
        DBRotorState,
        DBLowPassFilter,
        DBRotorState,
        DBLowPassFilter,
        DBLowPassFilter,
        DBLowPassFilter,
        DBLowPassFilter,
    )> {
        use crate::schema::low_pass_filter;
        use crate::schema::rotor_state;
        use crate::schema::simulation_frame;

        let mut conn = self.diesel_conn.lock().unwrap();

        let (rotor_state_1, rotor_state_2, rotor_state_3, rotor_state_4) = diesel::alias!(
            crate::schema::rotor_state as rotor_state_1,
            crate::schema::rotor_state as rotor_state_2,
            crate::schema::rotor_state as rotor_state_3,
            crate::schema::rotor_state as rotor_state_4,
        );

        let (
            rotor_lpf_1,
            rotor_lpf_2,
            rotor_lpf_3,
            rotor_lpf_4,
            gyro_lpf_1,
            gyro_lpf_2,
            gyro_lpf_3,
        ) = diesel::alias!(
            crate::schema::low_pass_filter as rotor_lpf_1,
            crate::schema::low_pass_filter as rotor_lpf_2,
            crate::schema::low_pass_filter as rotor_lpf_3,
            crate::schema::low_pass_filter as rotor_lpf_4,
            crate::schema::low_pass_filter as gyro_lpf_1,
            crate::schema::low_pass_filter as gyro_lpf_2,
            crate::schema::low_pass_filter as gyro_lpf_3,
        );

        simulation_frame::table
            .inner_join(
                rotor_state_1
                    .on(simulation_frame::rotor_1_state.eq(rotor_state_1.field(rotor_state::id))),
            )
            .inner_join(
                rotor_lpf_1.on(rotor_state_1
                    .field(rotor_state::pwm_low_pass_filter)
                    .eq(rotor_lpf_1.field(low_pass_filter::id))),
            )
            .inner_join(
                rotor_state_2
                    .on(simulation_frame::rotor_2_state.eq(rotor_state_2.field(rotor_state::id))),
            )
            .inner_join(
                rotor_lpf_2.on(rotor_state_2
                    .field(rotor_state::pwm_low_pass_filter)
                    .eq(rotor_lpf_2.field(low_pass_filter::id))),
            )
            .inner_join(
                rotor_state_3
                    .on(simulation_frame::rotor_3_state.eq(rotor_state_3.field(rotor_state::id))),
            )
            .inner_join(
                rotor_lpf_3.on(rotor_state_3
                    .field(rotor_state::pwm_low_pass_filter)
                    .eq(rotor_lpf_3.field(low_pass_filter::id))),
            )
            .inner_join(
                rotor_state_4
                    .on(simulation_frame::rotor_4_state.eq(rotor_state_4.field(rotor_state::id))),
            )
            .inner_join(
                rotor_lpf_4.on(rotor_state_4
                    .field(rotor_state::pwm_low_pass_filter)
                    .eq(rotor_lpf_4.field(low_pass_filter::id))),
            )
            .inner_join(gyro_lpf_1.on(
                simulation_frame::gyro_low_pass_filter_1.eq(gyro_lpf_1.field(low_pass_filter::id)),
            ))
            .inner_join(gyro_lpf_2.on(
                simulation_frame::gyro_low_pass_filter_2.eq(gyro_lpf_2.field(low_pass_filter::id)),
            ))
            .inner_join(gyro_lpf_3.on(
                simulation_frame::gyro_low_pass_filter_3.eq(gyro_lpf_3.field(low_pass_filter::id)),
            ))
            .load::<(
                DBSimulationFrame,
                DBRotorState,
                DBLowPassFilter,
                DBRotorState,
                DBLowPassFilter,
                DBRotorState,
                DBLowPassFilter,
                DBRotorState,
                DBLowPassFilter,
                DBLowPassFilter,
                DBLowPassFilter,
                DBLowPassFilter,
            )>(conn.deref_mut())
            .unwrap()
            // TODO: would be nice to filter as part of the dsl, but fuck it
            .into_iter()
            .find(|(sim_frame, ..)| sim_frame.id == simulation_id)
    }
}
