// TODO: finish this eventually

use std::ops::DerefMut;

// use diesel::{
//     query_dsl::methods::{FilterDsl, OrderDsl},
//     ExpressionMethods, JoinOnDsl, Queryable, RunQueryDsl, Selectable,
// };
use diesel::prelude::*;

use crate::{schema::low_pass_filter, simulation_frame::DBLowPassFilter, AscentDb};

#[derive(Clone, Queryable, Selectable, Debug)]
#[diesel(table_name = crate::schema::sample_point)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
pub struct DBSamplePoint {
    pub id: i64,
    pub drone_model_id: i64,
    pub discharge: f64,
    pub voltage: f64,
}

#[derive(Clone, Queryable, Selectable, Debug)]
#[diesel(table_name = crate::schema::drone_model)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
pub struct DBDroneModel {
    pub id: i64,
    pub quad_bat_capacity: f64,
    pub quad_bat_cell_count: i64,
    pub quad_bat_capacity_charged: f64,
    pub max_voltage_sag: f64,
    pub prop_max_rpm: f64,
    pub motor_1_lpf: i64,
    pub motor_2_lpf: i64,
    pub motor_3_lpf: i64,
    pub motor_4_lpf: i64,
    pub motor_kv: f64,
    pub motor_r: f64,
    pub motor_io: f64,
    pub prop_thrust_factor1: f64,
    pub prop_thrust_factor2: f64,
    pub prop_thrust_factor3: f64,
    pub prop_torque_factor: f64,
    pub prop_a_factor: f64,
    pub prop_inertia: f64,
    pub frame_drag_area1: f64,
    pub frame_drag_area2: f64,
    pub frame_drag_area3: f64,
    pub frame_drag_constant: f64,
    pub mass: f64,
    pub inv_tensor_diag1: f64,
    pub inv_tensor_diag2: f64,
    pub inv_tensor_diag3: f64,
}

impl AscentDb {
    pub fn select_sample_points(&self, drone_id: i64) -> Vec<DBSamplePoint> {
        use crate::schema::sample_point::dsl::*;

        let mut conn = self.diesel_conn.lock().unwrap();
        sample_point
            .filter(drone_model_id.eq(drone_id))
            .order(discharge.asc())
            .load::<DBSamplePoint>(conn.deref_mut())
            .unwrap()
    }

    pub fn select_drone_model(
        &self,
        drone_id: i64,
    ) -> Option<(
        DBDroneModel,
        DBLowPassFilter,
        DBLowPassFilter,
        DBLowPassFilter,
        DBLowPassFilter,
    )> {
        let mut conn = self.diesel_conn.lock().unwrap();

        let (motor_lpf_1, motor_lpf_2, motor_lpf_3, motor_lpf_4) = diesel::alias!(
            crate::schema::low_pass_filter as rotor_lpf_1,
            crate::schema::low_pass_filter as rotor_lpf_2,
            crate::schema::low_pass_filter as rotor_lpf_3,
            crate::schema::low_pass_filter as rotor_lpf_4
        );

        crate::schema::drone_model::table
            .filter(crate::schema::drone_model::id.eq(drone_id))
            .inner_join(motor_lpf_1.on(
                crate::schema::drone_model::motor_1_lpf.eq(motor_lpf_1.field(low_pass_filter::id)),
            ))
            .inner_join(motor_lpf_2.on(
                crate::schema::drone_model::motor_2_lpf.eq(motor_lpf_2.field(low_pass_filter::id)),
            ))
            .inner_join(motor_lpf_3.on(
                crate::schema::drone_model::motor_3_lpf.eq(motor_lpf_3.field(low_pass_filter::id)),
            ))
            .inner_join(motor_lpf_4.on(
                crate::schema::drone_model::motor_4_lpf.eq(motor_lpf_4.field(low_pass_filter::id)),
            ))
            .load::<(
                DBDroneModel,
                DBLowPassFilter,
                DBLowPassFilter,
                DBLowPassFilter,
                DBLowPassFilter,
            )>(conn.deref_mut())
            .unwrap()
            .pop()
    }
}
