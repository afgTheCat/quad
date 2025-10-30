// TODO: finish this eventually

use crate::{schema::low_pass_filter, simulation_frame::DBLowPassFilter, AscentDb};
use diesel::prelude::*;
use std::ops::DerefMut;

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

mod ascent_db_2 {
    use crate::{
        model::{DBDroneModel, DBSamplePoint},
        simulation_frame::DBLowPassFilter,
        AscentDb2,
    };
    use sqlx::{query_as, Connection, Sqlite, Transaction};

    async fn fetch_lpf(trx: &mut Transaction<'_, Sqlite>, id: i64) -> DBLowPassFilter {
        let query = query_as!(
            DBLowPassFilter,
            r#"
                SELECT *
                FROM low_pass_filter
                WHERE low_pass_filter.id = ?
            "#,
            id
        );
        query.fetch_one(trx.as_mut()).await.unwrap()
    }

    impl AscentDb2 {
        async fn select_sample_points_async(&mut self, drone_id: i64) -> Vec<DBSamplePoint> {
            let query = query_as!(
                DBSamplePoint,
                r#"
                    SELECT id, drone_model_id, discharge, voltage 
                    FROM sample_point WHERE drone_model_id = ? order by discharge asc
                "#,
                drone_id
            );
            query.fetch_all(&mut self.conn).await.unwrap()
        }

        fn select_sample_points(&mut self, drone_id: i64) -> Vec<DBSamplePoint> {
            smol::block_on(async { self.select_sample_points_async(drone_id).await })
        }

        async fn select_drone_model_async(
            &mut self,
            drone_id: i64,
        ) -> (
            DBDroneModel,
            DBLowPassFilter,
            DBLowPassFilter,
            DBLowPassFilter,
            DBLowPassFilter,
        ) {
            let mut trx = self.conn.begin().await.unwrap();
            let query = query_as!(
                DBDroneModel,
                r#"
                    SELECT *
                    FROM drone_model 
                    WHERE drone_model.id = ?
                "#,
                drone_id
            );
            let drone_model = query.fetch_one(&mut *trx).await.unwrap();
            let lpf1 = fetch_lpf(&mut trx, drone_model.motor_1_lpf).await;
            let lpf2 = fetch_lpf(&mut trx, drone_model.motor_2_lpf).await;
            let lpf3 = fetch_lpf(&mut trx, drone_model.motor_3_lpf).await;
            let lpf4 = fetch_lpf(&mut trx, drone_model.motor_4_lpf).await;
            trx.commit().await.unwrap();
            (drone_model, lpf1, lpf2, lpf3, lpf4)
        }

        fn select_drone_model(
            &mut self,
            drone_id: i64,
        ) -> (
            DBDroneModel,
            DBLowPassFilter,
            DBLowPassFilter,
            DBLowPassFilter,
            DBLowPassFilter,
        ) {
            smol::block_on(async { self.select_drone_model_async(drone_id).await })
        }
    }
}
