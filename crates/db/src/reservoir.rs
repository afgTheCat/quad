use crate::{schema::rc_model, AscentDb};
use diesel::{
    prelude::{Insertable, Queryable},
    query_dsl::methods::{FilterDsl, SelectDsl},
    ExpressionMethods, RunQueryDsl, Selectable,
};
use std::ops::DerefMut;

#[derive(Clone, Queryable, Selectable, Insertable)]
#[diesel(table_name = crate::schema::rc_model)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
pub struct DBRcData {
    pub id: i64,
    pub rc_id: String,
    pub n_internal_units: i64,
    pub input_scaling: f64,
    pub internal_weights: String,
    pub input_weights: Option<String>,
    pub alpha: f64,
    pub readout_coeff: Option<String>,
    pub readout_intercept: Option<String>,
}

#[derive(Insertable)]
#[diesel(table_name = crate::schema::rc_model)]
#[diesel(check_for_backend(diesel::sqlite::Sqlite))]
pub struct NewDBRcData {
    pub rc_id: String,
    pub n_internal_units: i64,
    pub input_scaling: f64,
    pub internal_weights: String,
    pub input_weights: Option<String>,
    pub alpha: f64,
    pub readout_coeff: Option<String>,
    pub readout_intercept: Option<String>,
}

impl AscentDb {
    pub fn insert_reservoir(&self, res: NewDBRcData) {
        let mut conn = self.diesel_conn.lock().unwrap();
        diesel::insert_into(rc_model::table)
            .values(&res)
            .execute(conn.deref_mut())
            .unwrap();
    }

    pub fn select_reservoir(&self, model_id: &str) -> Option<DBRcData> {
        use self::rc_model::dsl::*;

        let mut conn = self.diesel_conn.lock().unwrap();
        rc_model
            .filter(rc_id.eq(model_id))
            .load::<DBRcData>(conn.deref_mut())
            .unwrap()
            .pop()
    }

    pub fn select_reservoir_ids(&self) -> Vec<String> {
        use self::rc_model::dsl::*;

        let mut conn = self.diesel_conn.lock().unwrap();
        rc_model
            .select(rc_id)
            .load::<String>(conn.deref_mut())
            .unwrap()
    }
}

mod ascent_db_2 {
    use sqlx::query;

    use crate::{AscentDb2, DBRcData, NewDBRcData};

    impl AscentDb2 {
        async fn insert_reservoir(&mut self, res: NewDBRcData) {
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

        async fn select_reservoir(&mut self, model_id: &str) -> Option<DBRcData> {
            todo!()
        }

        async fn select_reservoir_ids(&mut self) -> Vec<String> {
            todo!()
        }
    }
}
