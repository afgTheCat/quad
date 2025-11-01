mod schema;

use sqlx::{Connection, Sqlite, Transaction, query_as};

use crate::{
    DataAccessLayer,
    db_loader::schema::{DBDroneModel, DBLowPassFilter, DBSamplePoint},
};

pub struct DBLoader {
    conn: sqlx::SqliteConnection,
}

impl DataAccessLayer for DBLoader {
    fn load_drone(&self, config_id: &str) -> drone::Drone {
        todo!()
    }

    fn load_simulation(&self, config_id: &str) -> simulator::Simulator {
        todo!()
    }

    fn load_replay(&self, sim_id: &str) -> loggers::FlightLog {
        todo!()
    }

    fn get_replay_ids(&self) -> Vec<String> {
        todo!()
    }

    fn get_reservoir_controller_ids(&self) -> Vec<String> {
        todo!()
    }

    fn load_res_controller(
        &self,
        controller_id: &str,
    ) -> flight_controller::controllers::res_controller::ResController {
        todo!()
    }
}

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

// TODO: remove this
struct SimulationId {
    simulation_id: String, // TODO: this should be string
}

impl DBLoader {
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

    async fn select_simulation_ids_async(&mut self) -> Vec<String> {
        let query = query_as!(
            SimulationId,
            r#"SELECT DISTINCT simulation_id from flight_log"#
        );
        let res = query.fetch_all(&mut self.conn).await.unwrap();
        res.iter().map(|x| x.simulation_id.clone()).collect()
    }
}
