use crate::{
    esn::Reservoir,
    input::RcInput,
    representation::{LastStateRepr, OutputRepr, Repr, RepresentationType},
    ridge::{RidgeRegression, RidgeRegressionSol},
};
use base64::{prelude::BASE64_STANDARD, Engine};
use db::{AscentDb2, DBRcData, NewDBRcData};
use flight_controller::{
    bindings::sitl_generated::dshotBitbangMode_e_DSHOT_BITBANG_AUTO, FlightControllerUpdate,
};
use nalgebra::DMatrix;

// TODO: serialize this
#[derive(Debug)]
pub struct DroneRc {
    pub esn: Reservoir,
    // representation: Box<dyn Repr>,
    pub readout: RidgeRegression,
}

// All the data that belong to a flight trajectory. [T, V]
struct FlightControllerInputs(DMatrix<f64>);

impl FlightControllerInputs {
    fn from_flight_controller_updates(update: &[FlightControllerUpdate]) -> Self {
        let rows = update
            .iter()
            .map(|up| up.to_rc_input().transpose())
            .collect::<Vec<_>>();
        FlightControllerInputs(DMatrix::from_rows(&rows))
    }
}

struct DroneInputSteps {
    episodes: usize,
    time: usize,
    vars: usize,
    updates: Vec<FlightControllerInputs>,
}

impl DroneInputSteps {
    fn input_at_time(&self, t: usize) -> DMatrix<f64> {
        let mut input_at_t: DMatrix<f64> = DMatrix::zeros(self.episodes, self.vars);
        for (i, ep) in self.updates.iter().enumerate() {
            input_at_t.set_row(i, &ep.0.row(t));
        }
        input_at_t
    }
}

impl DroneRc {
    pub fn new(
        n_internal_units: usize,
        connectivity: f64,
        spectral_radius: f64,
        input_scaling: f64,
        representation: RepresentationType,
        readout: RidgeRegression,
    ) -> Self {
        let esn = Reservoir::new(
            n_internal_units,
            connectivity,
            spectral_radius,
            input_scaling,
        );
        let representation: Box<dyn Repr> = match representation {
            RepresentationType::LastState => Box::new(LastStateRepr::new()),
            RepresentationType::Output(alpha) => Box::new(OutputRepr::new(alpha)),
        };

        Self {
            esn,
            // representation,
            readout,
        }
    }

    // this is kinda different, but should be ok
    pub fn fit(&mut self, input: Box<dyn RcInput>, data_points: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        self.readout
            .fit_multiple_svd(res_states[0].clone(), &data_points);
    }

    pub fn predict(&mut self, input: Box<dyn RcInput>) -> DMatrix<f64> {
        let res_states = self.esn.compute_state_matricies(&input);
        self.readout.predict(res_states[0].clone())
    }

    pub fn from_db(db_data: DBRcData) -> Self {
        let internal_weights_decoded = BASE64_STANDARD.decode(db_data.internal_weights).unwrap();
        let internal_weights: DMatrix<f64> =
            bincode::deserialize(&internal_weights_decoded).unwrap();
        let input_weights = if let Some(input_weights) = db_data.input_weights {
            let input_weights = BASE64_STANDARD.decode(input_weights).unwrap();
            Some(bincode::deserialize(&input_weights).unwrap())
        } else {
            None
        };
        let esn = Reservoir {
            n_internal_units: db_data.n_internal_units as usize,
            input_scaling: db_data.input_scaling,
            internal_weights,
            input_weights,
        };
        let sol = match (db_data.readout_coeff, db_data.readout_intercept) {
            (Some(coeff), Some(intercept)) => {
                let coeff_decoded = BASE64_STANDARD.decode(coeff).unwrap();
                let intercept_decoded = BASE64_STANDARD.decode(intercept).unwrap();
                Some(RidgeRegressionSol {
                    coeff: bincode::deserialize(&coeff_decoded).unwrap(),
                    intercept: bincode::deserialize(&intercept_decoded).unwrap(),
                })
            }
            _ => None,
        };
        let readout = RidgeRegression {
            alpha: db_data.alpha,
            sol,
        };
        DroneRc { esn, readout }
    }

    pub fn read_from_db(reservoir_id: &str, db: &AscentDb2) -> Option<Self> {
        let db_data = db.select_reservoir(reservoir_id)?;
        Some(Self::from_db(db_data))
    }

    pub fn to_new_db(&self, reservoir_id: String) -> NewDBRcData {
        let internal_weights_serialized =
            BASE64_STANDARD.encode(bincode::serialize(&self.esn.internal_weights).unwrap());
        let input_weights_serialized = if let Some(input_weights) = &self.esn.input_weights {
            Some(BASE64_STANDARD.encode(bincode::serialize(input_weights).unwrap()))
        } else {
            None
        };
        let (coeff, intercept) = if let Some(sol) = &self.readout.sol {
            (
                Some(BASE64_STANDARD.encode(bincode::serialize(&sol.coeff).unwrap())),
                Some(BASE64_STANDARD.encode(bincode::serialize(&sol.intercept).unwrap())),
            )
        } else {
            (None, None)
        };
        NewDBRcData {
            rc_id: reservoir_id.into(),
            input_scaling: self.esn.input_scaling,
            n_internal_units: self.esn.n_internal_units as i64,
            internal_weights: internal_weights_serialized,
            input_weights: input_weights_serialized,
            alpha: self.readout.alpha,
            readout_intercept: intercept,
            readout_coeff: coeff,
        }
    }

    pub fn save_model_to_db(&self, reservoir_id: String, db: &AscentDb2) {
        let new_db_rc_data = self.to_new_db(reservoir_id);
        db.insert_reservoir(new_db_rc_data);
    }
}

#[cfg(test)]
mod test {

    use super::DroneRc;
    use crate::{input::FlightInput, representation::RepresentationType, ridge::RidgeRegression};
    use db::{AscentDb2, FlightLogEvent};
    use flight_controller::MotorInput;
    use nalgebra::DMatrix;

    #[test]
    fn train_thing() {
        let db = AscentDb2::new("/home/gabor/ascent/quad/data.sqlite");
        let flight_log = db.get_simulation_data(&"86a9dd7f-f730-40cb-8fe8-e5a076867545");
        let mut drone_rc = DroneRc::new(
            500,
            0.3,
            0.99,
            0.2,
            RepresentationType::Output(1.),
            RidgeRegression::new(1.),
        );
        drone_rc.esn.set_input_weights(18);
        println!("ehhh {:?}", drone_rc.esn.input_weights.as_ref().unwrap());
        let input = FlightInput::new(vec![flight_log.clone()]);
        let data_points = DMatrix::from_columns(
            &flight_log
                .iter()
                .map(FlightLogEvent::to_rc_output)
                .collect::<Vec<_>>(),
        )
        .transpose();

        drone_rc.fit(Box::new(input.clone()), data_points);
        drone_rc.save_model_to_db("only_up".into(), &db);

        let mut new_rc_mode = DroneRc::read_from_db("only_up", &db).unwrap();
        let predicted_points = new_rc_mode.predict(Box::new(input));
        let mut rec_flight_logs = vec![];
        for (i, out) in predicted_points.row_iter().enumerate() {
            let motor_input = MotorInput {
                input: [
                    *out.get(0).unwrap(),
                    *out.get(1).unwrap(),
                    *out.get(2).unwrap(),
                    *out.get(3).unwrap(),
                ],
            };
            let fl = FlightLogEvent {
                range: flight_log[i].range.clone(),
                motor_input,
                battery_update: flight_log[i].battery_update,
                gyro_update: flight_log[i].gyro_update,
                channels: flight_log[i].channels,
            };
            rec_flight_logs.push(fl);
        }

        db.write_flight_logs("rec", &rec_flight_logs);
    }

    #[test]
    fn train_on_many() {
        let db = AscentDb2::new("/home/gabor/ascent/quad/data.sqlite");
        let simulation_id = db.get_all_simulation_ids();
        let tr_ids = simulation_id
            .iter()
            .filter(|id| id.starts_with("ds_id_1_tr"))
            .cloned()
            .collect::<Vec<_>>();
        let te_ids = simulation_id
            .iter()
            .filter(|id| id.starts_with("ds_id_1_te"))
            .cloned()
            .collect::<Vec<_>>();

        let mut drone_rc = DroneRc::new(
            500,
            0.3,
            0.99,
            0.2,
            RepresentationType::Output(1.),
            RidgeRegression::new(1.),
        );
        drone_rc.esn.set_input_weights(18);
        for tr_id in tr_ids {
            let flight_log = db.get_simulation_data(&tr_id);
            let input = FlightInput::new(vec![flight_log.clone()]);
            let data_points = DMatrix::from_columns(
                &flight_log
                    .iter()
                    .map(FlightLogEvent::to_rc_output)
                    .collect::<Vec<_>>(),
            )
            .transpose();
            drone_rc.fit(Box::new(input.clone()), data_points);
        }
    }
}
