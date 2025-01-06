use core::f64;

use crate::{
    esn::Reservoir,
    input::RcInput,
    representation::{LastStateRepr, OutputRepr, Repr, RepresentationType},
    ridge::{RidgeRegression, RidgeRegressionSol},
};
use base64::{prelude::BASE64_STANDARD, Engine};
use db::{DBRcData, NewDBRcData};
use flight_controller::FlightControllerUpdate;
use nalgebra::DMatrix;

// TODO: serialize this
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
        let input_weights: Option<DMatrix<f64>> = if let Some(input_weights) = db_data.input_weights
        {
            let input_weights_decoded = BASE64_STANDARD.decode(input_weights).unwrap();
            bincode::deserialize(&input_weights_decoded).unwrap()
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
}

#[cfg(test)]
mod test {
    use super::DroneRc;
    use crate::{input::FlightInput, representation::RepresentationType, ridge::RidgeRegression};
    use db::{AscentDb2, FlightLogEvent};
    use nalgebra::DMatrix;

    #[test]
    fn train_thing() {
        let db = AscentDb2::new("/home/gabor/ascent/quad/data.sqlite");
        let flight_log = db.get_simulation_data(&"7076b699-65d7-40b1-9ecb-0e58d664faf3");
        let mut drone_rc = DroneRc::new(
            500,
            0.3,
            0.99,
            0.2,
            RepresentationType::Output(1.),
            RidgeRegression::new(1.),
        );
        drone_rc.esn.set_input_weights(18);
        let input = FlightInput::new(vec![flight_log.clone()]);
        let data_points = DMatrix::from_columns(
            &flight_log
                .iter()
                .map(FlightLogEvent::to_rc_output)
                .collect::<Vec<_>>(),
        )
        .transpose();

        drone_rc.fit(Box::new(input.clone()), data_points);
        let predicted_points = drone_rc.predict(Box::new(input));
        for col in predicted_points.row_iter().skip(10000).take(1000) {
            // let asd = col.get(0);
            println!("predicted motor input: {}", col);
        }

        // TODO: lets not concern ourselfs with this
        // let new_flight_log = flight_log
        //     .iter()
        //     .zip(predicted_points.row_iter())
        //     .map(|(fl, pp)| FlightLogEvent {
        //         range: fl.range.clone(),
        //         motor_input: MotorInput {
        //             input: [
        //                 f64::clamp(*pp.get(0).unwrap(), 0., 1.),
        //                 f64::clamp(*pp.get(1).unwrap(), 0., 1.),
        //                 f64::clamp(*pp.get(2).unwrap(), 0., 1.),
        //                 f64::clamp(*pp.get(3).unwrap(), 0., 1.),
        //             ],
        //         },
        //         battery_update: fl.battery_update.clone(),
        //         gyro_update: fl.gyro_update.clone(),
        //         channels: fl.channels.clone(),
        //     })
        //     .collect::<Vec<_>>();
        // db.write_flight_logs("fake_simulation", &new_flight_log);
    }

    #[test]
    fn train_on_many() {
        let db = AscentDb2::new("/home/gabor/ascent/quad/data.sqlite");

        let mut drone_rc = DroneRc::new(
            500,
            0.3,
            0.99,
            0.2,
            RepresentationType::Output(1.),
            RidgeRegression::new(1.),
        );
    }
}
