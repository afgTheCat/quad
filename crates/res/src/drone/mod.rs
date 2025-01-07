use crate::{
    esn::Reservoir,
    input::RcInput,
    representation::{LastStateRepr, OutputRepr, Repr, RepresentationType},
    ridge::{RidgeRegression, RidgeRegressionSol},
};
use base64::{prelude::BASE64_STANDARD, Engine};
use db::{AscentDb2, DBRcData, NewDBRcData};
use nalgebra::DMatrix;

// TODO: serialize this
#[derive(Debug)]
pub struct DroneRc {
    pub esn: Reservoir,
    // representation: Box<dyn Repr>,
    pub readout: RidgeRegression,
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
