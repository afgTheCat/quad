use crate::{
    input::RcInput,
    representation::{LastStateRepr, OutputRepr, Repr, RepresentationType},
    reservoir::Esn,
};
use base64::{prelude::BASE64_STANDARD, Engine};
use db_common::{DBRcModel, NewDBRcModel};
use nalgebra::{DMatrix, QR};
use ridge::{RidgeRegression, RidgeRegressionSol};

// TODO: serialize this
pub struct DroneRc {
    pub esn: Esn,
    representation: Box<dyn Repr>,
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
        let esn = Esn::new(
            n_internal_units,
            connectivity,
            spectral_radius,
            input_scaling,
        );
        let representation: Box<dyn Repr> = match representation {
            RepresentationType::LastState => Box::new(LastStateRepr::default()),
            RepresentationType::Output(alpha) => Box::new(OutputRepr::new(alpha)),
        };
        Self {
            esn,
            representation,
            readout,
        }
    }

    // this is the same, maybe it works maybe it does not
    pub fn fit(&mut self, input: Box<dyn RcInput>, rc_data: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.fit_multiple_svd(input_repr, &rc_data);
    }

    // a blast from the past! we are basically fitting on the res states. In fact we are fitting on the
    // first res state. Let's see what it can do! Will this error? Will this not? I think I did
    // this when I did not understand what I am supposed to do!
    pub fn old_fit(&mut self, input: Box<dyn RcInput>, data_points: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        self.readout
            .fit_multiple_svd(res_states[0].clone(), &data_points);
    }

    pub fn predict(&mut self, input: Box<dyn RcInput>) -> DMatrix<f64> {
        let res_states = self.esn.compute_state_matricies(&input);
        self.readout.predict(res_states[0].clone())
    }

    pub fn from_db(db_data: DBRcModel) -> Self {
        let internal_weights_decoded = BASE64_STANDARD.decode(db_data.internal_weights).unwrap();
        let internal_weights: DMatrix<f64> =
            bincode::deserialize(&internal_weights_decoded).unwrap();
        let input_weights = if let Some(input_weights) = db_data.input_weights {
            let input_weights = BASE64_STANDARD.decode(input_weights).unwrap();
            Some(bincode::deserialize(&input_weights).unwrap())
        } else {
            None
        };
        let esn = Esn {
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
        DroneRc {
            esn,
            representation: Box::new(OutputRepr::new(1.)),
            readout,
        }
    }

    pub fn to_new_db(&self, reservoir_id: String) -> NewDBRcModel {
        let internal_weights_serialized =
            BASE64_STANDARD.encode(bincode::serialize(&self.esn.internal_weights).unwrap());
        let input_weights_serialized = self.esn.input_weights.as_ref().map(|input_weights| {
            BASE64_STANDARD.encode(bincode::serialize(input_weights).unwrap())
        });
        let (coeff, intercept) = if let Some(sol) = &self.readout.sol {
            (
                Some(BASE64_STANDARD.encode(bincode::serialize(&sol.coeff).unwrap())),
                Some(BASE64_STANDARD.encode(bincode::serialize(&sol.intercept).unwrap())),
            )
        } else {
            (None, None)
        };
        NewDBRcModel {
            rc_id: reservoir_id,
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
