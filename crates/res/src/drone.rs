use crate::{
    input::RcInput,
    representation::{LastStateRepr, OutputRepr, Representation, RepresentationType},
    reservoir::Esn,
};
use nalgebra::DMatrix;
use ridge::RidgeRegression;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone)]
pub struct DroneRc {
    pub esn: Esn,
    pub representation: Representation,
    // Could be changed to ElasticNetWrapper
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
        let representation = match representation {
            RepresentationType::LastState => Representation::LastState(LastStateRepr::default()),
            RepresentationType::Output(alpha) => Representation::Output(OutputRepr::new(alpha)),
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
}
