use super::{
    representation::{LastStateRepr, Repr},
    reservoir::Reservoir,
    ModelInput,
};
use crate::ridge::RidgeRegression;
use nalgebra::{DMatrix, DVector};

// TODO: we should be able to reproduce this
pub struct RcModel {
    pub esn: Reservoir,
    representation: Box<dyn Repr>,
    pub readout: RidgeRegression,
}

// Recresation of 'Reservoir computing approaches for representation and classification of multivariate time series'
impl RcModel {
    pub fn new(
        n_internal_units: usize,
        connectivity: f64,
        spectral_radius: f64,
        input_scaling: f64,
        embedding: f64,
        readout: RidgeRegression,
    ) -> Self {
        let esn = Reservoir::new(
            n_internal_units,
            connectivity,
            spectral_radius,
            input_scaling,
        );
        // let representation = OutputRepr::new(embedding);
        let representation = LastStateRepr::new();
        Self {
            esn,
            representation: Box::new(representation),
            readout,
        }
    }

    pub fn fit(&mut self, input: ModelInput, categories: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.fit_multiple(input_repr, categories);
    }

    pub fn predict(&mut self, input: ModelInput) -> Vec<usize> {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        let logits = self.readout.predict(input_repr);
        logits
            .row_iter()
            .map(|row| row.transpose().argmax().0)
            .collect::<Vec<_>>()
    }
}
