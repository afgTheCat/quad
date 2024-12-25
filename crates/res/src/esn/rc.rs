use super::{
    representation::{self, LastStateRepr, OutputRepr, Repr},
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

    fn compute_state_matricies(&mut self, input: &ModelInput) -> Vec<DMatrix<f64>> {
        let n_internal_units = self.esn.n_internal_units;
        let mut states: Vec<DMatrix<f64>> =
            vec![DMatrix::zeros(input.time, n_internal_units); input.episodes];
        let mut previous_state: DMatrix<f64> = DMatrix::zeros(input.episodes, n_internal_units);

        for t in 0..input.time {
            let current_input = input.input_at_time(t);
            previous_state = self.esn.integrate(current_input, previous_state);
            for ep in 0..input.episodes {
                states[ep].set_row(t, &previous_state.row(ep));
            }
        }

        states
    }

    pub fn fit(&mut self, input: ModelInput, categories: DMatrix<f64>) {
        let res_states = self.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.fit_multiple(input_repr, categories);
    }

    pub fn predict(&mut self, input: ModelInput) -> Vec<usize> {
        let res_states = self.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        let logits = self.readout.predict(input_repr);
        logits
            .row_iter()
            .map(|row| row.transpose().argmax().0)
            .collect::<Vec<_>>()
    }
}
