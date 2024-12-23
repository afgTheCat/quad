use nalgebra::{DMatrix, DVector};

use super::{reservoir::Reservoir, ModelInput};
use crate::ridge::RidgeRegression;

// TODO: we should be able to reproduce this
pub struct RcModel {
    pub esn_model: Reservoir,
    pub embedding: RidgeRegression,
    pub readout: RidgeRegression,
}

impl RcModel {
    pub fn new(
        n_internal_units: usize,
        connectivity: f64,
        spectral_radius: f64,
        input_scaling: f64,
        embedding: RidgeRegression,
        readout: RidgeRegression,
    ) -> Self {
        let esn_model = Reservoir::new(
            n_internal_units,
            connectivity,
            spectral_radius,
            input_scaling,
        );
        Self {
            esn_model,
            embedding,
            readout,
        }
    }

    // TODO: kinda slow, we can improve it
    fn compute_state_matricies(&mut self, input: &ModelInput) -> Vec<DMatrix<f64>> {
        let n_internal_units = self.esn_model.n_internal_units;
        let mut states: Vec<DMatrix<f64>> =
            vec![DMatrix::zeros(input.time, n_internal_units); input.episodes];
        let mut previous_state: DMatrix<f64> = DMatrix::zeros(input.episodes, n_internal_units);

        for t in 0..input.time {
            let current_input = input.input_at_time(t);
            previous_state = self.esn_model.integrate(current_input, previous_state);
            for ep in 0..input.episodes {
                states[ep].set_row(t, &previous_state.row(ep));
            }
        }

        states
    }

    pub fn fit(&mut self, input: ModelInput, categories: DMatrix<f64>) {
        let mut coeff_tr = vec![];
        let mut biases_tr = vec![];
        let res_states = self.compute_state_matricies(&input);
        // Episode?
        for (x, res_state) in input.inputs.iter().zip(res_states) {
            let (coeff, intercept) = self.embedding.fit_multiple(
                res_state.rows(0, input.time - 1).into_owned(),
                x.rows(1, input.time - 1).into_owned(),
            );
            coeff_tr.push(DVector::from(coeff.as_slice().to_vec()).transpose());
            biases_tr.push(intercept.transpose());
        }
        let coeff_tr = DMatrix::from_rows(&coeff_tr);
        let biases_tr = DMatrix::from_rows(&biases_tr);

        // TODO: chatgpt solution, prob bad
        let input_repr = DMatrix::from_fn(
            coeff_tr.nrows(),
            coeff_tr.ncols() + biases_tr.ncols(),
            |r, c| {
                if c < coeff_tr.ncols() {
                    coeff_tr[(r, c)]
                } else {
                    biases_tr[(r, c - coeff_tr.ncols())]
                }
            },
        );

        self.readout.fit_multiple(input_repr, categories);
    }

    pub fn predict(&mut self, input: ModelInput) -> Vec<usize> {
        let mut coeff_tr = vec![];
        let mut biases_tr = vec![];
        let res_states = self.compute_state_matricies(&input);

        for (x, res_state) in input.inputs.iter().zip(res_states) {
            let (coeff, intercept) = self.embedding.fit_multiple(
                res_state.rows(0, input.time - 1).into_owned(),
                x.rows(1, input.time - 1).into_owned(),
            );
            coeff_tr.push(DVector::from(coeff.as_slice().to_vec()).transpose());
            biases_tr.push(intercept.transpose());
        }
        let coeff_tr = DMatrix::from_rows(&coeff_tr);
        let biases_tr = DMatrix::from_rows(&biases_tr);

        let input_repr = DMatrix::from_fn(
            coeff_tr.nrows(),
            coeff_tr.ncols() + biases_tr.ncols(),
            |r, c| {
                if c < coeff_tr.ncols() {
                    coeff_tr[(r, c)]
                } else {
                    biases_tr[(r, c - coeff_tr.ncols())]
                }
            },
        );

        let logits = self.readout.predict(input_repr);
        logits
            .row_iter()
            .map(|row| row.transpose().argmax().0)
            .collect::<Vec<_>>()
    }
}
