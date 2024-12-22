use core::f64;

use nalgebra::{DMatrix, DVector};
use pyo3::prelude::*;
use rand::{
    distributions::{Bernoulli, Uniform},
    prelude::Distribution,
    thread_rng,
};

use crate::ridge::RidgeRegression;

// TODO: const generics
#[pyclass]
pub struct ClassicESNModel {
    internal_weights: DMatrix<f64>,
    input_weights: Option<DMatrix<f64>>,
    bias_vector: DVector<f64>,
}

impl ClassicESNModel {
    // This is pretty much the reservoir
    pub fn integrate(&self, input: DVector<f64>, state: DVector<f64>) -> DVector<f64> {
        let Some(input_weights) = self.input_weights.as_ref() else {
            panic!()
        };
        let state_before_tanh = &self.internal_weights * state.transpose()
            + input_weights * input.transpose()
            + &self.bias_vector;
        state_before_tanh.map(|e| e.tanh())
    }

    fn internal_weights(
        n_internal_units: usize,
        connectivity: f64,
        spectral_radius: f64,
    ) -> DMatrix<f64> {
        assert!(
            connectivity > 0.0 && connectivity <= 1.0,
            "Connectivity must be in (0, 1]."
        );

        // Generate a random sparse matrix with connectivity
        let mut rng = thread_rng();
        let uniform_dist = Uniform::new(-0.5, 0.5);
        let bernoulli = Bernoulli::new(connectivity).unwrap();
        let mut internal_weights = DMatrix::from_fn(n_internal_units, n_internal_units, |_, _| {
            if bernoulli.sample(&mut rng) {
                uniform_dist.sample(&mut rng)
            } else {
                0.0
            }
        });

        // Compute eigenvalues to find the spectral radius
        let eigenvalues = internal_weights.clone().symmetric_eigen().eigenvalues;
        let max_eigenvalue = eigenvalues
            .iter()
            .cloned()
            .map(f64::abs)
            .fold(f64::NEG_INFINITY, f64::max);

        // Scale matrix to match the desired spectral radius
        internal_weights /= max_eigenvalue / spectral_radius;

        internal_weights
    }

    fn input_weights(
        n_internal_units: usize,
        variables: usize,
        input_scaling: f64,
    ) -> DMatrix<f64> {
        let mut rng = thread_rng();
        let bernoulli = Bernoulli::new(0.5).unwrap();
        DMatrix::from_fn(n_internal_units, variables, |_, _| {
            if bernoulli.sample(&mut rng) {
                1.0 * input_scaling
            } else {
                -1. * input_scaling
            }
        })
    }
}

#[pymethods]
impl ClassicESNModel {
    #[new]
    fn new(n_internal_units: usize) -> PyResult<Self> {
        let internal_weights = Self::internal_weights(n_internal_units, 0.3, 0.99);
        let bias_vector = DVector::<f64>::zeros(n_internal_units);
        Ok(Self {
            internal_weights,
            bias_vector,
            input_weights: None,
        })
    }

    fn next(&self, input: Vec<f64>, state: Vec<f64>) -> PyResult<Vec<f64>> {
        let input = DVector::from_vec(input);
        let state = DVector::from_vec(state);
        let new_state = self.integrate(input, state);
        Ok(new_state.data.as_vec().clone())
    }
}

struct ModelInput {
    episodes: usize,
    time: usize,
    vars: usize,
    inputs: Vec<DMatrix<f64>>,
}

impl ModelInput {
    // return dim: [EPISIDES, VARS]
    fn input_at_time(&self, t: usize) -> DMatrix<f64> {
        let mut input_at_t: DMatrix<f64> = DMatrix::zeros(self.episodes, self.vars);
        for (i, ep) in self.inputs.iter().enumerate() {
            input_at_t.set_row(i, &ep.row(t));
        }
        input_at_t
    }
}

// TODO: we should be able to reproduce this
struct RcModel {
    n_internal_units: usize,
    connectivity: f64,
    spectral_radius: f64,
    input_scaling: f64,
    embedding: RidgeRegression,
    readout: RidgeRegression,
}

impl RcModel {
    // TODO: kinda slow, we can improve it
    fn compute_state_matricies(&self, input: &ModelInput) -> Vec<DMatrix<f64>> {
        let mut states: Vec<DMatrix<f64>> =
            vec![DMatrix::zeros(input.time, self.n_internal_units); input.episodes];
        let mut previous_state: DMatrix<f64> =
            DMatrix::zeros(input.episodes, self.n_internal_units);
        let internal_weights = ClassicESNModel::internal_weights(
            self.n_internal_units,
            self.connectivity,
            self.spectral_radius,
        );
        let input_weights =
            ClassicESNModel::input_weights(self.n_internal_units, input.vars, self.input_scaling);

        for t in 0..input.time {
            let current_input = input.input_at_time(t);
            let state_before_tanh = &internal_weights * previous_state.transpose()
                + &input_weights * current_input.transpose();
            previous_state = state_before_tanh.map(|e| e.tanh()).transpose();
            for ep in 0..input.episodes {
                states[ep].set_row(t, &state_before_tanh.transpose().row(ep));
            }
        }

        states
    }

    fn fit(&mut self, input: ModelInput, categories: DMatrix<f64>) {
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

    fn predict(&mut self, input: ModelInput) -> Vec<usize> {
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

#[pymodule]
fn res(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // pyo3_log::init();
    m.add_class::<ClassicESNModel>()?;
    // m.add_class::<MultiModel>()?;
    Ok(())
}

#[cfg(test)]
mod test {
    use super::ModelInput;
    use crate::{esn::RcModel, ridge::RidgeRegression};
    use matfile::{Array, NumericData};
    use nalgebra::DMatrix;
    use smartcore::preprocessing::categorical::OneHotEncoder;

    fn extract_double(data: Option<&Array>) -> Vec<f64> {
        let data = data.unwrap();
        let size = data.size();
        let NumericData::Double { real, .. } = data.data() else {
            panic!()
        };
        real.clone()
    }

    fn extract_model_input(data: Option<&Array>) -> ModelInput {
        let data = data.unwrap();
        let size = data.size();
        let NumericData::Double { real, .. } = data.data() else {
            panic!()
        };

        let inputs = (0..size[0])
            .map(|ep| {
                DMatrix::from_row_slice(
                    size[1],
                    size[2],
                    &real[ep * size[1] * size[2]..(ep + 1) * size[1] * size[2]],
                )
            })
            .collect::<Vec<_>>();

        ModelInput {
            episodes: size[0],
            time: size[1],
            vars: size[2],
            inputs,
        }
    }

    // kinda retarded but whatever
    fn one_hot_encode(input: Vec<f64>) -> DMatrix<f64> {
        let categories = input.iter().map(|i| i.round() as u64).collect::<Vec<_>>();
        let max_category = *categories.iter().max().unwrap();
        let mut encoded: DMatrix<f64> = DMatrix::zeros(categories.len(), max_category as usize);
        for (i, c) in categories.iter().enumerate() {
            encoded.row_mut(i)[*c as usize - 1] = 1.;
        }
        return encoded;
    }

    // This example reproduces the classification example from the Multivariate classification
    // example. Ridge regression is also implemented by hand as I do not trust the already existing
    // implementations
    #[test]
    fn reproduce_test() {
        let file = std::fs::File::open("/home/gaborfeher/ascent/quad/data/JpVow.mat").unwrap();
        let mat_file = matfile::MatFile::parse(file).unwrap();

        // [T]
        let Xtr = extract_model_input(mat_file.find_by_name("X"));
        let Xte = extract_model_input(mat_file.find_by_name("Xte"));
        let Ytr = extract_double(mat_file.find_by_name("Y"));
        let Yte = extract_double(mat_file.find_by_name("Yte"));

        let mut rc_model = RcModel {
            n_internal_units: 500,
            connectivity: 0.3,
            spectral_radius: 0.99,
            input_scaling: 0.2,
            embedding: RidgeRegression::new(1.),
            readout: RidgeRegression::new(1.),
        };

        let Ytr = one_hot_encode(Ytr);
        let Yte = one_hot_encode(Yte);
        rc_model.fit(Xtr, Ytr);
        let pred = rc_model.predict(Xte);
        println!("{pred:?}");

        // println!("{Yte:?}");
    }
}
