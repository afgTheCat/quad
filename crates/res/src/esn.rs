use crate::ridge::RidgeRegression;
use nalgebra::{DMatrix, DVector};
use rand::{
    distributions::{Bernoulli, Uniform},
    prelude::Distribution,
    thread_rng,
};

pub struct ClassicESNModel {
    n_internal_units: usize,
    input_scaling: f64,
    internal_weights: DMatrix<f64>,
    input_weights: Option<DMatrix<f64>>,
}

impl ClassicESNModel {
    fn new(
        n_internal_units: usize,
        connectivity: f64,
        spectral_radius: f64,
        input_scaling: f64,
    ) -> Self {
        let internal_weights =
            Self::internal_weights(n_internal_units, connectivity, spectral_radius);
        Self {
            input_scaling,
            n_internal_units,
            internal_weights,
            input_weights: None,
        }
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

    fn set_input_weights(&mut self, input: &ModelInput) {
        self.input_weights = Some(Self::input_weights(
            self.n_internal_units,
            input.vars,
            self.input_scaling,
        ));
    }

    fn integrate(
        &mut self,
        current_input: DMatrix<f64>,
        previous_state: DMatrix<f64>,
    ) -> DMatrix<f64> {
        let state_before_tanh = &self.internal_weights * previous_state.transpose()
            + self.input_weights.as_ref().unwrap() * current_input.transpose();
        state_before_tanh.map(|e| e.tanh()).transpose()
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
    esn_model: ClassicESNModel,
    embedding: RidgeRegression,
    readout: RidgeRegression,
}

impl RcModel {
    fn new(
        n_internal_units: usize,
        connectivity: f64,
        spectral_radius: f64,
        input_scaling: f64,
        embedding: RidgeRegression,
        readout: RidgeRegression,
    ) -> Self {
        let esn_model = ClassicESNModel::new(
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
        self.esn_model.set_input_weights(input);

        for t in 0..input.time {
            let current_input = input.input_at_time(t);
            previous_state = self.esn_model.integrate(current_input, previous_state);
            for ep in 0..input.episodes {
                states[ep].set_row(t, &previous_state.row(ep));
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

#[cfg(test)]
mod test {
    use super::ModelInput;
    use crate::{esn::RcModel, ridge::RidgeRegression};
    use matfile::{Array, NumericData};
    use nalgebra::DMatrix;

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
        let Ytr = extract_double(mat_file.find_by_name("Y"));

        let Xte = extract_model_input(mat_file.find_by_name("Xte"));
        let Yte = extract_double(mat_file.find_by_name("Yte"));

        let mut rc_model = RcModel::new(
            500,
            0.3,
            0.99,
            0.2,
            RidgeRegression::new(1.),
            RidgeRegression::new(1.),
        );

        let Ytr = one_hot_encode(Ytr);
        let Yte = one_hot_encode(Yte);

        rc_model.fit(Xtr, Ytr);

        let pred = rc_model.predict(Xte);
        println!("{pred:?}");

        // println!("{Yte:?}");
    }
}
