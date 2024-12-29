use matfile::MatFile;
use nalgebra::{Complex, ComplexField, DMatrix};
use rand::thread_rng;
use rand_distr::{Bernoulli, Distribution, Uniform};
use smartcore::metrics::{f1::F1, Metrics};

use crate::{
    extract_double,
    input::{RcInput, TSInput},
    one_hot_encode,
    representation::{LastStateRepr, OutputRepr, Repr, RepresentationType},
    ridge::RidgeRegression, // ModelInput,
};

pub struct Reservoir {
    pub n_internal_units: usize,
    pub input_scaling: f64,
    pub internal_weights: DMatrix<f64>,
    pub input_weights: Option<DMatrix<f64>>,
}

impl Reservoir {
    pub fn new(
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
        // let eigenvalues = internal_weights.clone().eigenvalues().unwrap();
        let eigenvalues = internal_weights.clone().schur().complex_eigenvalues();
        let max_eigenvalue = eigenvalues
            .iter()
            .cloned()
            .map(Complex::abs)
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

    pub fn set_input_weights(&mut self, nvars: usize) {
        self.input_weights = Some(Self::input_weights(
            self.n_internal_units,
            nvars,
            self.input_scaling,
        ));
    }

    // current input [episodes, vars]
    pub fn integrate(
        &mut self,
        current_input: DMatrix<f64>,
        previous_state: DMatrix<f64>,
    ) -> DMatrix<f64> {
        let state_before_tanh = &self.internal_weights * previous_state.transpose()
            + self.input_weights.as_ref().unwrap() * current_input.transpose();
        state_before_tanh.map(|e| e.tanh()).transpose()
    }

    pub fn compute_state_matricies(&mut self, input: &Box<dyn RcInput>) -> Vec<DMatrix<f64>> {
        let (eps, time, _) = input.shape();
        let n_internal_units = self.n_internal_units;
        let mut states: Vec<DMatrix<f64>> = vec![DMatrix::zeros(time, n_internal_units); eps];
        let mut previous_state: DMatrix<f64> = DMatrix::zeros(eps, n_internal_units);

        for t in 0..time {
            let current_input = input.input_at_time(t);
            previous_state = self.integrate(current_input, previous_state);
            for ep in 0..eps {
                states[ep].set_row(t, &previous_state.row(ep));
            }
        }

        states
    }
}

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
            representation,
            readout,
        }
    }

    pub fn fit(&mut self, input: Box<dyn RcInput>, categories: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.fit_multiple_cholesky2(input_repr, categories);
    }

    pub fn predict(&mut self, input: Box<dyn RcInput>) -> Vec<usize> {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        let logits = self.readout.predict(input_repr);
        logits
            .row_iter()
            .map(|row| row.transpose().argmax().0)
            .collect::<Vec<_>>()
    }
}

// I guess we should have this a bit better
pub fn fit_and_predict(model: &mut RcModel, mat_file: MatFile) -> f64 {
    let xtr = TSInput::from_mat_array(mat_file.find_by_name("X").unwrap());
    let ytr = one_hot_encode(extract_double(mat_file.find_by_name("Y")));

    let xte = TSInput::from_mat_array(mat_file.find_by_name("Xte").unwrap());
    let yte = extract_double(mat_file.find_by_name("Yte"));

    model.esn.set_input_weights(xtr.vars);
    model.fit(Box::new(xtr), ytr);
    let pred = model
        .predict(Box::new(xte))
        .iter()
        .map(|x| *x as f64 + 1.)
        .collect::<Vec<_>>();
    F1::new_with(1.).get_score(&yte, &pred)
}

#[cfg(test)]
mod test {
    use super::fit_and_predict;
    use crate::{esn::RcModel, representation::RepresentationType, ridge::RidgeRegression};

    // This example reproduces the classification example from the Multivariate classification
    // example. Ridge regression is also implemented by hand as I do not trust the already existing
    // implementations
    #[test]
    fn reproduce_test() {
        let file = std::fs::File::open("/home/gabor/ascent/quad/data/JpVow.mat").unwrap();
        let mat_file = matfile::MatFile::parse(file).unwrap();

        let mut rc_model = RcModel::new(
            500,
            0.3,
            0.99,
            0.2,
            // RepresentationType::Output(1.),
            RepresentationType::Output(1.),
            RidgeRegression::new(1.),
        );

        let f1 = fit_and_predict(&mut rc_model, mat_file);
        println!("f1: {f1:?}");
    }
}
