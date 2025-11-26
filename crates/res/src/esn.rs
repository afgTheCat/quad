use nalgebra::{Complex, ComplexField, DMatrix};
use rand::thread_rng;
use rand_distr::{Bernoulli, Distribution, Uniform};
use serde::{Deserialize, Serialize};

use crate::input::RcInput;

// classical ESN
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Esn {
    pub n_internal_units: usize,
    pub input_scaling: f64,
    pub internal_weights: DMatrix<f64>,
    pub input_weights: Option<DMatrix<f64>>,
}

impl Esn {
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

    pub fn integrate(
        &self,
        // [episodes, vars] <= current input
        current_input: DMatrix<f64>,
        previous_state: DMatrix<f64>,
    ) -> DMatrix<f64> {
        let state_before_tanh = &self.internal_weights * previous_state.transpose()
            + self.input_weights.as_ref().unwrap() * current_input.transpose();
        state_before_tanh.map(|e| e.tanh()).transpose()
    }

    // computes the strate matricies for each episode
    // episodes are represented as DMatrix where each row t represents the states at that time
    pub fn compute_state_matricies(&self, input: &Box<dyn RcInput>) -> Vec<DMatrix<f64>> {
        let (eps, time, _) = input.shape();
        let n_internal_units = self.n_internal_units;
        let mut states: Vec<DMatrix<f64>> = vec![DMatrix::zeros(time, n_internal_units); eps];
        let mut previous_state: DMatrix<f64> = DMatrix::zeros(eps, n_internal_units);

        for t in 0..time {
            let current_input = input.input_at_time(t);
            previous_state = self.integrate(current_input, previous_state);

            for (ep, state) in states.iter_mut().enumerate() {
                state.set_row(t, &previous_state.row(ep));
            }
        }

        states
    }
}
