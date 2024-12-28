use nalgebra::DMatrix;
use rand::thread_rng;
use rand_distr::{Bernoulli, Distribution, Uniform};

use super::ModelInput;

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
        let internal = &self.internal_weights * previous_state.transpose();
        let state_before_tanh = &self.internal_weights * previous_state.transpose()
            + self.input_weights.as_ref().unwrap() * current_input.transpose();
        state_before_tanh.map(|e| e.tanh()).transpose()
    }

    pub fn compute_state_matricies(&mut self, input: &ModelInput) -> Vec<DMatrix<f64>> {
        let n_internal_units = self.n_internal_units;
        let mut states: Vec<DMatrix<f64>> =
            vec![DMatrix::zeros(input.time, n_internal_units); input.episodes];
        let mut previous_state: DMatrix<f64> = DMatrix::zeros(input.episodes, n_internal_units);

        for t in 0..input.time {
            let current_input = input.input_at_time(t);
            previous_state = self.integrate(current_input, previous_state);
            for ep in 0..input.episodes {
                states[ep].set_row(t, &previous_state.row(ep));
            }
            println!("{previous_state}")
        }

        states
    }
}

#[cfg(test)]
mod test {
    use crate::esn::extract_model_input;

    use super::Reservoir;
    use nalgebra::DMatrix;

    #[test]
    fn misintegration_test() {
        let mut res = Reservoir::new(500, 0.2, 0.99, 0.2);
        res.set_input_weights(10);
        let mut previous_state: DMatrix<f64> = DMatrix::zeros(1, 500);
        let current_input = DMatrix::zeros(1, 10).map(|x: f64| 10.);
        let mut states = vec![];
        for t in 0..29 {
            let state = res.integrate(current_input.clone(), previous_state);
            states.push(state.clone());
            previous_state = state;
        }
        println!("{}", states[15]);
    }

    #[test]
    fn misintegration_test_two() {
        let mut res = Reservoir::new(500, 0.2, 0.99, 0.2);
        let file = std::fs::File::open("/Users/afgthecat/projects/quad/data/JpVow.mat").unwrap();
        let mat_file = matfile::MatFile::parse(file).unwrap();
        let mut Xtr = extract_model_input(mat_file.find_by_name("X"));
        Xtr.truncate();
        res.set_input_weights(Xtr.vars);
        let mut states = vec![];
        let mut previous_state: DMatrix<f64> = DMatrix::zeros(1, 500);
        for t in 0..29 {
            let current_input = Xtr.input_at_time(t);
            let state = res.integrate(current_input.clone(), previous_state);
            states.push(state.clone());
            previous_state = state;
        }

        for t in 0..29 {
            println!("{t} ---");
            println!("{:?}", states[t].data)
        }
    }
}
