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

        let internal_weights = DMatrix::from_row_slice(
            20,
            20,
            &[
                0.0,
                0.59372247,
                0.0,
                -0.47234392,
                0.0,
                0.23842437,
                0.0,
                -0.54493076,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.4828025,
                -0.16800314,
                0.25968145,
                0.0,
                -0.09882382,
                0.0,
                0.38556545,
                -0.15619888,
                -0.40947042,
                0.0,
                0.0,
                -0.31391958,
                0.0,
                0.0,
                0.01472447,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.40346616,
                0.0,
                0.0,
                0.0,
                -0.00339348,
                0.4969845,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.33124302,
                0.33403448,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.2544824,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.44305598,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.56885164,
                0.07462908,
                0.0,
                -0.05962856,
                0.05698565,
                0.0,
                0.10087295,
                0.0,
                0.0,
                -0.13135448,
                0.0,
                0.0,
                0.29346793,
                -0.16415214,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.41727489,
                0.0,
                -0.05605245,
                0.0,
                -0.63556718,
                0.5318507,
                0.0,
                0.01303058,
                0.0,
                -0.55032277,
                -0.29877251,
                0.0,
                0.0,
                0.0,
                -0.09053707,
                0.0,
                0.0,
                0.0,
                0.29535405,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.44159334,
                0.0,
                -0.12844811,
                0.0,
                -0.28161287,
                0.38999413,
                0.0,
                0.0,
                0.0,
                0.247197,
                0.0,
                -0.05564653,
                0.0,
                0.0,
                -0.12066676,
                0.0,
                -0.17095376,
                0.0,
                0.22381106,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.28931757,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.40700064,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.31295889,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.11835816,
                0.0,
                0.0,
                -0.13961451,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.58093118,
                0.0,
                0.0,
                0.0,
                0.59948392,
                0.0,
                0.0,
                0.0,
                0.0,
                0.57927741,
                0.0,
                -0.18640507,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.22411259,
                0.0,
                0.0,
                0.0,
                -0.02813732,
                -0.05385068,
                0.4540519,
                0.0,
                0.0,
                -0.59239816,
                0.0,
                0.0,
                0.0,
                -0.57839822,
                0.0,
                0.0,
                0.0,
                -0.10336747,
                0.46534906,
                0.0,
                0.0,
                0.41045406,
                0.0,
                0.0,
                0.23596726,
                0.0,
                0.0,
                0.0,
                0.09854208,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.29552816,
                0.18936731,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.60443099,
                0.0,
                0.52887403,
                0.0,
                -0.0522221,
                0.0,
                0.0,
                0.0,
                0.62679524,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.25131408,
                0.0,
                0.16767954,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.61520466,
                0.0,
                -0.60710792,
                -0.2580935,
                0.0,
                0.0,
                0.36086993,
                0.597314,
                0.0,
                -0.21705572,
                -0.45927494,
                0.0,
                0.0,
                0.17699714,
                0.0,
                0.5255288,
                0.0,
                0.37535891,
                0.024502,
                0.0,
                0.0,
                0.47116246,
                0.04706206,
                0.0,
                0.0,
                -0.35862647,
                0.0,
                0.0,
                0.0,
                0.11395535,
                0.0,
                0.46902859,
                0.59766099,
                -0.57648106,
                0.0,
                -0.53387126,
                0.0,
                0.52560742,
                0.0,
                0.0,
                0.0,
                0.58782793,
                0.41903185,
                0.35198917,
                0.0,
                0.0,
                0.0,
                0.25626692,
                0.0,
                0.5970748,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.33819866,
                0.0,
                0.0,
                -0.05445576,
                -0.47939132,
                -0.18814568,
                0.0,
                0.0,
                0.0,
                -0.47808417,
                0.18143499,
                0.5584504,
                0.0,
                0.0,
                0.0,
                -0.62919158,
                0.0,
                0.29161195,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.52315025,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.41477972,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.54698523,
                0.0,
                0.0,
                0.0,
                0.36254523,
                0.0,
                0.0,
                0.0,
                0.48065489,
                0.0,
                0.0,
                0.0,
                -0.39409164,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.35649873,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.13212337,
                -0.3094523,
                -0.34832737,
                0.0,
                0.0,
                -0.52084713,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.05705522,
            ],
        );

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
        // self.input_weights = Some(Self::input_weights(
        //     self.n_internal_units,
        //     nvars,
        //     self.input_scaling,
        // ));
        //
        self.input_weights = Some(DMatrix::from_row_slice(
            20,
            12,
            &[
                0.2, 0.2, 0.2, 0.2, -0.2, 0.2, -0.2, 0.2, 0.2, -0.2, 0.2, 0.2, 0.2, 0.2, -0.2,
                -0.2, -0.2, 0.2, 0.2, 0.2, 0.2, 0.2, -0.2, 0.2, -0.2, 0.2, -0.2, 0.2, 0.2, -0.2,
                -0.2, 0.2, -0.2, 0.2, -0.2, 0.2, 0.2, 0.2, 0.2, 0.2, -0.2, -0.2, 0.2, -0.2, 0.2,
                0.2, -0.2, -0.2, -0.2, -0.2, 0.2, -0.2, 0.2, -0.2, -0.2, -0.2, 0.2, -0.2, -0.2,
                -0.2, -0.2, -0.2, 0.2, -0.2, -0.2, -0.2, 0.2, -0.2, 0.2, -0.2, 0.2, -0.2, 0.2, 0.2,
                0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, 0.2, 0.2, -0.2, 0.2, -0.2,
                0.2, 0.2, -0.2, 0.2, -0.2, 0.2, -0.2, -0.2, 0.2, -0.2, 0.2, -0.2, 0.2, -0.2, 0.2,
                0.2, -0.2, 0.2, 0.2, 0.2, -0.2, 0.2, -0.2, 0.2, 0.2, -0.2, 0.2, -0.2, 0.2, 0.2,
                0.2, 0.2, 0.2, 0.2, 0.2, 0.2, -0.2, 0.2, -0.2, -0.2, 0.2, -0.2, 0.2, -0.2, -0.2,
                -0.2, 0.2, 0.2, 0.2, 0.2, 0.2, -0.2, 0.2, -0.2, -0.2, 0.2, 0.2, 0.2, -0.2, 0.2,
                0.2, 0.2, -0.2, 0.2, -0.2, 0.2, -0.2, 0.2, 0.2, 0.2, -0.2, -0.2, 0.2, -0.2, 0.2,
                0.2, 0.2, 0.2, -0.2, -0.2, 0.2, -0.2, 0.2, -0.2, -0.2, -0.2, 0.2, -0.2, -0.2, 0.2,
                0.2, -0.2, -0.2, 0.2, 0.2, -0.2, 0.2, 0.2, 0.2, 0.2, 0.2, -0.2, -0.2, -0.2, -0.2,
                0.2, 0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, 0.2, -0.2, -0.2, -0.2, -0.2, 0.2,
                -0.2, 0.2, 0.2, 0.2, -0.2, 0.2, -0.2, -0.2, 0.2, -0.2, -0.2, 0.2, 0.2, 0.2, 0.2,
                -0.2, 0.2, -0.2, 0.2, 0.2, -0.2, 0.2, 0.2, -0.2, -0.2, 0.2, -0.2, -0.2, -0.2, -0.2,
                -0.2, -0.2, 0.2,
            ],
        ))
    }

    // current input [episodes, vars]
    pub fn integrate(
        &mut self,
        current_input: DMatrix<f64>,
        previous_state: DMatrix<f64>,
    ) -> DMatrix<f64> {
        // let internal = &self.internal_weights * previous_state.transpose();
        println!(
            "dot {}",
            self.internal_weights.clone() * previous_state.transpose()
        );
        println!("input weights: {}", self.input_weights.as_ref().unwrap());
        println!(
            "input {}",
            self.input_weights.as_ref().unwrap() * current_input.transpose()
        );
        let state_before_tanh = &self.internal_weights * previous_state.transpose()
            + self.input_weights.as_ref().unwrap() * current_input.transpose();
        println!("state before tanh {state_before_tanh}");
        let asd = state_before_tanh.map(|e| e.tanh()).transpose();
        println!("state after tanh: {asd}");
        asd
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
        let n_internal_units = 20;
        let mut res = Reservoir::new(n_internal_units, 0.2, 0.99, 0.2);
        let file = std::fs::File::open("/home/gabor/ascent/quad/data/JpVow.mat").unwrap();
        let mat_file = matfile::MatFile::parse(file).unwrap();
        let mut xtr = extract_model_input(mat_file.find_by_name("X"));
        xtr.truncate();
        res.set_input_weights(xtr.vars);
        let mut states = vec![];
        let mut previous_state: DMatrix<f64> = DMatrix::zeros(1, n_internal_units);
        for t in 0..29 {
            let current_input = xtr.input_at_time(t);
            println!("current input {current_input}");
            let state = res.integrate(current_input.clone(), previous_state);
            println!("{t} ---");
            println!("{:?}", state);
            states.push(state.clone());
            previous_state = state;
        }
    }
}
