use crate::{
    extract_double,
    input::{RcInput, TSInput},
    one_hot_encode,
    representation::{LastStateRepr, OutputRepr, Representation, RepresentationType},
};
use matfile::MatFile;
use nalgebra::{clamp, DVector};
use nalgebra::{Complex, ComplexField, DMatrix};
use rand::thread_rng;
use rand_distr::{Bernoulli, Distribution, Uniform};
use ridge::{ridge2::ElasticNetWrapper, RidgeRegression};
use serde::{Deserialize, Serialize};
use smartcore::metrics::{f1::F1, Metrics};

pub struct GenericModelCore {
    pub a: DVector<f64>,
    pub b: DVector<f64>,
    pub c: DVector<f64>,
    pub d: DVector<f64>,
    pub connections: DMatrix<f64>,
    pub v: DVector<f64>,
    pub u: DVector<f64>,
}

impl GenericModelCore {
    pub fn new(
        a: DVector<f64>,
        b: DVector<f64>,
        c: DVector<f64>,
        d: DVector<f64>,
        v: DVector<f64>,
        u: DVector<f64>,
        connections: DMatrix<f64>,
    ) -> Self {
        Self {
            a,
            b,
            c,
            d,
            v,
            u,
            connections,
        }
    }

    pub fn diffuse(&mut self, mut input: DVector<f64>) -> DVector<f64> {
        let firings = self
            .v
            .into_iter()
            .enumerate()
            .filter(|(_, v)| (**v > 30.))
            .map(|(i, _)| i)
            .collect::<Vec<_>>();
        for i in firings {
            self.v[i] = self.c[i];
            self.u[i] += self.d[i];
            input += self.connections.column(i);
        }
        input
    }

    pub fn excite(&mut self, input: DVector<f64>, dt: f64) -> DVector<f64> {
        self.v +=
            dt * ((0.04 * &self.v * &self.v + 5. * &self.v - &self.u + input).add_scalar(140.));
        self.u += dt * &self.a * (&self.b * &self.v - &self.u);
        self.v.clone()
    }
}

struct ModelState {
    pub v: DVector<f64>,
    pub u: DVector<f64>,
}

/// Same as the original reservoir, only that is contains multiple unreleated reservoirs
pub struct MultiModelCore {
    pub a: DVector<f64>,
    pub b: DVector<f64>,
    pub c: DVector<f64>,
    pub d: DVector<f64>,
    pub connections: DMatrix<f64>,
    states: Vec<ModelState>,
}

impl MultiModelCore {
    pub fn new(
        a: DVector<f64>,
        b: DVector<f64>,
        c: DVector<f64>,
        d: DVector<f64>,
        v: Vec<DVector<f64>>,
        u: Vec<DVector<f64>>,
        connections: DMatrix<f64>,
    ) -> Self {
        let states = u
            .into_iter()
            .zip(v)
            .map(|(u, v)| ModelState { u, v })
            .collect::<Vec<_>>();
        Self {
            a,
            b,
            c,
            d,
            connections,
            states,
        }
    }

    pub fn diffuse(&mut self, input: Vec<DVector<f64>>) -> Vec<(DVector<f64>, Vec<usize>)> {
        let mut output = vec![];
        for (state, mut input) in self.states.iter_mut().zip(input.into_iter()) {
            let firings = state
                .v
                .into_iter()
                .enumerate()
                .filter(|(_, v)| (**v > 30.))
                .map(|(i, _)| i)
                .collect::<Vec<_>>();
            for i in firings.iter() {
                state.v[*i] = self.c[*i];
                state.u[*i] += self.d[*i];
                input += self.connections.column(*i);
            }
            output.push((input.clone(), firings));
        }
        output
    }

    pub fn excite(
        &mut self,
        input: Vec<(DVector<f64>, Vec<usize>)>,
        dt: f64,
    ) -> Vec<(DVector<f64>, Vec<usize>)> {
        let mut voltages: Vec<(DVector<f64>, Vec<usize>)> = vec![];
        for (state, (input, firings)) in self.states.iter_mut().zip(input.into_iter()) {
            let input = input.map(|i| clamp(i, -100., 50.));
            // NOTE: v += dt * 0.04v^2 + 5v + 140 - u + i
            state.v += dt
                * state.v.zip_zip_map(&state.u, &input, |v, u, i| {
                    0.04 * v.powi(2) + 5. * v + 140. - u + i
                });

            // NOTE: x = (b * v - u)
            let x = state.v.zip_zip_map(&self.b, &state.u, |v, b, u| b * v - u);
            // NOTE: u += dt * a * x = dt * a * (b * v - u)
            state.u += dt * self.a.zip_map(&x, |a, x| a * x);
            voltages.push((state.v.clone(), firings));
        }
        voltages
    }
}

// TODO: serialize this
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

pub struct RcModel {
    pub esn: Esn,
    representation: Representation,
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

    pub fn fit(&mut self, input: Box<dyn RcInput>, categories: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        // This is going to create create a linear model that is capable of predicting the next
        // input based on the reseroirs current state. Actually what we have here are flattened
        // linear models for all 270 training step!
        let input_repr = self.representation.repr(input, res_states);
        // Next we create a mapping from the representation to the categories. Why though? I have
        // no idea! I am not sure if this is ok tbh!
        self.readout.fit_multiple_svd(input_repr, &categories);
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

pub struct RcModel2 {
    pub esn: Esn,
    representation: Representation,
    pub readout: ElasticNetWrapper,
}

impl RcModel2 {
    pub fn new(
        n_internal_units: usize,
        connectivity: f64,
        spectral_radius: f64,
        input_scaling: f64,
        representation: RepresentationType,
        readout: ElasticNetWrapper,
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

    pub fn fit(&mut self, input: Box<dyn RcInput>, categories: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.fit_multiple(&input_repr, &categories);
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
pub fn fit_and_predict(model: &mut RcModel, mat_file: MatFile) {
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
    let f1 = F1::new_with(1.).get_score(&yte, &pred);
    println!("f1: {f1:?}");
}

pub fn fit_and_predict2(model: &mut RcModel2, mat_file: MatFile) {
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
    let f1 = F1::new_with(1.).get_score(&yte, &pred);
    println!("f1: {f1:?}");
}

#[cfg(test)]
mod test {
    use ridge::{ridge2::ElasticNetWrapper, RidgeRegression};

    use super::{fit_and_predict, RcModel};
    use crate::{
        representation::RepresentationType,
        reservoir::{fit_and_predict2, RcModel2},
    };

    // This example reproduces the classification example from the Multivariate classification
    // example. Ridge regression is also implemented by hand as I do not trust the already existing
    // implementations
    #[test]
    fn reproduce_test1() {
        let file = std::fs::File::open("/home/gabor/projects/quad/data/JpVow.mat").unwrap();
        let mat_file = matfile::MatFile::parse(file).unwrap();

        let mut rc_model = RcModel::new(
            500,
            0.3,
            0.99,
            0.2,
            RepresentationType::Output(1.),
            RidgeRegression::new(1.),
        );

        fit_and_predict(&mut rc_model, mat_file);
    }

    #[test]
    fn reproduce_test2() {
        let file = std::fs::File::open("/home/gabor/projects/quad/data/JpVow.mat").unwrap();
        let mat_file = matfile::MatFile::parse(file).unwrap();

        let mut rc_model = RcModel2::new(
            500,
            0.3,
            0.99,
            0.2,
            RepresentationType::Output(1.),
            ElasticNetWrapper::new_ridge(1.),
        );
        fit_and_predict2(&mut rc_model, mat_file);
    }
}
