use core::f64;
use nalgebra::{clamp, DMatrix, DVector};
use pyo3::prelude::*;
use rand::{
    distributions::{Bernoulli, Uniform},
    prelude::Distribution,
    thread_rng,
};

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
            .zip(v.into_iter())
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

// TODO: const generics
#[pyclass]
pub struct ClassicESNModel {
    internal_weights: DMatrix<f64>,
    input_weights: Option<DMatrix<f64>>,
    bias_vector: DVector<f64>,
}

impl ClassicESNModel {
    // This is pretty much the reservoir
    fn integrate(&self, input: DVector<f64>, state: DVector<f64>) -> DVector<f64> {
        let Some(input_weights) = self.input_weights.as_ref() else {
            panic!()
        };
        let state_before_tanh = &self.internal_weights * state.transpose()
            + input_weights * input.transpose()
            + &self.bias_vector;
        state_before_tanh.map(|e| e.tanh())
    }

    fn initialize_internal_weights(
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
}

#[pymethods]
impl ClassicESNModel {
    #[new]
    fn new(n_internal_units: usize) -> PyResult<Self> {
        let internal_weights = Self::initialize_internal_weights(n_internal_units, 0.3, 0.99);
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

#[pymodule]
fn res(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // pyo3_log::init();
    m.add_class::<ClassicESNModel>()?;
    // m.add_class::<MultiModel>()?;
    Ok(())
}
