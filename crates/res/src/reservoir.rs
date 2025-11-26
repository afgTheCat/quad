// TODO: this is version 2 of using the izhikevich model. Soon to be here is the Leaky integrate
// and fire model
use nalgebra::DMatrix;
use nalgebra::{clamp, DVector};

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
            .filter(|(_, v)| **v > 30.)
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
