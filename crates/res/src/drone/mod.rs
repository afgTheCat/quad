use crate::{
    esn::{reservoir::Reservoir, RcInput},
    ridge::RidgeRegression,
};
use flight_controller::FlightControllerUpdate;
use nalgebra::{DMatrix, DVector};

pub struct DroneRc {
    pub esn: Reservoir,
    pub embedding: RidgeRegression,
    pub readout: RidgeRegression,
}

// All the data that belong to a flight trajectory. [T, V]
struct FlightControllerInputs(DMatrix<f64>);

impl FlightControllerInputs {
    fn from_flight_controller_updates(update: &[FlightControllerUpdate]) -> Self {
        let rows = update
            .iter()
            .map(|up| up.to_rc_input().transpose())
            .collect::<Vec<_>>();
        FlightControllerInputs(DMatrix::from_rows(&rows))
    }
}

struct DroneInputSteps {
    episodes: usize,
    time: usize,
    vars: usize,
    updates: Vec<FlightControllerInputs>,
}

impl DroneInputSteps {
    fn input_at_time(&self, t: usize) -> DMatrix<f64> {
        let mut input_at_t: DMatrix<f64> = DMatrix::zeros(self.episodes, self.vars);
        for (i, ep) in self.updates.iter().enumerate() {
            input_at_t.set_row(i, &ep.0.row(t));
        }
        input_at_t
    }
}

impl DroneRc {
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
            esn: esn_model,
            embedding,
            readout,
        }
    }

    fn compute_state_matricies(&mut self, input: Box<dyn RcInput>) -> Vec<DMatrix<f64>> {
        let n_internal_units = self.esn.n_internal_units;
        let (episodes, time_steps, _) = input.shape();
        let mut states: Vec<DMatrix<f64>> =
            vec![DMatrix::zeros(time_steps, n_internal_units); episodes];
        let mut previous_state: DMatrix<f64> = DMatrix::zeros(episodes, n_internal_units);

        for t in 0..time_steps {
            let current_input = input.input_at_time(t);
            previous_state = self.esn.integrate(current_input, previous_state);
            for ep in 0..episodes {
                states[ep].set_row(t, &previous_state.row(ep));
            }
        }

        states
    }

    pub fn fit(&mut self, input: Box<dyn RcInput>) {}
}
