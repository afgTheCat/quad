use crate::{
    esn::Reservoir,
    input::RcInput,
    representation::{LastStateRepr, OutputRepr, Repr, RepresentationType},
    ridge::RidgeRegression,
};
use db::FlightLog;
use flight_controller::FlightControllerUpdate;
use nalgebra::{DMatrix, DVector};

pub struct DroneRc {
    pub esn: Reservoir,
    representation: Box<dyn Repr>,
    readout: RidgeRegression,
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

    // this is kinda different, but should be ok
    pub fn fit(&mut self, input: Box<dyn RcInput>, data_points: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.fit_multiple_svd2(input_repr, &data_points);
    }

    pub fn predict(&mut self, input: Box<dyn RcInput>) -> DMatrix<f64> {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.predict(input_repr)
    }
}

#[cfg(test)]
mod test {
    use super::DroneRc;
    use crate::{input::FlightInput, representation::RepresentationType, ridge::RidgeRegression};
    use db::{AscentDb, FlightLogEvent};
    use nalgebra::DMatrix;

    #[test]
    fn train_thing() {
        let db = AscentDb::new("/home/gabor/ascent/quad/data.db");
        let flight_log = db.get_simuation_data(&"7076b699-65d7-40b1-9ecb-0e58d664faf3");
        let mut drone_rc = DroneRc::new(
            500,
            0.3,
            0.99,
            0.2,
            RepresentationType::Output(1.),
            RidgeRegression::new(1.),
        );
        drone_rc.esn.set_input_weights(18);
        let input = FlightInput::new(vec![flight_log.clone()]);
        let data_points = DMatrix::from_columns(
            &flight_log
                .iter()
                .map(FlightLogEvent::to_rc_output)
                .collect::<Vec<_>>(),
        )
        .transpose();

        drone_rc.fit(Box::new(input), data_points);
    }
}
