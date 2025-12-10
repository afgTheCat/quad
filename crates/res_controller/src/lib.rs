use std::time::Duration;

use flight_controller::{FlightController, FlightControllerUpdate, MotorInput};
use nalgebra::DMatrix;
use res::{
    esn::Esn,
    input::{FlightInput, RcInput},
    representation::{
        AllStatesForSingleEp, BufferedStatesForSingleEp, LastStateRepr, OutputRepr, Representation,
        RepresentationType,
    },
};
use ridge::RidgeRegression;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone)]
pub struct DroneRc {
    pub esn: Esn,
    pub representation: Representation,
    // Could be changed to ElasticNetWrapper
    pub readout: RidgeRegression,
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
        let esn = Esn::new(
            n_internal_units,
            connectivity,
            spectral_radius,
            input_scaling,
        );
        let representation = match representation {
            RepresentationType::LastState => Representation::LastState(LastStateRepr::default()),
            RepresentationType::Output(alpha) => Representation::Output(OutputRepr::new(alpha)),
            RepresentationType::AllStates => {
                Representation::AllStateForSingle(AllStatesForSingleEp)
            }
            RepresentationType::BufferedStates(states) => {
                Representation::BufferedStates(BufferedStatesForSingleEp(states))
            }
        };
        Self {
            esn,
            representation,
            readout,
        }
    }

    // this is the same, maybe it works maybe it does not
    pub fn fit(&mut self, input: Box<dyn RcInput>, motor_inputs: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.fit_multiple_svd(input_repr, &motor_inputs);
    }

    pub fn predict(&self, input: Box<dyn RcInput>) -> DMatrix<f64> {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.predict(input_repr)
    }
}

impl FlightController for DroneRc {
    fn init(&self) {}

    fn update(&self, _delta_time: f64, update: FlightControllerUpdate) -> MotorInput {
        let rc_input = update.to_rc_input();
        let input = FlightInput::new_from_rc_input(vec![vec![rc_input]]);
        let pr = self.predict(Box::new(input));
        let motor_input_1 = f64::clamp(*pr.row(0).get(0).unwrap(), 0., 1.);
        let motor_input_2 = f64::clamp(*pr.row(0).get(1).unwrap(), 0., 1.);
        let motor_input_3 = f64::clamp(*pr.row(0).get(2).unwrap(), 0., 1.);
        let motor_input_4 = f64::clamp(*pr.row(0).get(3).unwrap(), 0., 1.);
        MotorInput {
            input: [motor_input_1, motor_input_2, motor_input_3, motor_input_4],
        }
    }

    fn scheduler_delta(&self) -> Duration {
        Duration::from_millis(5)
    }
}
