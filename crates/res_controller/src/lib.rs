use nalgebra::DMatrix;
use res::{
    esn::Esn,
    input::RcInput,
    representation::{
        AllStatesForSingleEp, LastStateRepr, OutputRepr, Representation, RepresentationType,
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

    pub fn predict(&mut self, input: Box<dyn RcInput>) -> DMatrix<f64> {
        let res_states = self.esn.compute_state_matricies(&input);
        self.readout.predict(res_states[0].clone())
    }
}
