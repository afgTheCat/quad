use nalgebra::DMatrix;
use ridge::ridge2::ElasticNetWrapper;

use crate::{
    input::RcInput,
    representation::{LastStateRepr, OutputRepr, Repr, RepresentationType},
    reservoir::Esn,
};

pub struct DroneRc2 {
    pub esn: Esn,
    representation: Box<dyn Repr>,
    pub readout: ElasticNetWrapper,
}

impl DroneRc2 {
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
        let representation: Box<dyn Repr> = match representation {
            RepresentationType::LastState => Box::new(LastStateRepr::default()),
            RepresentationType::Output(alpha) => Box::new(OutputRepr::new(alpha)),
        };
        Self {
            esn,
            representation,
            readout,
        }
    }

    // this is the same, maybe it works maybe it does not
    pub fn fit(&mut self, body_rates: Box<dyn RcInput>, rc_data: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&body_rates);
        let input_repr = self.representation.repr(body_rates, res_states);
        println!("input repr: {:?}", input_repr.shape());
        println!("rc data shape: {:?}", rc_data.shape());
        self.readout.fit_multiple(&input_repr, &rc_data);
    }

    pub fn predict(&mut self, input: Box<dyn RcInput>) -> DMatrix<f64> {
        let res_states = self.esn.compute_state_matricies(&input);
        self.readout.predict(res_states[0].clone())
    }
}
