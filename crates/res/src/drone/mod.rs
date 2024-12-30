use crate::{
    esn::Reservoir,
    input::RcInput,
    representation::{LastStateRepr, OutputRepr, Repr, RepresentationType},
    ridge::RidgeRegression,
};
use flight_controller::FlightControllerUpdate;
use nalgebra::DMatrix;

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

    // this is kinda different, but should be ok
    pub fn fit(&mut self, input: Box<dyn RcInput>, data_points: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        self.readout
            .fit_multiple_svd(res_states[0].clone(), &data_points);
    }

    pub fn predict(&mut self, input: Box<dyn RcInput>) -> DMatrix<f64> {
        let res_states = self.esn.compute_state_matricies(&input);
        self.readout.predict(res_states[0].clone())
    }
}

#[cfg(test)]
mod test {
    use super::DroneRc;
    use crate::{input::FlightInput, representation::RepresentationType, ridge::RidgeRegression};
    use db::{AscentDb, FlightLogEvent};
    use flight_controller::MotorInput;
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

        drone_rc.fit(Box::new(input.clone()), data_points);
        let predicted_points = drone_rc.predict(Box::new(input));
        for col in predicted_points.row_iter().skip(10000).take(1000) {
            // let asd = col.get(0);
            println!("predicted motor input: {}", col);
        }

        // TODO: lets not concern ourselfs with this
        // let new_flight_log = flight_log
        //     .iter()
        //     .zip(predicted_points.row_iter())
        //     .map(|(fl, pp)| FlightLogEvent {
        //         range: fl.range.clone(),
        //         motor_input: MotorInput {
        //             input: [
        //                 f64::clamp(*pp.get(0).unwrap(), 0., 1.),
        //                 f64::clamp(*pp.get(1).unwrap(), 0., 1.),
        //                 f64::clamp(*pp.get(2).unwrap(), 0., 1.),
        //                 f64::clamp(*pp.get(3).unwrap(), 0., 1.),
        //             ],
        //         },
        //         battery_update: fl.battery_update.clone(),
        //         gyro_update: fl.gyro_update.clone(),
        //         channels: fl.channels.clone(),
        //     })
        //     .collect::<Vec<_>>();
        // db.write_flight_logs("fake_simulation", &new_flight_log);
    }
}
