use crate::{FlightController, FlightControllerUpdate, MotorInput};
use res::{drone::DroneRc, input::FlightInput};
use serde::{Deserialize, Serialize};
use std::{sync::Mutex, time::Duration};

#[derive(Serialize, Deserialize)]
pub struct ResController {
    pub model: Mutex<DroneRc>,
}

impl ResController {
    pub fn new(drone_rc: DroneRc) -> Self {
        Self {
            model: Mutex::new(drone_rc),
        }
    }
}

impl FlightController for ResController {
    // TODO: set the reservoir states to zeros
    fn init(&self) {}

    // TODO: consider the delta time us
    fn update(&self, _delta_time: f64, update: FlightControllerUpdate) -> MotorInput {
        let rc_input = update.to_rc_input();
        let input = FlightInput::new_from_rc_input(vec![vec![rc_input]]);
        let mut model = self.model.lock().unwrap();

        let pr = model.predict(Box::new(input));
        let motor_input_1 = f64::clamp(*pr.row(0).get(0).unwrap(), 0., 1.);
        let motor_input_2 = f64::clamp(*pr.row(0).get(1).unwrap(), 0., 1.);
        let motor_input_3 = f64::clamp(*pr.row(0).get(2).unwrap(), 0., 1.);
        let motor_input_4 = f64::clamp(*pr.row(0).get(3).unwrap(), 0., 1.);
        MotorInput {
            input: [motor_input_1, motor_input_2, motor_input_3, motor_input_4],
        }
    }

    fn scheduler_delta(&self) -> std::time::Duration {
        Duration::from_millis(5)
    }
}
