use std::time::Duration;

use crate::{FlightController, MotorInput};

#[derive(Debug)]
pub struct NullController {
    scheduler_delta: Duration,
}

impl FlightController for NullController {
    fn init(&self) {}
    fn update(&self, _: f64, _: crate::FlightControllerUpdate) -> crate::MotorInput {
        MotorInput::default()
    }
    fn scheduler_delta(&self) -> std::time::Duration {
        self.scheduler_delta
    }
}

impl Default for NullController {
    fn default() -> Self {
        Self {
            scheduler_delta: Duration::from_micros(50),
        }
    }
}
