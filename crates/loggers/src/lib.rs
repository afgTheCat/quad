pub mod db_logger;
pub mod empty_logger;
pub mod file_logger;
pub mod rerun_logger;

use flight_controller::{BatteryUpdate, Channels, GyroUpdate, MotorInput};
use serde::{Deserialize, Serialize};
use std::{any::Any, time::Duration};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SnapShot {
    pub duration: Duration,
    pub motor_input: MotorInput,
    pub battery_update: BatteryUpdate,
    pub gyro_update: GyroUpdate,
    pub channels: Channels,
    // // TODO: this has a lot, I think this should be enough
    // pub current_frame: SimulationFrame,
}

impl SnapShot {
    pub fn new(
        duration: Duration,
        motor_input: MotorInput,
        battery_update: BatteryUpdate,
        gyro_update: GyroUpdate,
        channels: Channels,
    ) -> Self {
        Self {
            duration,
            motor_input,
            battery_update,
            gyro_update,
            channels,
        }
    }
}

// This is what we need to save
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct FlightLog {
    pub simulation_id: String,
    pub steps: Vec<SnapShot>,
}

impl FlightLog {
    pub fn new(simulation_id: String, steps: Vec<SnapShot>) -> Self {
        Self {
            simulation_id,
            steps,
        }
    }

    pub fn downsample(&mut self, target_dt: Duration) {
        let mut t = self.steps[0].duration;
        let mut new_steps = vec![self.steps[0].clone()];
        for sample in self.steps.iter() {
            if sample.duration - t >= target_dt {
                t = sample.duration;
                new_steps.push(sample.clone());
            }
        }
        self.steps = new_steps
    }
}

pub trait Logger: Sync + Send + Any {
    fn log_time_stamp(&mut self, snapshot: SnapShot);
    fn flush(&mut self);
    // fn set_simulation_id(&mut self, simulation_id: &str);
}
