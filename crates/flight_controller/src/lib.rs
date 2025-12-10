use nalgebra::DVector;
use serde::{Deserialize, Serialize};
use std::{ops::Index, time::Duration};

pub mod controllers;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct MotorInput {
    pub input: [f64; 4],
}

impl Index<usize> for MotorInput {
    type Output = f64;

    fn index(&self, index: usize) -> &f64 {
        &self.input[index]
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct BatteryUpdate {
    pub bat_voltage_sag: f64,
    pub bat_voltage: f64,
    pub amperage: f64,
    pub m_ah_drawn: f64,
    pub cell_count: u64,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct GyroUpdate {
    pub rotation: [f64; 4], // so far it was w, i, j, k
    pub linear_acc: [f64; 3],
    pub angular_velocity: [f64; 3],
}

// each channel between -1 and 1
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Channels {
    pub throttle: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct FlightControllerUpdate {
    pub battery_update: BatteryUpdate,
    pub gyro_update: GyroUpdate,
    pub channels: Channels,
}

impl FlightControllerUpdate {
    pub fn to_rc_input(&self) -> DVector<f64> {
        let Channels {
            throttle,
            roll,
            pitch,
            yaw,
        } = self.channels;
        // Keep the feature ordering in sync with training inputs (throttle, roll, yaw, pitch).
        DVector::from_row_slice(&[throttle, roll, yaw, pitch])
    }
}

pub trait FlightController: Send + Sync + 'static {
    fn init(&self);
    fn update(&self, delta_time: f64, update: FlightControllerUpdate) -> MotorInput;
    fn scheduler_delta(&self) -> Duration;
}

impl Default for MotorInput {
    fn default() -> Self {
        Self { input: [0.; 4] }
    }
}

impl Default for Channels {
    fn default() -> Self {
        Self {
            throttle: -1.,
            roll: 0.,
            pitch: 0.,
            yaw: 0.,
        }
    }
}

impl Channels {
    // TODO: verify this!
    pub fn to_bf_channels(&self) -> [f64; 8] {
        [
            self.roll,
            self.pitch,
            self.throttle,
            self.yaw,
            0.,
            0.,
            0.,
            0.,
        ]
    }
}
