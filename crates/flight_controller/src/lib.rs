use std::{f64, ops::Index};

pub mod bindings;
pub mod controllers;

#[derive(Debug, Clone, Copy)]
pub struct MotorInput([f64; 4]);

impl Index<usize> for MotorInput {
    type Output = f64;

    fn index(&self, index: usize) -> &f64 {
        &self.0[index]
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct BatteryUpdate {
    pub bat_voltage_sag: f64,
    pub bat_voltage: f64,
    pub amperage: f64,
    pub m_ah_drawn: f64,
    pub cell_count: u8,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct GyroUpdate {
    pub rotation: [f64; 4],
    pub acc: [f64; 3],
    pub gyro: [f64; 3],
}

// each channel between -1 and 1
#[derive(Debug, Clone, Copy, Default)]
pub struct Channels {
    pub throttle: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl Channels {
    // TODO: verify this!
    pub fn to_bf_channels(&self) -> [f32; 8] {
        [
            self.roll as f32,
            self.pitch as f32,
            self.throttle as f32,
            self.yaw as f32,
            0.,
            0.,
            0.,
            0.,
        ]
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct FlightControllerUpdate {
    pub battery_update: BatteryUpdate,
    pub gyro_update: GyroUpdate,
    pub channels: Channels,
}

pub trait FlightController: Send + Sync + 'static {
    fn init(&self);
    fn update(&self, update: FlightControllerUpdate) -> Option<MotorInput>;
}

impl Default for MotorInput {
    fn default() -> Self {
        Self([1.; 4])
    }
}
