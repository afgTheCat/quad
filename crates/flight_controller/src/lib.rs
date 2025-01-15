use nalgebra::DVector;
use std::{ops::Index, time::Duration};

pub mod bindings;
pub mod controllers;

#[derive(Debug, Clone, Copy)]
pub struct MotorInput {
    pub input: [f64; 4],
}

impl Index<usize> for MotorInput {
    type Output = f64;

    fn index(&self, index: usize) -> &f64 {
        &self.input[index]
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
    pub rotation: [f64; 4], // so far it was w, i, j, k
    pub linear_acc: [f64; 3],
    pub angular_velocity: [f64; 3],
}

// each channel between -1 and 1
#[derive(Debug, Clone, Copy)]
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
        let FlightControllerUpdate {
            battery_update:
                BatteryUpdate {
                    bat_voltage_sag,
                    bat_voltage,
                    amperage,
                    m_ah_drawn,
                    ..
                },
            gyro_update:
                GyroUpdate {
                    rotation,
                    linear_acc,
                    angular_velocity,
                },
            channels:
                Channels {
                    throttle,
                    roll,
                    pitch,
                    yaw,
                },
        } = self;
        DVector::from_row_slice(&[
            *bat_voltage_sag,
            *bat_voltage,
            *amperage,
            *m_ah_drawn,
            rotation[0],
            rotation[1],
            rotation[2],
            rotation[3],
            linear_acc[0],
            linear_acc[1],
            linear_acc[2],
            angular_velocity[0],
            angular_velocity[1],
            angular_velocity[2],
            *throttle,
            *roll,
            *yaw,
            *pitch,
        ])
    }
}

pub trait FlightController: Send + Sync + 'static {
    fn init(&self);
    fn deinit(&self);
    fn update(&self, delta_time_us: u64, update: FlightControllerUpdate) -> MotorInput;
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
