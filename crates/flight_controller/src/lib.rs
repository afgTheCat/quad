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
            self.throttle as f32,
            self.roll as f32,
            self.pitch as f32,
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
    fn update(&self, update: FlightControllerUpdate) -> MotorInput;
    fn set_armed(&self);
}

impl Default for MotorInput {
    fn default() -> Self {
        Self([1.; 4])
    }
}

#[cfg(test)]
mod test {
    use crate::{
        controllers::bf_controller::BFController, BatteryUpdate, Channels, FlightController,
        FlightControllerUpdate, GyroUpdate,
    };

    #[test]
    fn _bf_controller() {
        let controller = BFController;
        controller.init();
        controller.set_armed();

        let battery_update = BatteryUpdate {
            bat_voltage_sag: 1.,
            bat_voltage: 1.,
            amperage: 1.,
            m_ah_drawn: 1.,
            cell_count: 4,
        };

        let gyro_update = GyroUpdate {
            rotation: [0., 0., 0., 0.],
            acc: [0., 0., 0.],
            gyro: [0., 0., 0.],
        };

        let channels = Channels {
            throttle: 0.5,
            yaw: 0.5,
            pitch: 0.5,
            roll: 0.5,
        };

        for _ in 0..10 {
            let flight_controller_update = FlightControllerUpdate {
                battery_update,
                gyro_update,
                channels,
            };
            controller.update(flight_controller_update);
        }
    }
}
