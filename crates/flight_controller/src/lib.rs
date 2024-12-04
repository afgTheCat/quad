use std::{f64, ops::Index, time::Duration};

pub mod bindings;
pub mod controllers;

#[derive(Debug, Clone, Copy)]
pub struct MotorInput {
    // scheduler_cycle: u64,
    input: [f64; 4],
}

impl MotorInput {
    fn set_input(&mut self, input: [f64; 4]) {
        self.input = input;
        // self.scheduler_cycle += 1;
    }
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

#[derive(Debug, Clone, Copy, Default)]
pub struct FlightControllerUpdate {
    pub battery_update: BatteryUpdate,
    pub gyro_update: GyroUpdate,
    pub channels: Channels,
}

pub trait FlightController: Send + Sync + 'static {
    fn init(&self);
    fn update(&self, delta_time_us: u64, update: FlightControllerUpdate) -> MotorInput;
    fn scheduler_delta(&self) -> Duration;
}

impl Default for MotorInput {
    fn default() -> Self {
        Self {
            input: [0.; 4],
            // scheduler_cycle: 0,
        }
    }
}

#[cfg(test)]
mod test {
    use crate::{
        controllers::bf_controller::bf2::BFController2, BatteryUpdate, Channels, FlightController,
        FlightControllerUpdate, GyroUpdate,
    };
    use std::time::Instant;

    #[test]
    fn thing_2() {
        let controller = BFController2::new();
        controller.init();
        let start = Instant::now();
        let mut time_ellapsed = start.elapsed();

        loop {
            let battery_update = BatteryUpdate {
                bat_voltage_sag: 4.2,
                bat_voltage: 4.2,
                amperage: 0.,
                m_ah_drawn: 0.,
                cell_count: 4,
            };
            let gyro_update = GyroUpdate {
                rotation: [1., 0., 0., 0.],
                linear_acc: [0., 0., 0.],
                angular_velocity: [0., 0., 0.],
            };
            let channels = Channels {
                throttle: 1.,
                roll: 0.,
                pitch: 0.,
                yaw: 0.,
            };
            let update = FlightControllerUpdate {
                battery_update,
                gyro_update,
                channels,
            };
            controller.update(time_ellapsed.as_micros() as u64, update);
            time_ellapsed = start.elapsed();
        }
    }
}
