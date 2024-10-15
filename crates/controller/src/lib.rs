mod bindings;
pub mod controllers;

#[derive(Debug, Clone, Copy)]
pub struct Channels {
    throttle: f64,
    roll: f64,
    pitch: f64,
    yaw: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct MotorInput([f64; 4]);

impl Default for MotorInput {
    fn default() -> Self {
        Self([1.; 4])
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct SetPoints {
    roll: f64,
    pitch: f64,
    yaw: f64,
}

trait FlightController {
    fn set_channels(&mut self, channels: Channels);
    fn update_gyro_acc(
        &mut self,
        attitude: [f64; 4],
        acceleration: [f64; 3],
        angular_vel: [f64; 3],
    );
    // TODO: update gps
}
