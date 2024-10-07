use super::low_pass_filter::LowPassFilter;
use crate::rng_gen_range;
use bevy::math::{DVec3, DVec4};

#[derive(Debug, Clone, Default)]
pub struct Gyro {
    gyro_noise: DVec3,
    low_pass_filter: [LowPassFilter; 3],
    rotation: DVec4,
    acceleration: DVec3,
    gyro: DVec3,
    gyro_base_noise_amp: f64,
}

impl Gyro {
    fn new(
        gyro_noise: DVec3,
        low_pass_filter: [LowPassFilter; 3],
        rotation: DVec4,
        acceleration: DVec3,
        gyro: DVec3,
        gyro_base_noise_amp: f64,
    ) -> Self {
        Self {
            gyro_noise,
            low_pass_filter,
            rotation,
            acceleration,
            gyro,
            gyro_base_noise_amp,
        }
    }
}

impl Gyro {
    pub fn update_gyro_noise(&mut self) {
        let white_noise_x = rng_gen_range(-1.0..1.) * self.gyro_base_noise_amp;
        let white_noise_y = rng_gen_range(-1.0..1.) * self.gyro_base_noise_amp;
        let white_noise_z = rng_gen_range(-1.0..1.) * self.gyro_base_noise_amp;
        self.gyro_noise[0] = white_noise_x;
        self.gyro_noise[1] = white_noise_y;
        self.gyro_noise[1] = white_noise_z;
    }

    pub fn gyro_noise(&self) -> DVec3 {
        self.gyro_noise
    }

    pub fn low_pass_filter(&mut self) -> &mut [LowPassFilter; 3] {
        &mut self.low_pass_filter
    }

    pub fn set_acceleration(&mut self, acceleration: DVec3) {
        self.acceleration = acceleration
    }

    pub fn set_gyro(&mut self, gyro: DVec3) {
        self.gyro = gyro
    }

    pub fn set_rotation(&mut self, rotation: DVec4) {
        self.rotation = rotation
    }
}
