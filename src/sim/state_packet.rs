use bevy::math::{DVec3, DVec4};

#[derive(Debug, Clone)]
pub struct StatePacket {
    // pub delta: f64,
    // rc_data: [f64; 8], // TODO: readd when hooking up the thing
    // position: DVec3,
    // pub rotation: DMat3,
    // pub angular_velocity: DVec3,
    // pub linear_velocity: DVec3,
    pub motor_imbalance: [DVec3; 4], // TODO: what is this?
    pub gyro_base_noise_amp: f64,
    // gyrobase_noise_freq: f64,
    pub frame_harmonic_1_amp: f64,
    pub frame_harmonic_1_freq: f64,
    pub frame_harmonic_2_amp: f64,
    pub frame_harmonic_2_freq: f64,
    // pub prop_damage: DVec4,
    ground_effect: [f64; 4],
    vbat: f64,
    // pub contact: u8,
    commands: i32,
}

impl StatePacket {
    pub fn new(
        motor_imbalance: [DVec3; 4],
        gyro_base_noise_amp: f64,
        frame_harmonic_1_amp: f64,
        frame_harmonic_1_freq: f64,
        frame_harmonic_2_amp: f64,
        frame_harmonic_2_freq: f64,
        // prop_damage: DVec4,
        ground_effect: [f64; 4],
        vbat: f64,
        commands: i32,
    ) -> Self {
        Self {
            motor_imbalance,
            gyro_base_noise_amp,
            frame_harmonic_1_amp,
            frame_harmonic_1_freq,
            frame_harmonic_2_amp,
            frame_harmonic_2_freq,
            // prop_damage,
            ground_effect,
            vbat,
            commands,
        }
    }

    pub fn ground_effect(&self, i: usize) -> f64 {
        1.0 + ((self.ground_effect[i] * self.ground_effect[i]) * 0.7)
    }
}
