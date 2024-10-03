use bevy::math::{DMat3, DVec3, DVec4};

#[derive(Debug, Clone)]
pub struct StatePacket {
    pub delta: f64,
    rc_data: [f64; 8],
    position: DVec3,
    pub rotation: DMat3,
    pub angular_velocity: DVec3,
    pub linear_velocity: DVec3,
    // length is amplitude per axis
    pub motor_imbalance: [DVec3; 4],
    pub gyro_base_noise_amp: f64,
    gyrobase_noise_freq: f64,
    pub frame_harmonic_1_amp: f64,
    pub frame_harmonic_1_freq: f64,
    pub frame_harmonic_2_amp: f64,
    pub frame_harmonic_2_freq: f64,
    pub prop_damage: DVec4,
    ground_effect: [f64; 4],
    vbat: f64,
    // 1 true 0 false
    pub contact: u8,
    // combination of CommandType (bitmask)
    commands: i32,
}

impl StatePacket {
    pub fn new(
        delta: f64,
        rc_data: [f64; 8],
        position: DVec3,
        rotation: DMat3,
        angular_velocity: DVec3,
        linear_velocity: DVec3,
        motor_imbalance: [DVec3; 4],
        gyro_base_noise_amp: f64,
        gyrobase_noise_freq: f64,
        frame_harmonic_1_amp: f64,
        frame_harmonic_1_freq: f64,
        frame_harmonic_2_amp: f64,
        frame_harmonic_2_freq: f64,
        prop_damage: DVec4,
        ground_effect: [f64; 4],
        vbat: f64,
        contact: u8,
        commands: i32,
    ) -> Self {
        Self {
            delta,
            rc_data,
            position,
            rotation,
            angular_velocity,
            linear_velocity,
            // length is amplitude per axis
            motor_imbalance,
            gyro_base_noise_amp,
            gyrobase_noise_freq,
            frame_harmonic_1_amp,
            frame_harmonic_1_freq,
            frame_harmonic_2_amp,
            frame_harmonic_2_freq,
            prop_damage,
            ground_effect,
            vbat,
            // 1 true 0 false
            contact,
            // combination of CommandType (bitmask)
            commands,
        }
    }

    pub fn ground_effect(&self, i: usize) -> f64 {
        1.0 + ((self.ground_effect[i] * self.ground_effect[i]) * 0.7)
    }
}
