use bevy::math::{DVec3, DVec4};

#[derive(Debug, Clone)]
pub struct StateUpdatePacket {
    orientation: DVec4,
    angular_velocity: DVec3,
    linear_velocity: DVec3,
    motor_rpm: DVec4,
    motor_t: DVec4,
    beep: bool,
}

impl StateUpdatePacket {
    pub fn new(
        orientation: DVec4,
        angular_velocity: DVec3,
        linear_velocity: DVec3,
        motor_rpm: DVec4,
        motor_t: DVec4,
        beep: bool,
    ) -> Self {
        Self {
            orientation,
            angular_velocity,
            linear_velocity,
            motor_rpm,
            motor_t,
            beep,
        }
    }
}
