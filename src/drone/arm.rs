use bevy::math::{DMat3, DVec3};

use crate::{
    constants::{MAX_EFFECT_SPEED, M_PI},
    perlin_noise,
};

use super::{motor::Motor, propeller::Propeller};

#[derive(Debug, Clone, Default)]
pub struct Arm {
    pub propeller: Propeller,
    pub motor: Motor,
    pub arm_index: usize,
}

impl Arm {
    fn motor_thrust(
        &mut self,
        dt: f64,
        rpm: f64,
        rotation: DMat3,
        linear_velocity: DVec3,
        ground_effect: f64,
    ) -> f64 {
        // TODO: verify
        let up = rotation.x_axis;
        let vel_up = DVec3::dot(linear_velocity, up);
        let speed = linear_velocity.length();
        let speed_factor = f64::min(speed / MAX_EFFECT_SPEED, 1.);
        let mut reverse_thrust = f64::max(
            0.,
            DVec3::dot(
                linear_velocity.normalize(),
                (up * self.motor.thrust()).normalize() * -1.,
            ),
        );
        reverse_thrust = f64::max(0.0, reverse_thrust - 0.5) * 2.;
        reverse_thrust = reverse_thrust * reverse_thrust;
        let prop_wash_noise = self.motor.prop_wash_noise(dt);
        let prop_wash_effect = 1.0 - (speed_factor * prop_wash_noise * reverse_thrust * 0.95);

        let prop_damage_effect =
            1.0 - (f64::max(0.0, 0.5 * (perlin_noise(self.motor.phase() * speed) + 1.0)));
        self.propeller.prop_thrust(vel_up, rpm)
            * ground_effect
            * prop_wash_effect
            * prop_damage_effect
    }

    pub fn calculate_arm_m_torque(
        &mut self,
        ground_effect: f64,
        dt: f64,
        vbat: f64,
        ambient_temp: f64,
        rotation: DMat3,
        linear_velocity: DVec3,
    ) -> f64 {
        let up = rotation.x_axis;
        let vel_up = DVec3::dot(linear_velocity, up);
        let speed = linear_velocity.length();
        let volts = self.motor.volts(dt, vbat);
        let m_torque = self.motor.motor_torque(volts);
        let p_torque = self.propeller.prop_torque(vel_up, self.motor.rpm());
        let net_torque = m_torque - p_torque;
        let domega = net_torque / f64::max(self.propeller.inertia(), 0.00000001);

        // change in rpm
        let drpm = (domega * dt) * 60.0 / (2.0 * M_PI);
        let maxdrpm = f64::abs(volts * self.motor.kv() - self.motor.rpm());
        let rpm = self.motor.rpm() + f64::clamp(drpm, -maxdrpm, maxdrpm);
        let current = m_torque * self.motor.kv() / 8.3;
        // let ground_effect = state_packet.ground_effect(self.arm_index);
        let thrust = self.motor_thrust(dt, rpm, rotation, linear_velocity, ground_effect);
        self.motor.update_motor(
            current,
            speed,
            thrust,
            dt,
            vbat,
            ambient_temp,
            p_torque,
            m_torque,
            rpm,
        );
        self.motor.dir() * m_torque
    }

    pub fn pwm(&self) -> f64 {
        self.motor.pwm()
    }

    pub fn current(&self) -> f64 {
        self.motor.current()
    }

    pub fn thrust(&self) -> f64 {
        self.motor.thrust()
    }

    pub fn motor_pos(&self) -> DVec3 {
        self.motor.position()
    }

    pub fn motor_kv(&self) -> f64 {
        self.motor.kv()
    }

    pub fn motor_rpm(&self) -> f64 {
        self.motor.rpm()
    }

    pub fn motor_noise(&mut self, dt: f64) -> DMat3 {
        self.motor.motor_noise(dt)
    }

    pub fn motor_temp(&self) -> f64 {
        self.motor.temp()
    }
}
