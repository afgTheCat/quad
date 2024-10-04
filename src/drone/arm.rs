use bevy::math::{DMat3, DVec3, DVec4};

use crate::{
    constants::{MAX_EFFECT_SPEED, M_PI},
    perlin_noise,
};

use super::{motor::Motor, propeller::Propeller, state_packet::StatePacket};

#[derive(Debug, Clone)]
pub struct Arm {
    propeller: Propeller,
    motor: Motor,
    arm_index: usize,
}

impl Arm {
    fn new(propeller: Propeller, motor: Motor, arm_index: usize) -> Self {
        Self {
            propeller,
            motor,
            arm_index,
        }
    }

    // NOTE: Used in the python version
    pub fn calculate_motor_torques(&mut self, vel_up: f64, volts: f64) -> (f64, f64) {
        let m_torque = self.motor.motor_torque(volts, true);
        let p_torque = self.propeller.prop_torque(0., vel_up, self.motor.rpm());
        self.motor.update_motor_torques(p_torque, m_torque);
        (p_torque, m_torque)
    }

    pub fn calculate_rpm(&mut self, dt: f64, volts: f64, m_torque: f64, p_torque: f64) -> f64 {
        let net_torque = m_torque - p_torque;
        let domega = net_torque / f64::max(self.propeller.inertia(), 0.00000001);
        let drpm = (domega * dt) * 60.0 / (2.0 * M_PI);
        let maxdrpm = f64::abs(volts * self.motor.kv() - self.motor.rpm());
        let rpm = self.motor.rpm() + f64::clamp(drpm, -maxdrpm, maxdrpm);
        self.motor.set_rpm(rpm);
        rpm
    }

    fn get_rpm(&self) -> f64 {
        self.motor.get_rpm()
    }

    fn motor_torque(&self, volts: f64) -> f64 {
        self.motor.motor_torque(volts, true)
    }

    fn prop_torque(&self, rpm: f64, vel_up: f64) -> f64 {
        self.propeller.prop_torque(0., vel_up, rpm)
    }

    fn prop_thrust(&self, rpm: f64, vel_up: f64) -> f64 {
        self.propeller.prop_thrust(vel_up, rpm)
    }
}

impl Arm {
    fn calculate_motor_torques_inner(
        &mut self,
        vel_up: f64,
        volts: f64,
        prop_damage: f64,
        is_armed: bool,
    ) -> (f64, f64) {
        let m_torque = self.motor.motor_torque(volts, is_armed);
        let p_torque = self
            .propeller
            .prop_torque(prop_damage, vel_up, self.motor.rpm());
        self.motor.update_motor_torques(p_torque, m_torque);
        (p_torque, m_torque)
    }

    fn motor_thrust(
        &mut self,
        dt: f64,
        rpm: f64,
        rotation: DMat3,
        linear_velocity: DVec3,
        prop_damage: DVec4,
        ground_effect: f64,
    ) -> f64 {
        // TODO: verify
        let up = rotation.x_axis;
        let vel_up = DVec3::dot(linear_velocity, up);
        let prop_health_factor = 1.0 - prop_damage[self.arm_index];
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

        let prop_damage_effect = 1.0
            - (f64::max(0.0, 0.5 * (perlin_noise(self.motor.phase() * speed) + 1.0))
                * prop_damage[self.arm_index]);
        self.propeller.prop_thrust(vel_up, rpm)
            * prop_health_factor
            * ground_effect
            * prop_wash_effect
            * prop_damage_effect
    }

    pub fn update_aux_motor(
        &mut self,
        state_packet: &StatePacket,
        dt: f64,
        rpm: f64,
        m_torque: f64,
        vbat: f64,
        ambient_temp: f64,
        rotation: DMat3,
        linear_velocity: DVec3,
    ) {
        let speed = linear_velocity.length();
        let current = m_torque * self.motor.kv() / 8.3;
        let ground_effect = state_packet.ground_effect(self.arm_index);
        let thrust = self.motor_thrust(
            dt,
            rpm,
            rotation,
            linear_velocity,
            state_packet.prop_damage,
            ground_effect,
        );
        self.motor
            .update_motor_corrected(current, speed, thrust, dt, vbat, ambient_temp);
    }

    /// TODO: This is a corrected version of `calculate_arm_m_torque`
    /// This is also work in progress
    pub fn calculate_arm_m_torque_corrected(
        &mut self,
        state_packet: &StatePacket,
        dt: f64,
        vbat: f64,
        is_armed: bool,
        ambient_temp: f64,
        rotation: DMat3,
        linear_velocity: DVec3,
    ) -> f64 {
        // TODO: verify
        let up = rotation.x_axis;
        let vel_up = DVec3::dot(linear_velocity, up);
        let volts = self.motor.volts(dt, vbat);
        let (p_torque, m_torque) = self.calculate_motor_torques_inner(
            vel_up,
            volts,
            state_packet.prop_damage[self.arm_index],
            is_armed,
        );
        let rpm = self.calculate_rpm(dt, volts, m_torque, p_torque);
        self.update_aux_motor(
            state_packet,
            dt,
            rpm,
            m_torque,
            vbat,
            ambient_temp,
            rotation,
            linear_velocity,
        );
        self.motor.dir() * m_torque
    }

    pub fn calculate_arm_m_torque(
        &mut self,
        state_packet: &StatePacket,
        dt: f64,
        vbat: f64,
        is_armed: bool,
        ambient_temp: f64,
        rotation: DMat3,
        linear_velocity: DVec3,
    ) -> f64 {
        let up = rotation.x_axis;
        let vel_up = DVec3::dot(linear_velocity, up);
        let speed = linear_velocity.length();
        let volts = self.motor.volts(dt, vbat);
        let m_torque = self.motor.motor_torque(volts, is_armed);
        let p_torque = self.propeller.prop_torque(
            state_packet.prop_damage[self.arm_index],
            vel_up,
            self.motor.rpm(),
        );
        let (rpm, current) = if self.motor.burned_out() {
            (0., 0.)
        } else {
            let net_torque = m_torque - p_torque;
            let domega = net_torque / f64::max(self.propeller.inertia(), 0.00000001);
            let drpm = (domega * dt) * 60.0 / (2.0 * M_PI);
            let maxdrpm = f64::abs(volts * self.motor.kv() - self.motor.rpm());
            let rpm = self.motor.rpm() + f64::clamp(drpm, -maxdrpm, maxdrpm);
            let current = m_torque * self.motor.kv() / 8.3;
            (rpm, current)
        };
        let ground_effect = state_packet.ground_effect(self.arm_index);
        let thrust = self.motor_thrust(
            dt,
            rpm,
            rotation,
            linear_velocity,
            state_packet.prop_damage,
            ground_effect,
        );
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
