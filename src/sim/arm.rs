use bevy::{
    math::{DMat3, DVec3},
    prelude::Component,
};

use crate::{
    constants::{MAX_EFFECT_SPEED, MAX_SPEED_PROP_COOLING, M_PI},
    perlin_noise,
};

use super::{
    drone::{rpm_to_hz, shifted_phase},
    low_pass_filter::LowPassFilter,
};

#[derive(Debug, Clone)]
pub struct Propeller {
    prop_max_rpm: f64,
    prop_a_factor: f64,
    prop_torque_factor: f64,
    prop_inertia: f64,
    prop_thrust_factor: DVec3,
}

impl Default for Propeller {
    fn default() -> Self {
        Self {
            prop_max_rpm: 1.,
            prop_a_factor: 0.,
            prop_torque_factor: 0.,
            prop_inertia: 0.1,
            prop_thrust_factor: DVec3::ZERO,
        }
    }
}

impl Propeller {
    pub fn new(
        prop_max_rpm: f64,
        prop_a_factor: f64,
        prop_torque_factor: f64,
        prop_inertia: f64,
        prop_thrust_factor: DVec3,
    ) -> Self {
        Self {
            prop_max_rpm,
            prop_a_factor,
            prop_torque_factor,
            prop_inertia,
            prop_thrust_factor,
        }
    }

    // the thrust a propeller does
    pub fn prop_thrust(&self, vel_up: f64, rpm: f64) -> f64 {
        let prop_f = f64::max(
            0.0,
            self.prop_thrust_factor[0] * vel_up * vel_up
                + self.prop_thrust_factor[1] * vel_up
                + self.prop_thrust_factor[2],
        );
        let max_rpm = f64::max(self.prop_max_rpm, 0.01);
        let prop_a = self.prop_a_factor;
        let b = (prop_f - prop_a * max_rpm * max_rpm) / max_rpm;
        let result = b * rpm + prop_a * rpm * rpm;
        f64::max(result, 0.0)
    }

    pub fn prop_torque(&self, vel_up: f64, rpm: f64) -> f64 {
        self.prop_thrust(vel_up, rpm) * self.prop_torque_factor
    }
}

impl Propeller {
    pub fn inertia(&self) -> f64 {
        self.prop_inertia
    }
}

// TODO: defualt here is has to be manually implemeted
#[derive(Debug, Clone, Default)]
pub struct MotorProps {
    pub position: DVec3, // position relative to the main body
    pub motor_kv: f64,   // kv
    pub motor_r: f64,    // resistence
    pub motor_io: f64,   // idle current
    pub motor_rth: f64,  // thermal resistance (deg C per Watt)
    pub motor_cth: f64,  // thermal heat capacity (joules per deg C)
    pub motor_dir: f64,
    pub motor_max_t: f64,
}

impl MotorProps {
    fn new(
        position: DVec3,
        motor_kv: f64,
        motor_r: f64,
        motor_io: f64,
        motor_rth: f64,
        motor_cth: f64,
        motor_dir: f64,
        motor_max_t: f64,
    ) -> Self {
        Self {
            position,
            motor_kv,
            motor_r,
            motor_io,
            motor_rth,
            motor_cth,
            motor_dir,
            motor_max_t,
        }
    }
}

// TODO: defualt here is has to be manually implemeted
#[derive(Debug, Clone, Default)]
pub struct MotorState {
    pub pwm: f64, // pwm signal in percent [0,1]. Thiw will be
    // updated by BF
    pub pwm_low_pass_filter: LowPassFilter, // low pass filtered pwm value
    pub temp: f64,                          // motor core temp in deg C
    pub current: f64,                       // current running through motor in Amps
    pub rpm: f64,                           // motor revolutions per minute
    pub thrust: f64,                        // thrust output of motor / propeller combo
    pub m_torque: f64,                      // motor torque
    pub p_torque: f64,                      // propeller torque, counter acting motor torque
    pub prop_wash_low_pass_filter: LowPassFilter, // low pass filtered prop wash
    pub phase: f64, // sinusoidal phase of the motor rotation used for noise simulation
    pub phase_harmonic_1: f64, // phase freq * 2
    pub phase_harmonic_2: f64, // phase freq * 3
    pub phase_slow: f64, // phase freq * 0.01f
                    // pub burned_out: bool, // is the motor destroyed by over temp
}

impl MotorState {
    fn new(
        pwm: f64,
        pwm_low_pass_filter: LowPassFilter,
        temp: f64,
        current: f64,
        rpm: f64,
        thrust: f64,
        m_torque: f64,
        p_torque: f64,
        prop_wash_low_pass_filter: LowPassFilter,
        phase: f64,
        phase_harmonic_1: f64,
        phase_harmonic_2: f64,
        phase_slow: f64,
        // burned_out: bool,
    ) -> Self {
        Self {
            pwm,
            pwm_low_pass_filter,
            temp,
            current,
            rpm,
            thrust,
            m_torque,
            p_torque,
            prop_wash_low_pass_filter,
            phase,
            phase_harmonic_1,
            phase_harmonic_2,
            phase_slow,
            // burned_out,
        }
    }
}

impl MotorState {
    fn set_rpm(&mut self, rpm: f64) {
        self.rpm = rpm
    }

    fn get_rpm(&self) -> f64 {
        self.rpm
    }

    fn motor_noise(&mut self, dt: f64) -> DMat3 {
        self.phase = shifted_phase(dt, rpm_to_hz(self.rpm), self.phase);
        self.phase_harmonic_1 = shifted_phase(dt, rpm_to_hz(self.rpm), self.phase_harmonic_1);
        self.phase_harmonic_2 = shifted_phase(dt, rpm_to_hz(self.rpm), self.phase_harmonic_2);
        self.phase_slow = shifted_phase(dt, rpm_to_hz(self.rpm), self.phase_slow);

        let sin_phase = f64::sin(self.phase);
        let sin_phase_h1 = f64::sin(self.phase_harmonic_1);
        let sin_phase_h2 = f64::sin(self.phase_harmonic_2);

        let cos_phase = f64::cos(self.phase);
        let cos_phase_h1 = f64::cos(self.phase_harmonic_1);
        let cos_phase_h2 = f64::cos(self.phase_harmonic_2);
        DMat3 {
            x_axis: DVec3::new(sin_phase, sin_phase_h1, sin_phase_h2),
            y_axis: DVec3::new(cos_phase, cos_phase_h1, cos_phase_h2),
            z_axis: DVec3::new(
                sin_phase + cos_phase,
                sin_phase_h1 + cos_phase_h1,
                sin_phase_h2 + cos_phase_h2,
            ),
        }
    }
}

#[derive(Debug, Clone, Default, Component)]
pub struct Motor {
    pub state: MotorState,
    pub props: MotorProps,
}

impl Motor {
    fn new(state: MotorState, props: MotorProps) -> Self {
        Self { state, props }
    }

    fn update_motor_temp(
        &mut self,
        current: f64,
        speed: f64,
        thrust: f64,
        dt: f64,
        vbat: f64,
        ambient_temp: f64,
    ) {
        let power_draw = f64::abs(current) * vbat;
        let cooling = (1. - f64::exp(-speed * 0.2)) * 100.
            + (f64::min(MAX_SPEED_PROP_COOLING, speed) / MAX_SPEED_PROP_COOLING) * thrust * 4.;

        self.state.temp += ((f64::max(0.0, power_draw - cooling))
            - (self.state.temp - ambient_temp) / self.props.motor_rth)
            / self.props.motor_cth
            * dt;
    }

    pub fn temp(&self) -> f64 {
        self.state.temp
    }
}

impl Motor {
    pub fn set_rpm(&mut self, rpm: f64) {
        self.state.set_rpm(rpm)
    }

    pub fn get_rpm(&self) -> f64 {
        self.state.get_rpm()
    }

    pub fn phase_slow(&self) -> f64 {
        self.state.phase_slow
    }

    pub fn rpm(&self) -> f64 {
        self.state.rpm
    }

    pub fn phase(&self) -> f64 {
        self.state.phase
    }

    // Effective volts calculated by running it through the low pass filter
    pub fn volts(&mut self, dt: f64, vbat: f64) -> f64 {
        self.state
            .pwm_low_pass_filter
            .update(self.state.pwm, dt, 120.0)
            * vbat
    }

    pub fn thrust(&self) -> f64 {
        self.state.thrust
    }

    pub fn position(&self) -> DVec3 {
        self.props.position.clone()
    }

    pub fn motor_torque(&self, volts: f64) -> f64 {
        let kv = self.props.motor_kv;
        let back_emf_v = self.state.rpm / f64::max(kv, 0.0001);
        let base_current = (volts - back_emf_v) / f64::max(self.props.motor_r, 0.0001);
        let current = if base_current > 0. {
            f64::max(0., base_current - self.props.motor_io)
        } else {
            f64::min(0., base_current + self.props.motor_io)
        };
        let nm_per_a = 8.3 / f64::max(self.props.motor_kv, 0.0001);
        current * nm_per_a
    }

    pub fn prop_wash_noise(&mut self, dt: f64) -> f64 {
        let motor_phase_compressed = (self.state.phase_slow * 3.0).floor() / 3.0;
        self.state.prop_wash_low_pass_filter.update(
            f64::max(0.0, 0.5 * perlin_noise(motor_phase_compressed) + 1.0),
            dt,
            120.,
        )
    }

    pub fn kv(&self) -> f64 {
        self.props.motor_kv
    }

    pub fn update_motor_torques(&mut self, p_torque: f64, m_torque: f64) {
        self.state.p_torque = p_torque;
        self.state.m_torque = m_torque;
    }

    // TODO: too many arguments, propbably also wrong, will need to refactor
    pub fn update_motor(
        &mut self,
        current: f64,
        speed: f64,
        thrust: f64,
        dt: f64,
        vbat: f64,
        ambient_temp: f64,
        p_torque: f64,
        m_torque: f64,
        rpm: f64,
    ) {
        self.update_motor_temp(current, speed, thrust, dt, vbat, ambient_temp);
        self.state.current = current;
        self.state.p_torque = p_torque;
        self.state.m_torque = m_torque;
        self.state.thrust = thrust;
        self.state.rpm = rpm;
    }

    pub fn dir(&self) -> f64 {
        self.props.motor_dir
    }

    pub fn pwm(&self) -> f64 {
        self.state.pwm
    }

    pub fn current(&self) -> f64 {
        self.state.current
    }

    pub fn motor_noise(&mut self, dt: f64) -> DMat3 {
        self.state.motor_noise(dt)
    }
}

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
