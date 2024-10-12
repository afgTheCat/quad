use core::f64;

use fast_math::exp;
// use libm::exp;

use crate::{
    constants::{MAX_SPEED_PROP_COOLING, M_PI},
    low_pass_filter::LowPassFilter,
};
use nalgebra::{Matrix3, Vector3};

#[derive(Debug, Clone)]
pub struct Propeller {
    prop_max_rpm: f64,
    prop_a_factor: f64,
    prop_torque_factor: f64,
    prop_inertia: f64,
    prop_thrust_factor: Vector3<f64>,
}

impl Default for Propeller {
    fn default() -> Self {
        Self {
            prop_max_rpm: 1.,
            prop_a_factor: 0.,
            prop_torque_factor: 0.,
            prop_inertia: 0.1,
            prop_thrust_factor: Vector3::zeros(),
        }
    }
}

impl Propeller {
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
    pub position: Vector3<f64>, // position relative to the main body
    pub motor_kv: f64,          // kv
    pub motor_r: f64,           // resistence
    pub motor_io: f64,          // idle current
    pub motor_rth: f64,         // thermal resistance (deg C per Watt)
    pub motor_cth: f64,         // thermal heat capacity (joules per deg C)
    pub motor_dir: f64,
    // pub motor_max_t: f64,
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
    #[cfg(feature = "noise")]
    pub prop_wash_low_pass_filter: LowPassFilter, // low pass filtered prop wash
    #[cfg(feature = "noise")]
    pub phase: f64, // sinusoidal phase of the motor rotation used for noise simulation
    #[cfg(feature = "noise")]
    pub phase_slow: f64, // phase freq * 0.01f
    #[cfg(feature = "noise")]
    pub phase_harmonic_1: f64, // phase freq * 2
    #[cfg(feature = "noise")]
    pub phase_harmonic_2: f64, // phase freq * 3
                                            //
                                            // pub burned_out: bool, // is the motor destroyed by over temp
}

#[derive(Debug, Clone, Default)]
pub struct Motor {
    pub state: MotorState,
    pub props: MotorProps,
}

impl Motor {
    pub fn update_motor_temp(
        &mut self,
        current: f64,
        speed: f64,
        thrust: f64,
        dt: f64,
        vbat: f64,
        ambient_temp: f64,
    ) {
        let power_draw = f64::abs(current) * vbat;
        let cooling = (1. - exp((-speed * 0.2) as f32) as f64) * 100.
            + (f64::min(MAX_SPEED_PROP_COOLING, speed) / MAX_SPEED_PROP_COOLING) * thrust * 4.;

        self.state.temp += ((f64::max(0.0, power_draw - cooling))
            - (self.state.temp - ambient_temp) / self.props.motor_rth)
            / self.props.motor_cth
            * dt;
    }
}

impl Motor {
    pub fn rpm(&self) -> f64 {
        self.state.rpm
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

    pub fn position(&self) -> Vector3<f64> {
        self.props.position
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

    pub fn kv(&self) -> f64 {
        self.props.motor_kv
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
}

#[derive(Debug, Clone, Default)]
pub struct Arm {
    pub propeller: Propeller,
    pub motor: Motor,
}

impl Arm {
    fn motor_thrust(
        &mut self,
        rpm: f64,
        rotation: Matrix3<f64>,
        linear_velocity: Vector3<f64>,
        speed_factor: f64,
        #[cfg(feature = "noise")] dt: f64,
    ) -> f64 {
        let up = rotation.column(0);
        let vel_up = Vector3::dot(&linear_velocity, &up);
        // TODO: check this
        let thrust_corrected: Vector3<f64> = -(up * self.motor.thrust()).normalize();
        let mut reverse_thrust = f64::max(
            0.,
            Vector3::dot(&linear_velocity.normalize(), &thrust_corrected),
        );
        reverse_thrust = f64::max(0.0, reverse_thrust - 0.5) * 2.;
        reverse_thrust = reverse_thrust * reverse_thrust;

        #[cfg(feature = "noise")]
        let prop_wash_noise = self.motor.prop_wash_noise(dt);

        // NOTE: no noise is applied
        #[cfg(not(feature = "noise"))]
        let prop_wash_noise = 1.;

        let prop_wash_effect = 1.0 - (speed_factor * prop_wash_noise * reverse_thrust * 0.95);
        self.propeller.prop_thrust(vel_up, rpm) * prop_wash_effect
    }

    pub fn calculate_arm_m_torque(
        &mut self,
        dt: f64,
        vbat: f64,
        ambient_temp: f64,
        rotation: Matrix3<f64>,
        linear_velocity: Vector3<f64>,
        speed: f64,
        speed_factor: f64,
        vel_up: f64,
    ) -> f64 {
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
        let thrust = self.motor_thrust(
            rpm,
            rotation,
            linear_velocity,
            speed_factor,
            #[cfg(feature = "noise")]
            dt,
        );

        self.motor
            .update_motor_temp(current, speed, thrust, dt, vbat, ambient_temp);
        self.motor.state.current = current;
        self.motor.state.p_torque = p_torque;
        self.motor.state.m_torque = m_torque;
        self.motor.state.thrust = thrust;
        self.motor.state.rpm = rpm;
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

    pub fn motor_pos(&self) -> Vector3<f64> {
        self.motor.position()
    }

    pub fn set_pwm(&mut self, pwm: f64) {
        self.motor.state.pwm = pwm;
    }
}
