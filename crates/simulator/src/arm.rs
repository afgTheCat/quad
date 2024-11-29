use crate::low_pass_filter::LowPassFilter;
use core::f64;
use nalgebra::{Rotation3, Vector3};
use std::f64::consts::PI;

#[derive(Debug, Clone)]
pub struct Propeller {
    pub prop_max_rpm: f64,
    pub prop_a_factor: f64,
    pub prop_torque_factor: f64,
    pub prop_inertia: f64,
    pub prop_thrust_factor: Vector3<f64>,
}

impl Default for Propeller {
    fn default() -> Self {
        Self {
            prop_max_rpm: 1.,
            prop_a_factor: 0.,
            prop_torque_factor: 0.,
            prop_inertia: 0.1,
            prop_thrust_factor: Vector3::new(100., 100., 100.),
        }
    }
}

impl Propeller {
    // Calculates the
    pub fn prop_thrust(&self, vel_up: f64, rpm: f64) -> f64 {
        let prop_f = self.prop_thrust_factor[0] * vel_up * vel_up
            + self.prop_thrust_factor[1] * vel_up
            + self.prop_thrust_factor[2];
        let max_rpm = self.prop_max_rpm;
        let prop_a = self.prop_a_factor;
        let b = (prop_f - prop_a * max_rpm * max_rpm) / max_rpm;
        let result = b * rpm + prop_a * rpm * rpm;
        f64::max(result, 0.0)
    }
}

// TODO: defualt here is has to be manually implemeted
#[derive(Debug, Clone, Default)]
pub struct MotorProps {
    pub position: Vector3<f64>, // position relative to the main body
    pub motor_kv: f64,          // kv
    pub motor_r: f64,           // resistence
    pub motor_io: f64,          // idle current
    pub motor_dir: f64,
    #[cfg(feature = "temp")]
    pub motor_rth: f64, // thermal resistance (deg C per Watt)
    #[cfg(feature = "temp")]
    pub motor_cth: f64, // thermal heat capacity (joules per deg C)
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
    #[cfg(feature = "temp")]
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
        let cooling = (1. - exp((-speed * 0.2) as f32) as f64) * 100.
            + (f64::min(MAX_SPEED_PROP_COOLING, speed) / MAX_SPEED_PROP_COOLING) * thrust * 4.;

        self.state.temp += ((f64::max(0.0, power_draw - cooling))
            - (self.state.temp - ambient_temp) / self.props.motor_rth)
            / self.props.motor_cth
            * dt;
    }

    fn rpm(&self) -> f64 {
        self.state.rpm
    }

    // Calculates the effective voltage that the motor recieves
    // It is done by running the battery voltage through a low pass filter
    pub fn volts(&mut self, dt: f64, vbat: f64) -> f64 {
        self.state
            .pwm_low_pass_filter
            .update(self.state.pwm, dt, 120.0)
            * vbat
    }

    pub fn position(&self) -> Vector3<f64> {
        self.props.position
    }

    // https://en.wikipedia.org/wiki/Motor_constants#Motor_torque_constant
    pub fn motor_torque(&self, volts: f64) -> f64 {
        let kv = self.props.motor_kv;
        let back_emf_v = self.state.rpm / kv;
        let base_current = (volts - back_emf_v) / self.props.motor_r;
        let armature_current = if base_current > 0. {
            f64::max(0., base_current - self.props.motor_io)
        } else {
            f64::min(0., base_current + self.props.motor_io)
        };
        let torque_constant = 8.3 / kv; // why do we need to calculate this?
        armature_current * torque_constant
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
        rotation: Rotation3<f64>,
        linear_velocity_dir: Option<Vector3<f64>>,
        speed_factor: f64,
        vel_up: f64,
        #[cfg(feature = "noise")] dt: f64,
    ) -> f64 {
        let mut reverse_thrust = if let Some(linear_velocity_dir) = linear_velocity_dir {
            -Vector3::dot(
                &linear_velocity_dir,
                &(rotation.matrix().column(0) * self.motor.state.thrust).normalize(),
            )
        } else {
            0.
        };
        reverse_thrust = f64::max(0.0, reverse_thrust - 0.5) * 2.;
        reverse_thrust = reverse_thrust * reverse_thrust;

        #[cfg(feature = "noise")]
        let prop_wash_noise = self.motor.prop_wash_noise(dt);

        // NOTE: no noise is applied
        #[cfg(not(feature = "noise"))]
        let prop_wash_noise = 0.;

        let prop_wash_effect = 1.0 - (speed_factor * prop_wash_noise * reverse_thrust * 0.95);
        self.propeller.prop_thrust(vel_up, rpm) * prop_wash_effect
    }

    pub fn calculate_arm_m_torque(
        &mut self,
        dt: f64,
        vbat: f64,
        rotation: Rotation3<f64>,
        linear_velocity_dir: Option<Vector3<f64>>,
        speed_factor: f64,
        vel_up: f64,
        #[cfg(feature = "temp")] speed: f64,
        #[cfg(feature = "temp")] ambient_temp: f64,
    ) {
        // this value is internal to the motor
        let armature_volts = self.motor.volts(dt, vbat);
        // this value is internal to the motor
        let m_torque = self.motor.motor_torque(armature_volts);
        let p_torque = self.propeller.prop_thrust(vel_up, self.motor.state.rpm)
            * self.propeller.prop_torque_factor;
        // net torque that is exerted to the propeller
        let net_torque = m_torque - p_torque;
        let domega = net_torque / self.propeller.prop_inertia;
        // change in rpm
        let drpm = (domega * dt) * 60.0 / (2.0 * PI);
        let maxdrpm = f64::abs(armature_volts * self.motor.props.motor_kv - self.motor.state.rpm);
        let rpm = self.motor.state.rpm + f64::clamp(drpm, -maxdrpm, maxdrpm);
        let current = m_torque * self.motor.props.motor_kv / 8.3;
        // this uses rpm
        let thrust = self.motor_thrust(
            rpm,
            rotation,
            linear_velocity_dir,
            speed_factor,
            vel_up,
            #[cfg(feature = "noise")]
            dt,
        );
        #[cfg(feature = "temp")]
        self.motor
            .update_motor_temp(current, speed, thrust, dt, vbat, ambient_temp);
        self.motor.state.current = current;
        self.motor.state.p_torque = p_torque;
        self.motor.state.m_torque = m_torque;
        self.motor.state.thrust = thrust;
        self.motor.state.rpm = rpm;
    }

    pub fn pwm(&self) -> f64 {
        self.motor.pwm()
    }

    pub fn current(&self) -> f64 {
        self.motor.current()
    }

    pub fn thrust(&self) -> f64 {
        self.motor.state.thrust
    }

    pub fn motor_pos(&self) -> Vector3<f64> {
        self.motor.position()
    }

    pub fn rpm(&self) -> f64 {
        self.motor.rpm()
    }

    pub fn set_pwm(&mut self, pwm: f64) {
        self.motor.state.pwm = pwm;
    }
}
