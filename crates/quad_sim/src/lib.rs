pub mod arm;
mod constants;
pub mod controller;
mod low_pass_filter;
#[cfg(feature = "noise")]
pub mod noise;
pub mod rigid_body;
pub mod sample_curve;

use arm::Arm;
pub use arm::Motor;
// use bevy::math::{Matrix3<f64>, Vector3<f64>, DVec4};
use constants::MAX_EFFECT_SPEED;
use low_pass_filter::LowPassFilter;
use nalgebra::{Matrix3, Vector3, Vector4};
use rand::{Rng, SeedableRng};
use rand_xoshiro::Xoshiro256PlusPlus;
use rigid_body::RigidBody;
use sample_curve::SampleCurve;
use std::{cell::RefCell, ops::Range};

#[derive(Debug, Clone, Default)]
pub struct Gyro {
    low_pass_filter: [LowPassFilter; 3],
    rotation: Vector4<f64>,
    acceleration: Vector3<f64>,
    gyro_angular_vel: Vector3<f64>,
    #[cfg(feature = "noise")]
    gyro_base_noise_amp: f64,
}

impl Gyro {
    pub fn set_angular_velocity(
        &mut self,
        rotation: Matrix3<f64>,
        angular_velocity: Vector3<f64>,
        dt: f64,
        #[cfg(feature = "noise")] combined_noise: Vector3<f64>,
    ) {
        #[cfg(feature = "noise")]
        let mut angular_velocity = angular_velocity + combined_noise;
        #[cfg(not(feature = "noise"))]
        let mut angular_velocity = angular_velocity;

        let cutoff_freq = 300.;
        angular_velocity[0] =
            self.low_pass_filter()[0].update(angular_velocity[0], dt, cutoff_freq);
        angular_velocity[1] =
            self.low_pass_filter()[1].update(angular_velocity[0], dt, cutoff_freq);
        angular_velocity[2] =
            self.low_pass_filter()[2].update(angular_velocity[0], dt, cutoff_freq);
        self.gyro_angular_vel = rotation.transpose() * angular_velocity;
    }

    pub fn set_acceleration(&mut self, rotation: Matrix3<f64>, acceleration: Vector3<f64>) {
        let gravity_acceleration = Vector3::new(0., -9.81, 0.);
        self.acceleration = rotation.transpose() * (acceleration + gravity_acceleration)
    }

    pub fn low_pass_filter(&mut self) -> &mut [LowPassFilter; 3] {
        &mut self.low_pass_filter
    }

    pub fn angular_vel(&self) -> Vector3<f64> {
        self.gyro_angular_vel
    }

    pub fn set_rotation(&mut self, rotation: Vector4<f64>) {
        self.rotation = rotation
    }

    pub fn rotation(&self) -> Vector4<f64> {
        self.rotation
    }

    pub fn acceleration(&self) -> Vector3<f64> {
        self.acceleration
    }
}

// This is the implementation that the thing used
pub fn mat3_to_quat(mat3: Matrix3<f64>) -> Vector4<f64> {
    let trace = mat3.trace();

    if trace > 0.0 {
        let s = f64::sqrt(trace + 1.);
        Vector4::new(
            mat3.column(2)[1] - mat3.column(1)[2] * 0.5 / s,
            mat3.column(0)[2] - mat3.column(2)[0] * 0.5 / s,
            mat3.column(1)[0] - mat3.column(0)[1] * 0.5 / s,
            s * 0.5,
        )
    } else {
        let i = if mat3.column(0)[0] < mat3.column(1)[1] {
            if mat3.column(1)[1] < mat3.column(2)[2] {
                2
            } else {
                1
            }
        } else if mat3[0] < mat3[2 * 3 + 2] {
            2
        } else {
            0
        };
        let j = (i + 1) % 3;
        let k = (i + 2) % 3;
        let s = f64::sqrt(mat3[i * 3 + i] - mat3[j * 3 + j] - mat3[k * 3 + k] + 1.);
        let mut quat = Vector4::new(0., 0., 0., 0.);

        quat[i] = s * 0.5;

        quat[3] = (mat3.column(k)[j] - mat3.column(j)[k]) * 0.5 / s;
        quat[j] = (mat3.column(j)[i] - mat3.column(i)[j]) * 0.5 / s;
        quat[k] = (mat3.column(k)[i] - mat3.column(i)[k]) * 0.5 / s;
        quat
    }
}

#[cfg(feature = "noise")]
#[derive(Clone, Default)]
pub struct FrameCharachteristics {
    pub prop_harmonic_1_amp: f64,
    pub prop_harmonic_2_amp: f64,
    pub frame_harmonic_phase_1: f64,
    pub frame_harmonic_phase_2: f64,
    pub frame_harmonic_1_amp: f64,
    pub frame_harmonic_1_freq: f64,
    pub frame_harmonic_2_amp: f64,
    pub frame_harmonic_2_freq: f64,
    pub motor_imbalance: [Vector3<f64>; 4],
}

#[derive(Clone)]
pub struct Drone {
    #[cfg(feature = "noise")]
    pub frame_charachteristics: FrameCharachteristics,
    pub battery: Battery,
    pub arms: [Arm; 4],
    pub rigid_body: RigidBody,
    pub gyro: Gyro,
}

// TODO: what is the difference between full_capacity and quad_bat_capacity_charged?
#[derive(Debug, Clone)]
pub struct BatteryProps {
    pub full_capacity: f64, // mAH I guess
    pub bat_voltage_curve: SampleCurve,
    pub quad_bat_cell_count: f64,
    pub quad_bat_capacity_charged: f64,
    pub max_voltage_sag: f64,
}

#[derive(Debug, Clone, Default)]
pub struct BatteryState {
    capacity: f64,
    bat_voltage: f64,
    bat_voltage_sag: f64,
    amperage: f64,
    m_ah_drawn: f64,
}

#[derive(Debug, Clone)]
pub struct Battery {
    pub props: BatteryProps,
    pub state: BatteryState,
}

impl Battery {
    pub fn update(&mut self, dt: f64, pwm_sum: f64, current_sum: f64) {
        let bat_capacity_full = f64::max(self.props.full_capacity, 1.0);
        let bat_charge = self.state.capacity / bat_capacity_full;
        self.state.bat_voltage = f64::max(
            self.props.bat_voltage_curve.sample(1. - bat_charge) * self.props.quad_bat_cell_count,
            0.1,
        );
        let power_factor_squared = f64::max(0., pwm_sum / 4.).powi(2);
        let charge_factor_inv =
            1.0 - (self.state.capacity / f64::max(self.props.quad_bat_capacity_charged, 1.));

        let v_sag = self.props.max_voltage_sag * power_factor_squared
            + (self.props.max_voltage_sag
                * charge_factor_inv
                * charge_factor_inv
                * power_factor_squared);
        self.state.bat_voltage_sag = f64::clamp(
            self.state.bat_voltage_sag - v_sag - rng_gen_range(-0.01..0.01),
            0.0,
            100.,
        );
        let m_a_min = f64::min(0.2, rng_gen_range(-0.125..0.375))
            / f64::max(self.state.bat_voltage_sag, 0.01);
        let currentm_as = f64::max(current_sum / 3.6, m_a_min);
        self.state.amperage = currentm_as * 3.6;
        self.state.capacity -= currentm_as * dt;
        self.state.m_ah_drawn = self.props.quad_bat_capacity_charged - self.state.capacity;
    }

    pub fn cell_count(&self) -> f64 {
        self.props.quad_bat_cell_count
    }

    pub fn get_bat_voltage_sag(&self) -> f64 {
        self.state.bat_voltage_sag
    }

    pub fn get_bat_voltage(&self) -> f64 {
        self.state.bat_voltage
    }

    pub fn amperage(&self) -> f64 {
        self.state.amperage
    }

    pub fn m_ah_drawn(&self) -> f64 {
        self.state.m_ah_drawn
    }
}

impl Drone {
    pub fn set_motor_pwms(&mut self, pwms: Vector4<f64>) {
        for i in 0..4 {
            self.arms[i].set_pwm(pwms[i]);
        }
    }

    // Step first, we have to test this!
    fn calculate_physics(&mut self, motor_torque: f64, dt: f64) {
        let rotation = self.rigid_body.rotation;
        let individual_arm_foces = self
            .arms
            .iter()
            .map(|arm| rotation * Vector3::new(0., arm.thrust(), 0.))
            .collect::<Vec<_>>();
        let sum_arm_forces = individual_arm_foces.iter().sum();
        let sum_prop_torques = individual_arm_foces
            .iter()
            .enumerate()
            .map(|(i, force)| {
                let rad = rotation * self.arms[i].motor_pos();
                Vector3::cross(&rad, &force)
            })
            .sum();
        self.rigid_body
            .integrate(motor_torque, &sum_arm_forces, &sum_prop_torques, dt);
    }

    fn calculate_motors(&mut self, dt: f64, ambient_temp: f64) -> f64 {
        let vbat = self.battery.state.bat_voltage_sag; // is this what we want?
        let speed = self.rigid_body.linear_velocity.norm();
        let speed_factor = f64::min(speed / MAX_EFFECT_SPEED, 1.);
        let vel_up = Vector3::dot(
            &self.rigid_body.linear_velocity,
            &self.rigid_body.rotation.column(0),
        );
        let m_torques = self.arms.iter_mut().map(|arm| {
            arm.calculate_arm_m_torque(
                dt,
                vbat,
                ambient_temp,
                &self.rigid_body.rotation,
                &self.rigid_body.linear_velocity,
                speed,
                speed_factor,
                vel_up,
            )
        });
        m_torques.sum()
    }

    pub fn update_physics(&mut self, dt: f64, ambient_temp: f64) {
        let motor_torque = self.calculate_motors(dt, ambient_temp);
        self.calculate_physics(motor_torque, dt);

        let pwm_sum = self.arms.iter().map(|arm| arm.pwm()).sum();
        let current_sum = self.arms.iter().map(|arm| arm.current()).sum();
        self.battery.update(dt, pwm_sum, current_sum);
    }

    pub fn update_gyro(&mut self, dt: f64) {
        #[cfg(feature = "noise")]
        let combined_noise = self.calculate_combined_noise(dt);

        // sets attitude
        self.gyro
            .set_rotation(mat3_to_quat(self.rigid_body.rotation));

        self.gyro.set_angular_velocity(
            self.rigid_body.rotation,
            self.rigid_body.angular_velocity,
            dt,
            #[cfg(feature = "noise")]
            combined_noise,
        );

        // sets acceleration
        self.gyro
            .set_acceleration(self.rigid_body.rotation, self.rigid_body.acceleration);
    }
}

thread_local! {
    static RNG: RefCell<Xoshiro256PlusPlus> = RefCell::new(Xoshiro256PlusPlus::from_entropy());
}

pub fn rng_gen_range(range: Range<f64>) -> f64 {
    RNG.with(|rng| rng.borrow_mut().gen_range(range))
}
