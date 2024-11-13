pub mod arm;
mod constants;
mod low_pass_filter;
#[cfg(feature = "noise")]
pub mod noise;
pub mod rigid_body;
pub mod sample_curve;

use arm::Arm;
pub use arm::Motor;
use constants::MAX_EFFECT_SPEED;
use core::f64;
use flight_controller::{BatteryUpdate, GyroUpdate, MotorInput};
use low_pass_filter::LowPassFilter;
use nalgebra::{Rotation3, UnitQuaternion, Vector3, Vector4};
use rand::{Rng, SeedableRng};
use rand_xoshiro::Xoshiro256PlusPlus;
use rigid_body::RigidBody;
use sample_curve::SampleCurve;
use std::{cell::RefCell, ops::Range};

#[derive(Debug, Clone, Default)]
pub struct Gyro {
    low_pass_filter: [LowPassFilter; 3],
    previous_rotation: UnitQuaternion<f64>,
    rotation: UnitQuaternion<f64>, // so far it was w, i, j, k
    acceleration: Vector3<f64>,
    gyro_angular_vel: Vector3<f64>,
    #[cfg(feature = "noise")]
    gyro_base_noise_amp: f64,
}

impl Gyro {
    pub fn set_angular_velocity(
        &mut self,
        rotation: Rotation3<f64>,
        angular_velocity: Vector3<f64>,
        dt: f64,
        #[cfg(feature = "noise")] combined_noise: Vector3<f64>,
    ) {
        #[cfg(feature = "noise")]
        let mut angular_velocity = angular_velocity + combined_noise;
        #[cfg(not(feature = "noise"))]
        let mut gyro_angular_velocity = angular_velocity;

        let cutoff_freq = 300.;
        gyro_angular_velocity[0] =
            self.low_pass_filter()[0].update(angular_velocity[0], dt, cutoff_freq);
        gyro_angular_velocity[1] =
            self.low_pass_filter()[1].update(angular_velocity[0], dt, cutoff_freq);
        gyro_angular_velocity[2] =
            self.low_pass_filter()[2].update(angular_velocity[0], dt, cutoff_freq);
        self.gyro_angular_vel = rotation.transpose() * angular_velocity;
    }

    pub fn set_acceleration(&mut self, rotation: Rotation3<f64>, acceleration: Vector3<f64>) {
        self.acceleration = rotation.transpose() * acceleration
    }

    pub fn low_pass_filter(&mut self) -> &mut [LowPassFilter; 3] {
        &mut self.low_pass_filter
    }

    pub fn angular_vel(&self) -> Vector3<f64> {
        self.gyro_angular_vel
    }

    pub fn set_rotation(&mut self, rotation: UnitQuaternion<f64>) {
        self.rotation = rotation
    }

    pub fn acceleration(&self) -> Vector3<f64> {
        self.acceleration
    }

    pub fn update(
        &mut self,
        dt: f64,
        rotation: Rotation3<f64>,
        angular_velocity: Vector3<f64>,
        acceleration: Vector3<f64>,
        #[cfg(feature = "noise")] combined_noise: Vector3<f64>,
    ) -> GyroUpdate {
        let new_rotation = UnitQuaternion::from(rotation);
        self.previous_rotation = new_rotation;
        // BF expects the order to be w, x, y, z
        self.set_rotation(new_rotation);
        self.set_angular_velocity(
            rotation,
            angular_velocity,
            dt,
            #[cfg(feature = "noise")]
            combined_noise,
        );
        self.set_acceleration(rotation, acceleration);

        GyroUpdate {
            rotation: [
                self.rotation.w,
                self.rotation.i,
                self.rotation.j,
                self.rotation.k,
            ],
            linear_acc: self.acceleration.data.0[0],
            angular_velocity: self.gyro_angular_vel.data.0[0],
        }
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
    //battery capacacity rating
    pub quad_bat_capacity: f64, // mAH I guess
    pub bat_voltage_curve: SampleCurve,
    pub quad_bat_cell_count: u8,
    //charged up capacity, can be lower or higher than quadBatCapacity
    pub quad_bat_capacity_charged: f64,
    pub max_voltage_sag: f64,
}

#[derive(Debug, Clone, Default)]
pub struct BatteryState {
    pub capacity: f64,
    pub bat_voltage: f64,
    pub bat_voltage_sag: f64,
    pub amperage: f64,
    pub m_ah_drawn: f64,
}

#[derive(Debug, Clone)]
pub struct Battery {
    pub props: BatteryProps,
    pub state: BatteryState,
}

impl Battery {
    pub fn update(&mut self, dt: f64, pwm_sum: f64, current_sum: f64) {
        let bat_charge = self.state.capacity / self.props.quad_bat_capacity;
        self.state.bat_voltage = f64::max(
            self.props.bat_voltage_curve.sample(1. - bat_charge)
                * self.props.quad_bat_cell_count as f64,
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
            self.state.bat_voltage - v_sag - rng_gen_range(-0.01..0.01),
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

    // pub fn cell_count(&self) -> u8 {
    //     self.props.quad_bat_cell_count
    // }

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

    pub fn battery_update(&self) -> BatteryUpdate {
        BatteryUpdate {
            cell_count: self.props.quad_bat_cell_count,
            bat_voltage_sag: self.state.bat_voltage_sag,
            bat_voltage: self.state.bat_voltage,
            amperage: self.state.amperage,
            m_ah_drawn: self.state.m_ah_drawn,
        }
    }
}

impl Drone {
    pub fn set_motor_pwms(&mut self, pwms: MotorInput) {
        for i in 0..4 {
            self.arms[i].set_pwm(pwms[i]);
        }
    }

    pub fn thrusts(&self) -> Vector4<f64> {
        Vector4::from_row_slice(
            &self
                .arms
                .iter()
                .map(|arm| arm.thrust())
                .collect::<Vec<f64>>(),
        )
    }

    pub fn rpms(&self) -> Vector4<f64> {
        Vector4::from_row_slice(&self.arms.iter().map(|arm| arm.rpm()).collect::<Vec<f64>>())
    }

    // Step first, we have to test this!
    fn calculate_physics(&mut self, dt: f64) {
        let motor_torque = self
            .arms
            .iter()
            .map(|arm| arm.motor.state.m_torque * arm.motor.props.motor_dir)
            .sum();
        let rotation = self.rigid_body.rotation;
        let individual_arm_forces = self
            .arms
            .iter()
            .map(|arm| rotation * Vector3::new(0., arm.thrust(), 0.))
            .collect::<Vec<_>>();
        let sum_arm_forces = individual_arm_forces.iter().sum();
        let sum_prop_torques = individual_arm_forces
            .iter()
            .enumerate()
            .map(|(i, force)| {
                let rad = rotation * self.arms[i].motor_pos();
                Vector3::cross(&rad, &force)
            })
            .sum();
        self.rigid_body
            .integrate(motor_torque, sum_arm_forces, sum_prop_torques, dt);
    }

    fn calculate_motors(&mut self, dt: f64, ambient_temp: f64) {
        let vbat = self.battery.state.bat_voltage_sag; // is this what we want?
        let speed = self.rigid_body.linear_velocity.norm();
        let speed_factor = f64::min(speed / MAX_EFFECT_SPEED, 1.);
        let vel_up = f64::max(
            0.,
            Vector3::dot(
                &self.rigid_body.linear_velocity,
                &self.rigid_body.rotation.matrix().column(0),
            ),
        );
        for i in 0..4 {
            self.arms[i].calculate_arm_m_torque(
                dt,
                vbat,
                self.rigid_body.rotation,
                self.rigid_body.linear_velocity_dir,
                speed_factor,
                vel_up,
                #[cfg(feature = "temp")]
                speed,
                #[cfg(feature = "temp")]
                ambient_temp,
            )
        }
    }

    pub fn update_physics(&mut self, dt: f64, ambient_temp: f64) {
        self.calculate_motors(dt, ambient_temp);
        self.calculate_physics(dt);

        let pwm_sum = self.arms.iter().map(|arm| arm.pwm()).sum();
        let current_sum = self.arms.iter().map(|arm| arm.current()).sum();
        self.battery.update(dt, pwm_sum, current_sum);
    }

    pub fn motor_pwms(&self) -> Vector4<f64> {
        Vector4::from_row_slice(&self.arms.iter().map(|arm| arm.pwm()).collect::<Vec<f64>>())
    }

    pub fn update_gyro(&mut self, dt: f64) -> GyroUpdate {
        #[cfg(feature = "noise")]
        let combined_noise = self.calculate_combined_noise(dt);

        self.gyro.update(
            dt,
            self.rigid_body.rotation,
            self.rigid_body.angular_velocity,
            self.rigid_body.acceleration,
            #[cfg(feature = "noise")]
            combined_noise,
        )
    }
}

thread_local! {
    static RNG: RefCell<Xoshiro256PlusPlus> = RefCell::new(Xoshiro256PlusPlus::from_entropy());
}

pub fn rng_gen_range(range: Range<f64>) -> f64 {
    RNG.with(|rng| rng.borrow_mut().gen_range(range))
}
