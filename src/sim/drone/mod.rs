pub mod arm;
#[cfg(feature = "gyro_noise")]
pub mod noise;

use super::rigid_body::RigidBody;
use crate::low_pass_filter::LowPassFilter;
use crate::rng_gen_range;
use crate::sample_curve::SampleCurve;
use arm::Arm;
use bevy::{
    math::{DMat3, DVec3, DVec4},
    prelude::Component,
};

#[derive(Debug, Clone, Default)]
pub struct Gyro {
    low_pass_filter: [LowPassFilter; 3],
    rotation: DVec4,
    acceleration: DVec3,
    gyro_angular_vel: DVec3,
    #[cfg(feature = "gyro_noise")]
    gyro_base_noise_amp: f64,
}

impl Gyro {
    pub fn set_angular_velocity(
        &mut self,
        rotation: DMat3,
        angular_velocity: DVec3,
        dt: f64,
        #[cfg(feature = "gyro_noise")] combined_noise: DVec3,
    ) {
        #[cfg(feature = "gyro_noise")]
        let mut angular_velocity = angular_velocity + combined_noise;
        #[cfg(not(feature = "gyro_noise"))]
        let mut angular_velocity = angular_velocity;

        let cutoff_freq = 300.;
        angular_velocity[0] =
            self.low_pass_filter()[0].update(angular_velocity[0], dt, cutoff_freq);
        angular_velocity[1] =
            self.low_pass_filter()[1].update(angular_velocity[0], dt, cutoff_freq);
        angular_velocity[2] =
            self.low_pass_filter()[2].update(angular_velocity[0], dt, cutoff_freq);
        self.gyro_angular_vel = xform_inv(rotation, angular_velocity);
    }

    pub fn set_acceleration(&mut self, rotation: DMat3, acceleration: DVec3) {
        let gravity_acceleration = DVec3::new(0., -9.81, 0.);
        self.acceleration = xform_inv(rotation, acceleration + gravity_acceleration)
    }

    pub fn low_pass_filter(&mut self) -> &mut [LowPassFilter; 3] {
        &mut self.low_pass_filter
    }

    pub fn angular_vel(&self) -> DVec3 {
        self.gyro_angular_vel
    }

    pub fn set_rotation(&mut self, rotation: DVec4) {
        self.rotation = rotation
    }

    pub fn rotation(&self) -> DVec4 {
        self.rotation
    }

    pub fn acceleration(&self) -> DVec3 {
        self.acceleration
    }
}

pub fn xform_inv(m: DMat3, v: DVec3) -> DVec3 {
    DVec3::new(
        (m.x_axis[0] * v[0]) + (m.y_axis[0] * v[1]) + (m.z_axis[0] * v[2]),
        (m.x_axis[1] * v[0]) + (m.y_axis[1] * v[1]) + (m.z_axis[1] * v[2]),
        (m.x_axis[2] * v[0]) + (m.y_axis[2] * v[1]) + (m.z_axis[2] * v[2]),
    )
}

// rotates a position according to the rotation matrix
pub fn xform(m: DMat3, v: DVec3) -> DVec3 {
    DVec3::new(
        DVec3::dot(m.x_axis, v),
        DVec3::dot(m.y_axis, v),
        DVec3::dot(m.z_axis, v),
    )
}

// This is the implementation that the thing used
pub fn mat3_to_quat(mat3: DMat3) -> DVec4 {
    let trace = mat3.x_axis[0] + mat3.y_axis[1] + mat3.z_axis[2];

    if trace > 0.0 {
        let s = f64::sqrt(trace + 1.);
        DVec4::new(
            mat3.z_axis[1] - mat3.y_axis[2] * 0.5 / s,
            mat3.x_axis[2] - mat3.z_axis[0] * 0.5 / s,
            mat3.y_axis[0] - mat3.x_axis[1] * 0.5 / s,
            s * 0.5,
        )
    } else {
        let i = if mat3.x_axis[0] < mat3.y_axis[1] {
            if mat3.y_axis[1] < mat3.z_axis[2] {
                2
            } else {
                1
            }
        } else if mat3.x_axis[0] < mat3.z_axis[2] {
            2
        } else {
            0
        };
        let j = (i + 1) % 3;
        let k = (i + 2) % 3;
        let s = f64::sqrt(mat3.col(i)[i] - mat3.col(j)[j] - mat3.col(k)[k] + 1.);
        let mut quat = DVec4::new(0., 0., 0., 0.);

        quat[i] = s * 0.5;

        quat[3] = (mat3.col(k)[j] - mat3.col(j)[k]) * 0.5 / s;
        quat[j] = (mat3.col(j)[i] - mat3.col(i)[j]) * 0.5 / s;
        quat[k] = (mat3.col(k)[i] - mat3.col(i)[k]) * 0.5 / s;
        quat
    }
}

#[cfg(feature = "gyro_noise")]
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
    pub motor_imbalance: [DVec3; 4],
}

#[derive(Clone, Component)]
pub struct Drone {
    #[cfg(feature = "gyro_noise")]
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

    pub fn vbat_sagged(&self) -> f64 {
        self.state.bat_voltage_sag
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
    pub fn set_motor_pwms(&mut self, pwms: DVec4) {
        for i in 0..4 {
            self.arms[i].set_pwm(pwms[i]);
        }
    }

    // Step first, we have to test this!
    pub fn calculate_physics(&mut self, motor_torque: f64, dt: f64) {
        let rotation = self.rigid_body.rotation;
        let sum_arm_forces = self.arms.iter().fold(DVec3::new(0., 0., 0.), |acc, arm| {
            acc + xform(rotation, DVec3::new(0., arm.thrust(), 0.))
        });
        let sum_prop_torques = self.arms.iter().fold(DVec3::new(0., 0., 0.), |acc, arm| {
            // calculates the force that is the downwards compared to the drone
            let force = xform(rotation, DVec3::new(0., arm.thrust(), 0.));
            // calculates the motor position relative to the body frame
            let rad = xform(rotation, arm.motor_pos());
            acc + DVec3::cross(rad, force)
        });
        self.rigid_body
            .integrate(motor_torque, sum_arm_forces, sum_prop_torques, dt);
    }

    pub fn calculate_motors(
        &mut self,
        // state_packet: &StatePacket,
        dt: f64,
        ambient_temp: f64,
    ) -> f64 {
        let mut res_prop_torque: f64 = 0.;
        let vbat = self.battery.vbat_sagged(); // is this what we want?
        for i in 0..4 {
            res_prop_torque += self.arms[i].calculate_arm_m_torque(
                dt,
                vbat,
                ambient_temp,
                self.rigid_body.rotation,
                self.rigid_body.linear_velocity,
            );
        }
        res_prop_torque
    }

    pub fn update_physics(&mut self, dt: f64, ambient_temp: f64) {
        let motor_torque = self.calculate_motors(dt, ambient_temp);
        self.calculate_physics(motor_torque, dt);

        let pwm_sum = self.arms.iter().fold(0.0, |pwm, arm| pwm + arm.pwm());
        let current_sum = self.arms.iter().fold(0., |acc, arm| acc + arm.current());
        self.battery.update(dt, pwm_sum, current_sum);
    }

    pub fn update_gyro(&mut self, dt: f64) {
        #[cfg(feature = "gyro_noise")]
        let combined_noise = self.calculate_combined_noise(dt);

        // sets attitude
        self.gyro
            .set_rotation(mat3_to_quat(self.rigid_body.rotation));

        self.gyro.set_angular_velocity(
            self.rigid_body.rotation,
            self.rigid_body.angular_velocity,
            dt,
            #[cfg(feature = "gyro_noise")]
            combined_noise,
        );

        // sets acceleration
        self.gyro
            .set_acceleration(self.rigid_body.rotation, self.rigid_body.acceleration);
    }
}
