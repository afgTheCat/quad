use super::arm::Arm;
use super::battery::Battery;
use super::rigid_body::RigidBody;
use crate::rng_gen_range;
use crate::{constants::M_PI, low_pass_filter::LowPassFilter};
use bevy::{
    math::{DMat3, DVec3, DVec4},
    prelude::Component,
};

#[derive(Debug, Clone, Default)]
pub struct Gyro {
    gyro_noise: DVec3,
    low_pass_filter: [LowPassFilter; 3],
    rotation: DVec4,
    acceleration: DVec3,
    gyro_angular_vel: DVec3,
    gyro_base_noise_amp: f64,
}

impl Gyro {
    pub fn update_gyro_noise(&mut self) {
        let white_noise_x = rng_gen_range(-1.0..1.) * self.gyro_base_noise_amp;
        let white_noise_y = rng_gen_range(-1.0..1.) * self.gyro_base_noise_amp;
        let white_noise_z = rng_gen_range(-1.0..1.) * self.gyro_base_noise_amp;
        self.gyro_noise[0] = white_noise_x;
        self.gyro_noise[1] = white_noise_y;
        self.gyro_noise[1] = white_noise_z;
    }

    pub fn set_angular_velocity(
        &mut self,
        rotation: DMat3,
        angular_velocity: DVec3,
        combined_noise: DVec3,
        dt: f64,
    ) {
        let mut angular_velocity = angular_velocity + combined_noise;
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

    pub fn gyro_noise(&self) -> DVec3 {
        self.gyro_noise
    }

    pub fn low_pass_filter(&mut self) -> &mut [LowPassFilter; 3] {
        &mut self.low_pass_filter
    }

    pub fn set_gyro_angular_vel(&mut self, gyro: DVec3) {
        self.gyro_angular_vel = gyro
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

pub fn shifted_phase(dt: f64, hz: f64, phase_start: f64) -> f64 {
    let two_pi = 2. * M_PI;
    let phase_shift = two_pi * dt * hz;
    let phase_updated = phase_start + phase_shift;
    if f64::abs(phase_updated) > two_pi {
        phase_updated - (two_pi * f64::floor(phase_updated / two_pi))
    } else {
        phase_updated
    }
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

pub fn rpm_to_hz(rpm: f64) -> f64 {
    rpm / 60.
}

#[derive(Clone, Default, Component)]
pub struct Drone {
    pub prop_harmonic_1_amp: f64,
    pub prop_harmonic_2_amp: f64,
    pub frame_harmonic_phase_1: f64,
    pub frame_harmonic_phase_2: f64,
    pub frame_harmonic_1_amp: f64, // TODO: can this change?
    pub frame_harmonic_1_freq: f64,
    pub frame_harmonic_2_amp: f64,
    pub frame_harmonic_2_freq: f64,
    pub motor_imbalance: [DVec3; 4],
    pub combined_noise: DVec3,
    pub motor_noise: DVec3,
    pub battery: Battery,
    pub arms: [Arm; 4],
    pub rigid_body: RigidBody,
    pub gyro: Gyro,
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

    fn motor_kv(&self) -> DVec4 {
        DVec4::new(
            self.arms[0].motor_kv(),
            self.arms[1].motor_kv(),
            self.arms[2].motor_kv(),
            self.arms[3].motor_kv(),
        )
    }

    fn motor_rpm(&self) -> DVec4 {
        DVec4::new(
            self.arms[0].motor_rpm(),
            self.arms[1].motor_rpm(),
            self.arms[2].motor_rpm(),
            self.arms[3].motor_rpm(),
        )
    }

    fn motor_noise(&mut self, dt: f64) -> [DMat3; 4] {
        [
            self.arms[0].motor_noise(dt),
            self.arms[1].motor_noise(dt),
            self.arms[2].motor_noise(dt),
            self.arms[3].motor_noise(dt),
        ]
    }

    fn motor_temp(&mut self) -> DVec4 {
        DVec4::new(
            self.arms[0].motor_temp(),
            self.arms[1].motor_temp(),
            self.arms[2].motor_temp(),
            self.arms[3].motor_temp(),
        )
    }

    fn update_motor_noise(&mut self, dt: f64) {
        let max_v = self.battery.cell_count() * 4.2;
        let max_rpm = (self.motor_kv() * max_v).max(DVec4::splat(0.1));
        let motor_rpm = self.motor_rpm();
        let rpm_factor = motor_rpm.max(DVec4::ZERO) / max_rpm;
        let rpm_factor_squared = rpm_factor.powf(2.);
        let dmg_factor = DVec4::splat(0.05);
        let rpm_dmg_factor = dmg_factor * rpm_factor_squared;
        let m_noise = self.motor_noise(dt);

        let mut noise = DVec3::new(0., 0., 0.);
        for i in 0..3 {
            noise[0] += m_noise[i].x_axis[0] * self.motor_imbalance[i][0] * rpm_dmg_factor[i]
                + m_noise[i].x_axis[1]
                    * self.motor_imbalance[i][0]
                    * rpm_dmg_factor[i]
                    * self.prop_harmonic_1_amp
                + m_noise[i].x_axis[2]
                    * self.motor_imbalance[i][0]
                    * rpm_dmg_factor[i]
                    * self.prop_harmonic_2_amp;
            noise[1] += m_noise[i].y_axis[0] * self.motor_imbalance[i][1] * rpm_dmg_factor[i]
                + m_noise[i].y_axis[1]
                    * self.motor_imbalance[i][1]
                    * rpm_dmg_factor[i]
                    * self.prop_harmonic_1_amp
                + m_noise[i].y_axis[2]
                    * self.motor_imbalance[i][1]
                    * rpm_dmg_factor[i]
                    * self.prop_harmonic_2_amp;

            noise[2] += (m_noise[i].z_axis[0] * self.motor_imbalance[i][2] * rpm_dmg_factor[i]
                + m_noise[i].z_axis[1]
                    * self.motor_imbalance[i][2]
                    * rpm_dmg_factor[i]
                    * self.prop_harmonic_1_amp
                + m_noise[i].z_axis[2]
                    * self.motor_imbalance[i][2]
                    * rpm_dmg_factor[i]
                    * self.prop_harmonic_2_amp)
                * 0.5;
        }
        self.frame_harmonic_phase_1 = shifted_phase(
            dt,
            self.frame_harmonic_1_freq + rng_gen_range(-70.0..70.),
            self.frame_harmonic_phase_1,
        );
        self.frame_harmonic_phase_2 = shifted_phase(
            dt,
            self.frame_harmonic_2_freq + rng_gen_range(-60.0..60.),
            self.frame_harmonic_phase_2,
        );
        let rpm_factor_h_dec = DVec4::min(
            DVec4::max(motor_rpm, DVec4::ZERO / (max_rpm * 0.15)),
            DVec4::ONE,
        );
        let rpm_factor_h = rpm_factor_h_dec.element_sum() * 0.25;

        let rpm_factor_h_1_inc = DVec4::min(
            DVec4::max(motor_rpm, DVec4::ZERO / (max_rpm * 0.43)),
            DVec4::splat(1.),
        );
        let rpm_factor_h_1_inv = (rpm_factor_h_1_inc + -1.).element_sum() * 0.25;

        let rpm_factor_h_2_inc = DVec4::min(
            DVec4::max(motor_rpm, DVec4::ZERO / (max_rpm * 0.3)),
            DVec4::ONE,
        );
        let rpm_factor_h_2_inv = (rpm_factor_h_2_inc + -1.).element_sum() * 0.25;

        noise[0] += (self.motor_imbalance[0][0] * dmg_factor[0]
            + self.motor_imbalance[1][0] * dmg_factor[1]
            + self.motor_imbalance[2][0] * dmg_factor[2]
            + self.motor_imbalance[3][0] * dmg_factor[3])
            * 0.25
            * self.frame_harmonic_1_amp
            * self.frame_harmonic_phase_1.sin()
            * rpm_factor_h_1_inv
            * rpm_factor_h;
        noise[0] += (self.motor_imbalance[0][0] * dmg_factor[0]
            + self.motor_imbalance[1][0] * dmg_factor[1]
            + self.motor_imbalance[2][0] * dmg_factor[2]
            + self.motor_imbalance[3][0] * dmg_factor[3])
            * 0.25
            * self.frame_harmonic_2_amp
            * self.frame_harmonic_phase_2.sin()
            * rpm_factor_h_2_inv
            * rpm_factor_h;

        noise[1] += (self.motor_imbalance[0][1] * dmg_factor[0]
            + self.motor_imbalance[1][1] * dmg_factor[1]
            + self.motor_imbalance[2][1] * dmg_factor[2]
            + self.motor_imbalance[3][1] * dmg_factor[3])
            * 0.25
            * self.frame_harmonic_1_amp
            * self.frame_harmonic_phase_1.cos()
            * rpm_factor_h_1_inv
            * rpm_factor_h;
        noise[1] += (self.motor_imbalance[0][1] * dmg_factor[0]
            + self.motor_imbalance[1][1] * dmg_factor[1]
            + self.motor_imbalance[2][1] * dmg_factor[2]
            + self.motor_imbalance[3][1] * dmg_factor[3])
            * 0.25
            * self.frame_harmonic_2_amp
            * self.frame_harmonic_phase_2.cos()
            * rpm_factor_h_2_inv
            * rpm_factor_h;

        noise[2] += (self.motor_imbalance[0][0] * dmg_factor[0]
            + self.motor_imbalance[1][0] * dmg_factor[1]
            + self.motor_imbalance[2][0] * dmg_factor[2]
            + self.motor_imbalance[3][0] * dmg_factor[3])
            * 0.25
            * self.frame_harmonic_1_amp
            * self.frame_harmonic_phase_1.sin()
            * rpm_factor_h_1_inv
            * rpm_factor_h;
        noise[2] += (self.motor_imbalance[0][2] * dmg_factor[0]
            + self.motor_imbalance[1][2] * dmg_factor[1]
            + self.motor_imbalance[2][2] * dmg_factor[2]
            + self.motor_imbalance[3][2] * dmg_factor[3])
            * 0.25
            * self.frame_harmonic_2_amp
            * self.frame_harmonic_phase_2.sin()
            * rpm_factor_h_2_inv
            * rpm_factor_h;
        self.motor_noise = noise;
    }

    #[cfg(feature = "noise")]
    pub fn calculate_noise(&mut self, dt: f64) {
        self.gyro.update_gyro_noise();
        self.update_motor_noise(dt);
        self.combined_noise = self.gyro.gyro_noise() + self.motor_noise;
    }

    pub fn update_gyro(&mut self, dt: f64) {
        // Only calculate noise if we need it to
        #[cfg(feature = "noise")]
        self.calculate_noise(dt);

        // sets attitude
        self.gyro
            .set_rotation(mat3_to_quat(self.rigid_body.rotation));

        self.gyro.set_angular_velocity(
            self.rigid_body.rotation,
            self.rigid_body.angular_velocity,
            self.combined_noise,
            dt,
        );

        // sets acceleration
        self.gyro
            .set_acceleration(self.rigid_body.rotation, self.rigid_body.acceleration);
    }
}
