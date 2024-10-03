mod arm;
mod battery;
mod gyro;
mod low_pass_filter;
mod motor;
mod propeller;
mod rigid_body;
mod sample_curve;
pub mod state_packet;
mod state_update_packet;

use crate::constants::M_PI;
use crate::rng_gen_range;
use arm::Arm;
pub use battery::Battery;
use bevy::{
    math::{DMat3, DVec3, DVec4},
    prelude::Component,
};
use gyro::Gyro;
use rigid_body::RigidBody;
use state_packet::StatePacket;
use state_update_packet::StateUpdatePacket;

pub fn xform_inv(m: DMat3, v: DVec3) -> DVec3 {
    DVec3::new(
        (m.x_axis[0] * v[0]) + (m.y_axis[0] * v[1]) + (m.z_axis[0] * v[2]),
        (m.x_axis[1] * v[0]) + (m.y_axis[1] * v[1]) + (m.z_axis[1] * v[2]),
        (m.x_axis[2] * v[0]) + (m.y_axis[2] * v[1]) + (m.z_axis[2] * v[2]),
    )
}

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

#[derive(Clone, Component)]
pub struct Drone {
    armed: bool,

    prop_harmonic_1_amp: f64,
    prop_harmonic_2_amp: f64,
    frame_harmonic_phase_1: f64,
    frame_harmonic_phase_2: f64,
    combined_noise: DVec3,
    acceleration: DVec3,

    motor_noise: DVec3,
    battery: Battery,
    arms: [Arm; 4],
    body: RigidBody,
    gyro: Gyro,
}

impl Drone {
    pub fn new(
        armed: bool,
        prop_harmonic_1_amp: f64,
        prop_harmonic_2_amp: f64,
        frame_harmonic_phase_1: f64,
        frame_harmonic_phase_2: f64,
        combined_noise: DVec3,
        acceleration: DVec3,
        motor_noise: DVec3,
        battery: Battery,
        arms: [Arm; 4],
        body: RigidBody,
        gyro: Gyro,
    ) -> Self {
        Self {
            armed,
            prop_harmonic_1_amp,
            prop_harmonic_2_amp,
            frame_harmonic_phase_1,
            frame_harmonic_phase_2,
            combined_noise,
            acceleration,
            motor_noise,
            battery,
            arms,
            body,
            gyro,
        }
    }

    pub fn calculate_motors(
        &mut self,
        state_packet: &StatePacket,
        dt: f64,
        ambient_temp: f64,
    ) -> f64 {
        let mut res_prop_torque: f64 = 0.;
        let vbat = self.battery.vbat_sagged(); // is this what we want?
        for i in 0..4 {
            res_prop_torque += self.arms[i].calculate_arm_m_torque(
                state_packet,
                dt,
                vbat,
                self.armed,
                ambient_temp,
            );
        }
        res_prop_torque
    }

    pub fn calculate_physics(
        &mut self,
        state_packet: &mut StatePacket,
        motor_torque: f64,
        dt: f64,
    ) -> DVec3 {
        let gravity_force = self.body.gravity_force();
        let drag_linear = self
            .body
            .drag_linear(state_packet.linear_velocity, state_packet.rotation);
        let total_force = gravity_force - drag_linear
            + self.arms.iter().fold(DVec3::new(0., 0., 0.), |acc, arm| {
                acc + xform(state_packet.rotation, DVec3::new(0., arm.thrust(), 0.))
            });
        let acceleration = self.body.acceleration(total_force);
        // Step 1: update linear velocity
        state_packet.linear_velocity =
            state_packet.linear_velocity.clone() + acceleration.clone() * dt;
        let drag_angular = self
            .body
            .drag_angular(state_packet.linear_velocity, state_packet.rotation);

        let prop_torques = self.arms.iter().fold(DVec3::new(0., 0., 0.), |acc, arm| {
            let force = xform(state_packet.rotation, DVec3::new(0., arm.thrust(), 0.));
            let rad = xform(state_packet.rotation, arm.motor_pos());
            acc + DVec3::cross(rad, force)
        });
        let total_moment = state_packet.rotation.y_axis * motor_torque
            + state_packet.rotation.x_axis * drag_angular[1]
            + state_packet.rotation.y_axis * drag_angular[0]
            + state_packet.rotation.z_axis * drag_angular[2]
            + prop_torques;
        let angular_acc = self.body.angular_acc(total_moment, &state_packet.rotation);
        let angular_velocity = (state_packet.angular_velocity.clone() + angular_acc * dt).clamp(
            DVec3::new(-100., -100., -100.),
            DVec3::new(100., 100., 100.),
        );
        // Step 2: update angular velocity
        state_packet.angular_velocity = angular_velocity.clone();

        let rotation = self
            .body
            .rotation(angular_velocity, dt, &state_packet.rotation);
        // STEP 3: update rotation
        state_packet.rotation = rotation;
        acceleration
    }

    pub fn update_physics(
        &mut self,
        state_packet: &mut StatePacket,
        dt: f64,
        ambient_temp: f64,
    ) -> StateUpdatePacket {
        let motor_torque = self.calculate_motors(state_packet, dt, ambient_temp);
        self.acceleration = self.calculate_physics(state_packet, motor_torque, dt);

        let pwm_sum = self.arms.iter().fold(0.0, |pwm, arm| pwm + arm.pwm());
        let current_sum = self.arms.iter().fold(0., |acc, arm| acc + arm.current());
        self.battery.update(dt, pwm_sum, current_sum);
        // bunch of updates to the state update packet
        let orientation = mat3_to_quat(state_packet.rotation);
        StateUpdatePacket::new(
            orientation,
            state_packet.angular_velocity,
            state_packet.linear_velocity,
            self.motor_rpm(),
            self.motor_temp(),
            false, // wtf
        )
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

    fn update_motor_noise(&mut self, dt: f64, state_packet: &StatePacket) {
        let max_v = self.battery.cell_count() * 4.2;
        let max_rpm = (self.motor_kv() * max_v).max(DVec4::splat(0.1));
        let motor_rpm = self.motor_rpm();
        let rpm_factor = motor_rpm.max(DVec4::ZERO) / max_rpm.clone();
        let rpm_factor_squared = rpm_factor.powf(2.);
        let dmg_factor = state_packet.prop_damage + 0.05;
        let rpm_dmg_factor = dmg_factor.clone() * rpm_factor_squared;
        let m_noise = self.motor_noise(dt);

        let mut noise = DVec3::new(0., 0., 0.);
        for i in 0..3 {
            noise[0] +=
                m_noise[i].x_axis[0] * state_packet.motor_imbalance[i][0] * rpm_dmg_factor[i]
                    + m_noise[i].x_axis[1]
                        * state_packet.motor_imbalance[i][0]
                        * rpm_dmg_factor[i]
                        * self.prop_harmonic_1_amp
                    + m_noise[i].x_axis[2]
                        * state_packet.motor_imbalance[i][0]
                        * rpm_dmg_factor[i]
                        * self.prop_harmonic_2_amp;
            noise[1] +=
                m_noise[i].y_axis[0] * state_packet.motor_imbalance[i][1] * rpm_dmg_factor[i]
                    + m_noise[i].y_axis[1]
                        * state_packet.motor_imbalance[i][1]
                        * rpm_dmg_factor[i]
                        * self.prop_harmonic_1_amp
                    + m_noise[i].y_axis[2]
                        * state_packet.motor_imbalance[i][1]
                        * rpm_dmg_factor[i]
                        * self.prop_harmonic_2_amp;

            noise[2] +=
                (m_noise[i].z_axis[0] * state_packet.motor_imbalance[i][2] * rpm_dmg_factor[i]
                    + m_noise[i].z_axis[1]
                        * state_packet.motor_imbalance[i][2]
                        * rpm_dmg_factor[i]
                        * self.prop_harmonic_1_amp
                    + m_noise[i].z_axis[2]
                        * state_packet.motor_imbalance[i][2]
                        * rpm_dmg_factor[i]
                        * self.prop_harmonic_2_amp)
                    * 0.5;
        }
        self.frame_harmonic_phase_1 = shifted_phase(
            dt,
            state_packet.frame_harmonic_1_freq + rng_gen_range(-70.0..70.),
            self.frame_harmonic_phase_1,
        );
        self.frame_harmonic_phase_2 = shifted_phase(
            dt,
            state_packet.frame_harmonic_2_freq + rng_gen_range(-60.0..60.),
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

        noise[0] += (state_packet.motor_imbalance[0][0] * dmg_factor[0]
            + state_packet.motor_imbalance[1][0] * dmg_factor[1]
            + state_packet.motor_imbalance[2][0] * dmg_factor[2]
            + state_packet.motor_imbalance[3][0] * dmg_factor[3])
            * 0.25
            * state_packet.frame_harmonic_1_amp
            * self.frame_harmonic_phase_1.sin()
            * rpm_factor_h_1_inv
            * rpm_factor_h;
        noise[0] += (state_packet.motor_imbalance[0][0] * dmg_factor[0]
            + state_packet.motor_imbalance[1][0] * dmg_factor[1]
            + state_packet.motor_imbalance[2][0] * dmg_factor[2]
            + state_packet.motor_imbalance[3][0] * dmg_factor[3])
            * 0.25
            * state_packet.frame_harmonic_2_amp
            * self.frame_harmonic_phase_2.sin()
            * rpm_factor_h_2_inv
            * rpm_factor_h;

        noise[1] += (state_packet.motor_imbalance[0][1] * dmg_factor[0]
            + state_packet.motor_imbalance[1][1] * dmg_factor[1]
            + state_packet.motor_imbalance[2][1] * dmg_factor[2]
            + state_packet.motor_imbalance[3][1] * dmg_factor[3])
            * 0.25
            * state_packet.frame_harmonic_1_amp
            * self.frame_harmonic_phase_1.cos()
            * rpm_factor_h_1_inv
            * rpm_factor_h;
        noise[1] += (state_packet.motor_imbalance[0][1] * dmg_factor[0]
            + state_packet.motor_imbalance[1][1] * dmg_factor[1]
            + state_packet.motor_imbalance[2][1] * dmg_factor[2]
            + state_packet.motor_imbalance[3][1] * dmg_factor[3])
            * 0.25
            * state_packet.frame_harmonic_2_amp
            * self.frame_harmonic_phase_2.cos()
            * rpm_factor_h_2_inv
            * rpm_factor_h;

        noise[2] += (state_packet.motor_imbalance[0][0] * dmg_factor[0]
            + state_packet.motor_imbalance[1][0] * dmg_factor[1]
            + state_packet.motor_imbalance[2][0] * dmg_factor[2]
            + state_packet.motor_imbalance[3][0] * dmg_factor[3])
            * 0.25
            * state_packet.frame_harmonic_1_amp
            * self.frame_harmonic_phase_1.sin()
            * rpm_factor_h_1_inv
            * rpm_factor_h;
        noise[2] += (state_packet.motor_imbalance[0][2] * dmg_factor[0]
            + state_packet.motor_imbalance[1][2] * dmg_factor[1]
            + state_packet.motor_imbalance[2][2] * dmg_factor[2]
            + state_packet.motor_imbalance[3][2] * dmg_factor[3])
            * 0.25
            * state_packet.frame_harmonic_2_amp
            * self.frame_harmonic_phase_2.sin()
            * rpm_factor_h_2_inv
            * rpm_factor_h;
        self.motor_noise = noise;
    }

    pub fn update_gyro(&mut self, state_packet: &StatePacket, dt: f64) {
        self.gyro
            .update_gyro_noise(state_packet.gyro_base_noise_amp);
        self.update_motor_noise(dt, state_packet);
        self.combined_noise = self.gyro.gyro_noise() + self.motor_noise.clone();
        self.gyro.set_rotation(mat3_to_quat(state_packet.rotation));
        let mut angular_velocity =
            state_packet.angular_velocity.clone() + self.combined_noise.clone();

        let cutoff_freq = 300.;
        angular_velocity[0] =
            self.gyro.low_pass_filter()[0].update(angular_velocity[0], dt, cutoff_freq);
        angular_velocity[1] =
            self.gyro.low_pass_filter()[1].update(angular_velocity[0], dt, cutoff_freq);
        angular_velocity[2] =
            self.gyro.low_pass_filter()[2].update(angular_velocity[0], dt, cutoff_freq);
        self.gyro
            .set_gyro(xform_inv(state_packet.rotation, angular_velocity));
        let gravity_acceleration = DVec3::new(0., -9.81, 0.);
        self.gyro.set_acceleration(xform_inv(
            state_packet.rotation,
            self.acceleration.clone() + gravity_acceleration,
        ));
    }
}
