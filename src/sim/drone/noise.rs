use super::{shifted_phase, Drone, Gyro};
use crate::rng_gen_range;
use crate::sim::drone::DMat3;
use bevy::math::{DVec3, DVec4};

impl Drone {
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

    fn update_motor_noises(&mut self, dt: f64) -> [DMat3; 4] {
        [
            self.arms[0].motor_noise(dt),
            self.arms[1].motor_noise(dt),
            self.arms[2].motor_noise(dt),
            self.arms[3].motor_noise(dt),
        ]
    }

    fn calculate_motor_noise(&mut self, dt: f64) -> DVec3 {
        let max_v = self.battery.cell_count() * 4.2;
        let max_rpm = (self.motor_kv() * max_v).max(DVec4::splat(0.1));
        let motor_rpm = self.motor_rpm();
        let rpm_factor = motor_rpm.max(DVec4::ZERO) / max_rpm;
        let rpm_factor_squared = rpm_factor.powf(2.);
        let dmg_factor = DVec4::splat(0.05);
        let rpm_dmg_factor = dmg_factor * rpm_factor_squared;
        let m_noise = self.update_motor_noises(dt);

        let mut noise = DVec3::new(0., 0., 0.);
        for i in 0..3 {
            noise[0] += m_noise[i].x_axis[0]
                * self.frame_charachteristics.motor_imbalance[i][0]
                * rpm_dmg_factor[i]
                + m_noise[i].x_axis[1]
                    * self.frame_charachteristics.motor_imbalance[i][0]
                    * rpm_dmg_factor[i]
                    * self.frame_charachteristics.prop_harmonic_1_amp
                + m_noise[i].x_axis[2]
                    * self.frame_charachteristics.motor_imbalance[i][0]
                    * rpm_dmg_factor[i]
                    * self.frame_charachteristics.prop_harmonic_2_amp;
            noise[1] += m_noise[i].y_axis[0]
                * self.frame_charachteristics.motor_imbalance[i][1]
                * rpm_dmg_factor[i]
                + m_noise[i].y_axis[1]
                    * self.frame_charachteristics.motor_imbalance[i][1]
                    * rpm_dmg_factor[i]
                    * self.frame_charachteristics.prop_harmonic_1_amp
                + m_noise[i].y_axis[2]
                    * self.frame_charachteristics.motor_imbalance[i][1]
                    * rpm_dmg_factor[i]
                    * self.frame_charachteristics.prop_harmonic_2_amp;

            noise[2] += (m_noise[i].z_axis[0]
                * self.frame_charachteristics.motor_imbalance[i][2]
                * rpm_dmg_factor[i]
                + m_noise[i].z_axis[1]
                    * self.frame_charachteristics.motor_imbalance[i][2]
                    * rpm_dmg_factor[i]
                    * self.frame_charachteristics.prop_harmonic_1_amp
                + m_noise[i].z_axis[2]
                    * self.frame_charachteristics.motor_imbalance[i][2]
                    * rpm_dmg_factor[i]
                    * self.frame_charachteristics.prop_harmonic_2_amp)
                * 0.5;
        }
        self.frame_charachteristics.frame_harmonic_phase_1 = shifted_phase(
            dt,
            self.frame_charachteristics.frame_harmonic_1_freq + rng_gen_range(-70.0..70.),
            self.frame_charachteristics.frame_harmonic_phase_1,
        );
        self.frame_charachteristics.frame_harmonic_phase_2 = shifted_phase(
            dt,
            self.frame_charachteristics.frame_harmonic_2_freq + rng_gen_range(-60.0..60.),
            self.frame_charachteristics.frame_harmonic_phase_2,
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

        noise[0] += (self.frame_charachteristics.motor_imbalance[0][0] * dmg_factor[0]
            + self.frame_charachteristics.motor_imbalance[1][0] * dmg_factor[1]
            + self.frame_charachteristics.motor_imbalance[2][0] * dmg_factor[2]
            + self.frame_charachteristics.motor_imbalance[3][0] * dmg_factor[3])
            * 0.25
            * self.frame_charachteristics.frame_harmonic_1_amp
            * self.frame_charachteristics.frame_harmonic_phase_1.sin()
            * rpm_factor_h_1_inv
            * rpm_factor_h;
        noise[0] += (self.frame_charachteristics.motor_imbalance[0][0] * dmg_factor[0]
            + self.frame_charachteristics.motor_imbalance[1][0] * dmg_factor[1]
            + self.frame_charachteristics.motor_imbalance[2][0] * dmg_factor[2]
            + self.frame_charachteristics.motor_imbalance[3][0] * dmg_factor[3])
            * 0.25
            * self.frame_charachteristics.frame_harmonic_2_amp
            * self.frame_charachteristics.frame_harmonic_phase_2.sin()
            * rpm_factor_h_2_inv
            * rpm_factor_h;

        noise[1] += (self.frame_charachteristics.motor_imbalance[0][1] * dmg_factor[0]
            + self.frame_charachteristics.motor_imbalance[1][1] * dmg_factor[1]
            + self.frame_charachteristics.motor_imbalance[2][1] * dmg_factor[2]
            + self.frame_charachteristics.motor_imbalance[3][1] * dmg_factor[3])
            * 0.25
            * self.frame_charachteristics.frame_harmonic_1_amp
            * self.frame_charachteristics.frame_harmonic_phase_1.cos()
            * rpm_factor_h_1_inv
            * rpm_factor_h;
        noise[1] += (self.frame_charachteristics.motor_imbalance[0][1] * dmg_factor[0]
            + self.frame_charachteristics.motor_imbalance[1][1] * dmg_factor[1]
            + self.frame_charachteristics.motor_imbalance[2][1] * dmg_factor[2]
            + self.frame_charachteristics.motor_imbalance[3][1] * dmg_factor[3])
            * 0.25
            * self.frame_charachteristics.frame_harmonic_2_amp
            * self.frame_charachteristics.frame_harmonic_phase_2.cos()
            * rpm_factor_h_2_inv
            * rpm_factor_h;

        noise[2] += (self.frame_charachteristics.motor_imbalance[0][0] * dmg_factor[0]
            + self.frame_charachteristics.motor_imbalance[1][0] * dmg_factor[1]
            + self.frame_charachteristics.motor_imbalance[2][0] * dmg_factor[2]
            + self.frame_charachteristics.motor_imbalance[3][0] * dmg_factor[3])
            * 0.25
            * self.frame_charachteristics.frame_harmonic_1_amp
            * self.frame_charachteristics.frame_harmonic_phase_1.sin()
            * rpm_factor_h_1_inv
            * rpm_factor_h;
        noise[2] += (self.frame_charachteristics.motor_imbalance[0][2] * dmg_factor[0]
            + self.frame_charachteristics.motor_imbalance[1][2] * dmg_factor[1]
            + self.frame_charachteristics.motor_imbalance[2][2] * dmg_factor[2]
            + self.frame_charachteristics.motor_imbalance[3][2] * dmg_factor[3])
            * 0.25
            * self.frame_charachteristics.frame_harmonic_2_amp
            * self.frame_charachteristics.frame_harmonic_phase_2.sin()
            * rpm_factor_h_2_inv
            * rpm_factor_h;
        noise
    }

    pub fn calculate_combined_noise(&mut self, dt: f64) -> DVec3 {
        let gyro_noise = self.gyro.calculate_gyro_noise();
        let motor_noise = self.calculate_motor_noise(dt);
        gyro_noise + motor_noise
    }
}

impl Gyro {
    pub fn calculate_gyro_noise(&self) -> DVec3 {
        let white_noise_x = rng_gen_range(-1.0..1.) * self.gyro_base_noise_amp;
        let white_noise_y = rng_gen_range(-1.0..1.) * self.gyro_base_noise_amp;
        let white_noise_z = rng_gen_range(-1.0..1.) * self.gyro_base_noise_amp;
        DVec3::new(white_noise_x, white_noise_y, white_noise_z)
    }
}
