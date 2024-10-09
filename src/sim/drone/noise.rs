use super::{
    arm::{Arm, Motor, MotorState},
    Drone, Gyro,
};
use crate::sim::drone::DMat3;
use crate::{constants::M_PI, rng_gen_range};
use bevy::math::{DVec3, DVec4};

// Adopted from the Cpp source file, did not test the validity nor the speed
// TODO: we can either speed this up or find an alternative. For now I will just feature flag it
const PERM: [u8; 256] = [
    151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69,
    142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219,
    203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175,
    74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230,
    220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76,
    132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173,
    186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206,
    59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163,
    70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232,
    178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162,
    241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204,
    176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141,
    128, 195, 78, 66, 215, 61, 156, 180,
];

fn hash(i: i32) -> u8 {
    PERM[i as u8 as usize]
}

fn grad(hash: i32, x: f64) -> f64 {
    let h = hash & 0x0F;
    let mut grad = (1 + (h & 7)) as f64;
    if (h & 8) != 0 {
        grad *= -1.
    }
    grad * x
}

#[inline(never)]
pub fn simplex_1d_noise(x: f64) -> f64 {
    let i0 = x.floor();
    let i1 = i0 + 1.;

    let x0 = x - i0;
    let x1 = x0 - 1.;

    let mut t0 = 1.0 - x0 * x0;
    t0 *= t0;

    let n0 = t0 * t0 * grad(hash(i0 as i32) as i32, x0);

    let mut t1 = 1.0 - x1 * x1;
    t1 *= t1;
    let n1 = t1 * t1 * grad(hash(i1 as i32) as i32, x1);
    0.395 * (n0 + n1)
}

pub fn rpm_to_hz(rpm: f64) -> f64 {
    rpm / 60.
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

impl Drone {
    fn motor_kv(&self) -> DVec4 {
        DVec4::new(
            self.arms[0].motor.kv(),
            self.arms[1].motor.kv(),
            self.arms[2].motor.kv(),
            self.arms[3].motor.kv(),
        )
    }

    fn motor_rpm(&self) -> DVec4 {
        DVec4::new(
            self.arms[0].motor.rpm(),
            self.arms[1].motor.rpm(),
            self.arms[2].motor.rpm(),
            self.arms[3].motor.rpm(),
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

impl MotorState {
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

impl Motor {
    pub fn motor_noise(&mut self, dt: f64) -> DMat3 {
        self.state.motor_noise(dt)
    }

    pub fn phase(&self) -> f64 {
        self.state.phase
    }

    pub fn prop_wash_noise(&mut self, dt: f64) -> f64 {
        let motor_phase_compressed = (self.state.phase_slow * 3.0).floor() / 3.0;
        self.state.prop_wash_low_pass_filter.update(
            f64::max(0.0, 0.5 * simplex_1d_noise(motor_phase_compressed) + 1.0),
            dt,
            120.,
        )
    }
}

impl Arm {
    pub fn motor_noise(&mut self, dt: f64) -> DMat3 {
        self.motor.motor_noise(dt)
    }
}
