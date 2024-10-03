use crate::constants::M_PI;

mod arm;
mod battery;
mod gyro;
mod low_pass_filter;
mod motor;
mod propeller;
mod sample_curve;
mod state_packet;

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

pub fn rpm_to_hz(rpm: f64) -> f64 {
    rpm / 60.
}
