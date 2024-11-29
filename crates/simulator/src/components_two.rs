// This is a more or less faithful implementation of the original simitl cpp simulator

use crate::{
    constants::{AIR_RHO, MAX_EFFECT_SPEED},
    low_pass_filter::LowPassFilter,
    rng_gen_range,
    sample_curve::SampleCurve,
};
use core::f64;
use derive_more::derive::{Deref, DerefMut};
use nalgebra::{ComplexField, Matrix3, Normed, Rotation3, Vector3};
use std::f64::consts::PI;

#[derive(Debug, Clone, Default)]
pub struct BatteryState {
    pub capacity: f64,
    pub bat_voltage: f64,
    pub bat_voltage_sag: f64,
    pub amperage: f64,
    pub m_ah_drawn: f64,
}

struct RotorState {
    current: f64,
    rpm: f64,
    motor_torque: f64,     // the torque calculated
    effective_thrust: f64, // reverse thrust is not accounted for
    pwm: f64,
    rotor_dir: f64,
    motor_pos: Vector3<f64>,
}

#[derive(Deref, DerefMut)]
struct RotorsState([RotorState; 4]);

struct DroneFrameState {
    position: Vector3<f64>,
    rotation: Rotation3<f64>,
    linear_velocity: Vector3<f64>,
    velocity_direction: Option<Vector3<f64>>,
    angular_velocity: Vector3<f64>,
}

struct SimulationFrame {
    battery_state: BatteryState,
    rotors_state: RotorsState,
    drone_state: DroneFrameState,
}

trait FrameComponet {
    fn set_new_state(
        &mut self,
        current_frame: &SimulationFrame,
        next_frame: &mut SimulationFrame,
        dt: f64,
    );
}

#[derive(Debug, Clone)]
pub struct BatteryModel {
    pub quad_bat_capacity: f64, // mAH I guess
    pub bat_voltage_curve: SampleCurve,
    pub quad_bat_cell_count: u8,
    pub quad_bat_capacity_charged: f64,
    pub max_voltage_sag: f64,
}

impl FrameComponet for BatteryModel {
    fn set_new_state(
        &mut self,
        current_frame: &SimulationFrame,
        next_frame: &mut SimulationFrame,
        dt: f64,
    ) {
        let state = &current_frame.battery_state;
        let bat_charge = state.capacity / self.quad_bat_capacity;
        let bat_voltage = f64::max(
            self.bat_voltage_curve.sample(1. - bat_charge) * self.quad_bat_cell_count as f64,
            0.1,
        );
        let pwm_sum: f64 = current_frame.rotors_state.iter().map(|s| s.pwm).sum();
        let power_factor_squared = f64::max(0., pwm_sum / 4.).powi(2);
        let charge_factor_inv =
            1.0 - (state.capacity / f64::max(self.quad_bat_capacity_charged, 1.));

        let v_sag = self.max_voltage_sag * power_factor_squared
            + (self.max_voltage_sag * charge_factor_inv * charge_factor_inv * power_factor_squared);
        let bat_voltage_sag = f64::clamp(
            state.bat_voltage - v_sag - rng_gen_range(-0.01..0.01),
            0.0,
            100.,
        );
        let m_a_min = f64::min(0.2, rng_gen_range(-0.125..0.375)) / f64::max(bat_voltage_sag, 0.01);
        let current_sum: f64 = current_frame.rotors_state.iter().map(|s| s.current).sum();
        let currentm_as = f64::max(current_sum / 3.6, m_a_min);
        let capacity = state.capacity - currentm_as * dt;
        next_frame.battery_state = BatteryState {
            capacity,
            bat_voltage,
            bat_voltage_sag,
            amperage: currentm_as * 3.6,
            m_ah_drawn: self.quad_bat_capacity_charged - capacity,
        }
    }
}

// The rotor model
pub struct RotorModel {
    pub prop_max_rpm: f64,
    pub pwm_low_pass_filter: [LowPassFilter; 4],
    pub motor_kv: f64, // kv
    pub motor_r: f64,  // resistence
    pub motor_io: f64, // idle current
    pub prop_thrust_factor: Vector3<f64>,
    pub prop_torque_factor: f64,
    pub prop_a_factor: f64,
    pub prop_inertia: f64,
}

impl RotorModel {
    // Calculates the motor torque based on the motor torque constant.
    // https://en.wikipedia.org/wiki/Motor_constants#Motor_torque_constant
    fn motor_torque(&self, armature_volts: f64, rpm: f64) -> f64 {
        let kv = self.motor_kv;
        let back_emf_v = rpm / kv;
        let base_current = (armature_volts - back_emf_v) / self.motor_r;
        let armature_current = if base_current > 0. {
            f64::max(0., base_current - self.motor_io)
        } else {
            f64::min(0., base_current + self.motor_io)
        };
        let torque_constant = 8.3 / kv; // why do we need to calculate this?
        armature_current * torque_constant
    }

    // Calculates the current thrust that the rotor is exerting.
    fn prop_thrust(&self, vel_up: f64, rpm: f64) -> f64 {
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

impl FrameComponet for RotorModel {
    fn set_new_state(
        &mut self,
        current_frame: &SimulationFrame,
        next_frame: &mut SimulationFrame,
        dt: f64,
    ) {
        let vel_up = f64::max(
            0.,
            Vector3::dot(
                &current_frame.drone_state.linear_velocity,
                &current_frame.drone_state.rotation.matrix().column(0),
            ),
        );

        let state = &current_frame.rotors_state;
        for (i, rotor) in state.iter().enumerate() {
            // let pwm = update.pwm[i];
            let armature_volt = self.pwm_low_pass_filter[i].update(rotor.pwm, dt, 120.)
                * current_frame.battery_state.bat_voltage_sag;

            // For this calculation we only operate with the effective thrust. I have no idea why
            // but this is the original SITL code and it seems deliberate. I suppose we could duble
            // check this if we have time.
            let prop_torque = rotor.effective_thrust * self.prop_torque_factor;

            // The original new torque was calculated via a motor_torque and a prop torque. The
            // prop torque was a function of the *old* rpm and the new velocity, while the motor
            // torque was calculated using the *new* voltage. This is super messy and probably
            // warrents a rewrite. The issue here is that the net torque depends on the rpm through
            // prop torque, but the net torque itself is calculated to calculate the change in rpm
            let net_torque = rotor.motor_torque - prop_torque;
            let domega = net_torque / self.prop_inertia;
            let drpm = (domega * dt) * 60.0 / (2.0 * PI);
            let maxdrpm = f64::abs(armature_volt * self.motor_kv - rotor.rpm);
            let rpm = rotor.rpm + f64::clamp(drpm, -maxdrpm, maxdrpm);
            let motor_torque = self.motor_torque(armature_volt, rotor.rpm);
            let current = motor_torque * self.motor_kv / 8.3;
            let effective_thrust = self.prop_thrust(vel_up, rpm);
            next_frame.rotors_state[i] = RotorState {
                rpm,
                current,
                effective_thrust,
                motor_torque,
                pwm: 0.,
                rotor_dir: rotor.rotor_dir, // This is here as it will be used later on
                motor_pos: rotor.motor_pos,
            };
        }
    }
}

struct DroneModel {
    frame_drag_area: Vector3<f64>,
    frame_drag_constant: f64,
    motor_positions: [Vector3<f64>; 4],
    motor_dir: [f64; 4],
    mass: f64,
}

impl DroneModel {
    fn drag_linear(
        &self,
        drag_dir: &Vector3<f64>,
        linear_velocity_dir: &Vector3<f64>,
        rotation: Rotation3<f64>,
    ) -> Vector3<f64> {
        let local_dir = rotation.transpose() * linear_velocity_dir;
        let area_linear = Vector3::dot(&self.frame_drag_area, &local_dir.abs());
        drag_dir * area_linear
    }

    fn drag_angular(
        &self,
        drag_dir: &Vector3<f64>,
        linear_velocity_dir: &Vector3<f64>,
        rotation: Rotation3<f64>,
    ) -> Vector3<f64> {
        let local_dir = rotation.transpose() * linear_velocity_dir;
        let area_angular = Vector3::dot(&self.frame_drag_area, &local_dir);
        let drag_angular: Vector3<f64> = rotation.transpose() * (drag_dir * area_angular) * 0.001;
        drag_angular
    }
}

// TODO: this is majorly bad
impl FrameComponet for DroneModel {
    fn set_new_state(
        &mut self,
        current_frame: &SimulationFrame,
        next_frame: &mut SimulationFrame,
        dt: f64,
    ) {
        let mut sum_torque = Vector3::zeros();
        // let mut sum_force = Vector3::zeros();

        let rotation = current_frame.drone_state.rotation;
        let (linear_velocity_dir, speed) =
            if current_frame.drone_state.linear_velocity.lp_norm(1) > 0. {
                let linear_velocity_dir = current_frame.drone_state.linear_velocity.normalize();
                let speed = current_frame.drone_state.linear_velocity.norm();
                (linear_velocity_dir, speed)
            } else {
                (Vector3::zeros(), 0.)
            };
        let drag_dir =
            speed.powi(2) * linear_velocity_dir * 0.5 * AIR_RHO * self.frame_drag_constant;

        let mut sum_force = -self.drag_linear(&drag_dir, &linear_velocity_dir, rotation);
        // TODO: once testing is done, readd this to the moments
        let drag_angular = self.drag_angular(&drag_dir, &linear_velocity_dir, rotation);

        let speed_factor = f64::min(speed / MAX_EFFECT_SPEED, 1.);
        for rotor in next_frame.rotors_state.iter() {
            // apply motor torque
            sum_torque += rotation.matrix().column(1) * rotor.motor_torque;
            let mut reverse_thrust = -Vector3::dot(
                &linear_velocity_dir, // already normalized
                &(rotation.matrix().column(0) * rotor.effective_thrust).normalize(),
            );

            reverse_thrust = f64::max(0.0, reverse_thrust - 0.5) * 2.;
            reverse_thrust = reverse_thrust * reverse_thrust;
            let prop_wash_effect = 1.0 - (speed_factor * reverse_thrust * 0.95);

            let actual_thrust =
                rotation * Vector3::new(0., rotor.effective_thrust * prop_wash_effect, 0.);

            let rad = rotation * rotor.motor_pos;
            sum_torque += Vector3::cross(&rad, &actual_thrust);
            sum_force += actual_thrust;
        }
    }
}
