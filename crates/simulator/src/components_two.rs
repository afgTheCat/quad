// This is a more or less faithful implementation of the original simitl cpp simulator

use crate::{
    constants::{AIR_RHO, GRAVITY, MAX_EFFECT_SPEED},
    low_pass_filter::LowPassFilter,
    rng_gen_range,
    sample_curve::SampleCurve,
    DroneUpdate, SimulationDebugInfo,
};
use derive_more::derive::{Deref, DerefMut};
use flight_controller::{BatteryUpdate, GyroUpdate, MotorInput};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3, Vector4};
use std::f64::consts::PI;

#[derive(Debug, Clone, Default)]
pub struct BatteryStateTwo {
    pub capacity: f64,
    pub bat_voltage: f64,
    pub bat_voltage_sag: f64,
    pub amperage: f64,
    pub m_ah_drawn: f64,
}

impl BatteryStateTwo {
    pub fn battery_update(&self, cell_count: u8) -> BatteryUpdate {
        BatteryUpdate {
            cell_count,
            bat_voltage_sag: self.bat_voltage_sag,
            bat_voltage: self.bat_voltage,
            amperage: self.amperage,
            m_ah_drawn: self.m_ah_drawn,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RotorStateTwo {
    pub current: f64,
    pub rpm: f64,
    pub motor_torque: f64,     // the torque calculated
    pub effective_thrust: f64, // reverse thrust is not accounted for
    pub pwm: f64,
    pub rotor_dir: f64,
    pub motor_pos: Vector3<f64>,
}

#[derive(Debug, Deref, DerefMut, Clone)]
pub struct RotorsStateTwo(pub [RotorStateTwo; 4]);

#[derive(Debug, Clone)]
pub struct DroneFrameStateTwo {
    pub position: Vector3<f64>,
    pub rotation: Rotation3<f64>,
    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct SimulationFrame {
    pub battery_state: BatteryStateTwo,
    pub rotors_state: RotorsStateTwo,
    pub drone_state: DroneFrameStateTwo,
    pub gyro_state: GyroStateTwo,
}

// this is probably not useful anymore
trait FrameModel {
    fn set_new_state(
        &mut self,
        current_frame: &SimulationFrame,
        next_frame: &mut SimulationFrame,
        dt: f64,
    );
}

#[derive(Debug, Clone)]
pub struct BatteryModel {
    pub quad_bat_capacity: f64,
    pub bat_voltage_curve: SampleCurve,
    pub quad_bat_cell_count: u8,
    pub quad_bat_capacity_charged: f64,
    pub max_voltage_sag: f64,
}

impl FrameModel for BatteryModel {
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
        next_frame.battery_state = BatteryStateTwo {
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

impl FrameModel for RotorModel {
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
            next_frame.rotors_state[i] = RotorStateTwo {
                rpm,
                current,
                effective_thrust,
                motor_torque,
                pwm: rotor.pwm,
                rotor_dir: rotor.rotor_dir, // This is here as it will be used later on
                motor_pos: rotor.motor_pos,
            };
        }
    }
}

pub struct DroneModel {
    pub frame_drag_area: Vector3<f64>,
    pub frame_drag_constant: f64,
    pub mass: f64,
    pub inv_tensor: Matrix3<f64>,
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

fn cross_product_matrix(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0., -v[2], v[1], v[2], 0., -v[0], -v[1], v[0], 0.)
}

// TODO: this could be better
impl FrameModel for DroneModel {
    fn set_new_state(
        &mut self,
        current_frame: &SimulationFrame,
        next_frame: &mut SimulationFrame,
        dt: f64,
    ) {
        let mut sum_force = Vector3::new(0., -GRAVITY * self.mass, 0.);
        let mut sum_torque = Vector3::zeros();

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

        sum_force -= self.drag_linear(&drag_dir, &linear_velocity_dir, rotation);
        // TODO: once testing is done, readd this to the moments
        let drag_angular = self.drag_angular(&drag_dir, &linear_velocity_dir, rotation);

        let speed_factor = f64::min(speed / MAX_EFFECT_SPEED, 1.);
        for rotor in next_frame.rotors_state.iter() {
            // apply motor torque
            sum_torque += rotation.matrix().column(1) * rotor.motor_torque * rotor.rotor_dir;
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

        let acceleration = sum_force / self.mass;
        let position = current_frame.drone_state.position
            + dt * current_frame.drone_state.linear_velocity
            + (acceleration * dt.powi(2)) / 2.;
        let linear_velocity = current_frame.drone_state.linear_velocity + acceleration * dt;

        let angular_acc: Vector3<f64> =
            rotation * self.inv_tensor * rotation.transpose() * sum_torque;
        let angular_velocity = current_frame.drone_state.angular_velocity + angular_acc * dt;
        let rotation = Rotation3::from_matrix_eps(
            &((Matrix3::identity() + cross_product_matrix(angular_velocity * dt))
                * rotation.matrix()),
            0.0000000001,
            100,
            rotation,
        );

        next_frame.drone_state = DroneFrameStateTwo {
            position,
            rotation,
            linear_velocity,
            angular_velocity,
            acceleration,
        };
    }
}

#[derive(Debug, Clone)]
pub struct GyroStateTwo {
    pub rotation: UnitQuaternion<f64>, // so far it was w, i, j, k
    pub acceleration: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
}

impl GyroStateTwo {
    fn gyro_update(&self) -> GyroUpdate {
        GyroUpdate {
            rotation: [
                self.rotation.w,
                self.rotation.i,
                self.rotation.j,
                self.rotation.k,
            ],
            linear_acc: self.acceleration.data.0[0],
            angular_velocity: self.angular_velocity.data.0[0],
        }
    }
}

// we can possibly integrate things here
pub struct GyroModel {
    pub low_pass_filter: [LowPassFilter; 3],
}

impl FrameModel for GyroModel {
    fn set_new_state(&mut self, _: &SimulationFrame, next_frame: &mut SimulationFrame, dt: f64) {
        let rotation = next_frame.drone_state.rotation;
        let frame_angular_velocity = next_frame.drone_state.angular_velocity;
        let cutoff_freq = 300.;
        let gyro_vel_x = self.low_pass_filter[0].update(frame_angular_velocity[0], dt, cutoff_freq);
        let gyro_vel_y = self.low_pass_filter[1].update(frame_angular_velocity[1], dt, cutoff_freq);
        let gyro_vel_z = self.low_pass_filter[2].update(frame_angular_velocity[2], dt, cutoff_freq);
        let angular_velocity =
            rotation.transpose() * Vector3::new(gyro_vel_x, gyro_vel_y, gyro_vel_z);
        let acceleration = rotation.transpose() * next_frame.drone_state.acceleration;
        next_frame.gyro_state = GyroStateTwo {
            rotation: UnitQuaternion::from(rotation),
            acceleration,
            angular_velocity,
        }
    }
}

pub struct SimulationTwo {
    // data
    pub current_frame: SimulationFrame,
    pub next_frame: SimulationFrame,

    // models
    pub battery_model: BatteryModel,
    pub rotor_model: RotorModel,
    pub drone_model: DroneModel,
    pub gyro_model: GyroModel,

    pub dt: f64,
}

impl SimulationTwo {
    pub fn set_motor_pwms(&mut self, pwms: MotorInput) {
        let rotor_state = &mut self.current_frame.rotors_state;
        for i in 0..4 {
            rotor_state.0[i].pwm = pwms[i];
        }
    }

    pub fn update(&mut self) -> DroneUpdate {
        self.battery_model
            .set_new_state(&self.current_frame, &mut self.next_frame, self.dt);
        self.rotor_model
            .set_new_state(&self.current_frame, &mut self.next_frame, self.dt);
        self.drone_model
            .set_new_state(&self.current_frame, &mut self.next_frame, self.dt);
        self.gyro_model
            .set_new_state(&self.current_frame, &mut self.next_frame, self.dt);

        std::mem::swap(&mut self.current_frame, &mut self.next_frame);
        DroneUpdate {
            gyro_update: self.current_frame.gyro_state.gyro_update(),
            battery_update: self
                .current_frame
                .battery_state
                .battery_update(self.battery_model.quad_bat_cell_count),
        }
    }

    pub fn debug_info(&self) -> SimulationDebugInfo {
        let rotors_state = &self.current_frame.rotors_state;
        let drone_state = &self.current_frame.drone_state;
        let battery_state = &self.current_frame.battery_state;

        // TODO: readd this
        let thrusts = Vector4::from_row_slice(
            &rotors_state
                .iter()
                .map(|r| r.effective_thrust)
                .collect::<Vec<f64>>(),
        );
        let rpms =
            Vector4::from_row_slice(&rotors_state.iter().map(|r| r.rpm).collect::<Vec<f64>>());
        let pwms =
            Vector4::from_row_slice(&rotors_state.iter().map(|r| r.rpm).collect::<Vec<f64>>());

        SimulationDebugInfo {
            rotation: drone_state.rotation,
            position: drone_state.position,
            linear_velocity: drone_state.linear_velocity,
            acceleration: drone_state.acceleration,
            angular_velocity: drone_state.angular_velocity,
            thrusts,
            rpms,
            pwms,
            bat_voltage: battery_state.bat_voltage,
            bat_voltage_sag: battery_state.bat_voltage_sag,
        }
    }
}
