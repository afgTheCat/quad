pub mod default_drone;

use derive_more::derive::{Deref, DerefMut};
use flight_controller::{BatteryUpdate, GyroUpdate, MotorInput};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use rand::{Rng, SeedableRng, rngs::StdRng};
use serde::{Deserialize, Serialize};
use std::{cell::RefCell, f64::consts::PI, ops::Range};

pub const MAX_EFFECT_SPEED: f64 = 18.0;
pub const AIR_RHO: f64 = 1.225;
pub const GRAVITY: f64 = 9.81;

thread_local! {
    static RNG: RefCell<StdRng> = RefCell::new(StdRng::seed_from_u64(0));
}

pub fn rng_gen_range(range: Range<f64>) -> f64 {
    RNG.with(|rng| rng.borrow_mut().gen_range(range))
}

fn interpolate(a: f64, b: f64, i: f64) -> f64 {
    a + ((b - a) * i)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SamplePoint {
    pub discharge: f64,
    pub voltage: f64,
}

impl SamplePoint {
    pub fn new(discharge: f64, voltage: f64) -> Self {
        Self { discharge, voltage }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SampleCurve {
    pub sample_points: Vec<SamplePoint>,
    pub min_discharge_point: SamplePoint,
    pub max_discharge_point: SamplePoint,
}

impl SampleCurve {
    pub fn new(sample_points: Vec<SamplePoint>) -> Self {
        let min_discharge_point = sample_points.first().unwrap().clone();
        let max_discharge_point = sample_points.last().unwrap().clone();
        Self {
            sample_points,
            min_discharge_point,
            max_discharge_point,
        }
    }

    pub fn sample(&self, discharge_level: f64) -> f64 {
        if discharge_level > self.max_discharge_point.discharge {
            self.max_discharge_point.voltage
        } else if discharge_level < self.min_discharge_point.discharge {
            self.min_discharge_point.discharge
        } else {
            self.sample_points
                .iter()
                .enumerate()
                .find_map(|(index, sample)| {
                    let next_sample = &self.sample_points[index + 1];
                    if sample.discharge <= discharge_level
                        && next_sample.discharge > discharge_level
                    {
                        let factor = (discharge_level - sample.discharge)
                            / (next_sample.discharge - sample.discharge);
                        Some(interpolate(sample.voltage, next_sample.voltage, factor))
                    } else {
                        None
                    }
                })
                .unwrap()
        }
    }
}

fn cross_product_matrix(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0., -v[2], v[1], v[2], 0., -v[0], -v[1], v[0], 0.)
}

trait DroneComponent {
    fn set_new_state(
        &self,
        current_frame: &SimulationFrame,
        next_frame: &mut SimulationFrame,
        dt: f64,
    );
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LowPassFilter {
    pub output: f64,
    pub e_pow: f64,
}

impl LowPassFilter {
    pub fn new(output: f64, e_pow: f64) -> Self {
        Self { output, e_pow }
    }

    pub fn update(&self, input: f64, delta_time: f64, cutoff_frequency: f64) -> (f64, f64) {
        let e_pow = 1.0 - f64::exp(-delta_time * 2.0 * PI * cutoff_frequency);
        let output = self.output + (input - self.output) * self.e_pow;
        (output, e_pow)
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BatteryState {
    pub capacity: f64,
    pub bat_voltage: f64, // -> view
    pub bat_voltage_sag: f64,
    pub amperage: f64,
    pub m_ah_drawn: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RotorState {
    pub current: f64,
    pub rpm: f64,
    pub motor_torque: f64,     // the torque calculated
    pub effective_thrust: f64, // reverse thrust is not accounted for
    pub pwm: f64,
    pub rotor_dir: f64,
    pub motor_pos: Vector3<f64>,
    pub pwm_low_pass_filter: LowPassFilter,
}

#[derive(Debug, Deref, DerefMut, Clone, Serialize, Deserialize)]
pub struct RotorsState(pub [RotorState; 4]);

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DroneFrameState {
    pub position: Vector3<f64>,
    pub rotation: Rotation3<f64>,
    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GyroState {
    pub rotation: UnitQuaternion<f64>, // so far it was w, i, j, k
    pub acceleration: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub low_pass_filters: [LowPassFilter; 3],
}

impl GyroState {
    pub fn gyro_update(&self) -> GyroUpdate {
        GyroUpdate {
            rotation: [
                self.rotation.i,
                self.rotation.j,
                self.rotation.k,
                self.rotation.w,
            ],
            linear_acc: self.acceleration.data.0[0],
            angular_velocity: self.angular_velocity.data.0[0],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationFrame {
    pub battery_state: BatteryState,
    pub rotors_state: RotorsState,
    pub drone_frame_state: DroneFrameState,
    pub gyro_state: GyroState,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryModel {
    pub quad_bat_capacity: f64,
    pub bat_voltage_curve: SampleCurve,
    pub quad_bat_cell_count: u64,
    pub quad_bat_capacity_charged: f64,
    pub max_voltage_sag: f64,
}

impl DroneComponent for BatteryModel {
    fn set_new_state(
        &self,
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
        let bat_voltage_sag =
            f64::clamp(bat_voltage - v_sag - rng_gen_range(-0.01..0.01), 0.0, 100.);
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
#[derive(Debug, Clone, Serialize, Deserialize)]
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

impl DroneComponent for RotorModel {
    fn set_new_state(
        &self,
        current_frame: &SimulationFrame,
        next_frame: &mut SimulationFrame,
        dt: f64,
    ) {
        let vel_up = f64::max(
            0.,
            Vector3::dot(
                &current_frame.drone_frame_state.linear_velocity,
                &current_frame.drone_frame_state.rotation.matrix().column(0),
            ),
        );

        let state = &current_frame.rotors_state;
        for (i, rotor) in state.iter().enumerate() {
            let (motor_pwm, motor_e_pow) = rotor.pwm_low_pass_filter.update(rotor.pwm, dt, 120.);
            let armature_volt = motor_pwm * current_frame.battery_state.bat_voltage_sag;

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
                pwm: rotor.pwm,
                rotor_dir: rotor.rotor_dir, // This is here as it will be used later on
                motor_pos: rotor.motor_pos,
                pwm_low_pass_filter: LowPassFilter::new(motor_pwm, motor_e_pow),
            };
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DroneModel {
    pub frame_drag_area: Vector3<f64>,
    pub frame_drag_constant: f64,
    pub mass: f64,
    pub inv_tensor: Matrix3<f64>,
}

impl DroneComponent for DroneModel {
    fn set_new_state(
        &self,
        current_frame: &SimulationFrame,
        next_frame: &mut SimulationFrame,
        dt: f64,
    ) {
        let mut sum_force = Vector3::new(0., -GRAVITY * self.mass, 0.);
        let mut sum_torque = Vector3::zeros();

        let rotation = current_frame.drone_frame_state.rotation;
        let (linear_velocity_dir, speed) =
            if current_frame.drone_frame_state.linear_velocity.lp_norm(1) > 0. {
                let linear_velocity_dir =
                    current_frame.drone_frame_state.linear_velocity.normalize();
                let speed = current_frame.drone_frame_state.linear_velocity.norm();
                (linear_velocity_dir, speed)
            } else {
                (Vector3::zeros(), 0.)
            };
        let drag_dir =
            speed.powi(2) * linear_velocity_dir * 0.5 * AIR_RHO * self.frame_drag_constant;

        sum_force -= self.drag_linear(&drag_dir, &linear_velocity_dir, rotation);

        // let local_dir = rotation.transpose() * linear_velocity_dir;
        // let area_angular = Vector3::dot(&self.frame_drag_area, &local_dir);

        // TODO: once testing is done, readd this to the moments
        // let drag_angular = self.drag_angular(&drag_dir, &linear_velocity_dir, rotation);

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
        let position = current_frame.drone_frame_state.position
            + dt * current_frame.drone_frame_state.linear_velocity
            + (acceleration * dt.powi(2)) / 2.;
        let linear_velocity = current_frame.drone_frame_state.linear_velocity + acceleration * dt;

        let angular_acc: Vector3<f64> =
            rotation * self.inv_tensor * rotation.transpose() * sum_torque;
        let angular_velocity = current_frame.drone_frame_state.angular_velocity + angular_acc * dt;
        let rotation = Rotation3::from_matrix_eps(
            &((Matrix3::identity() + cross_product_matrix(angular_velocity * dt))
                * rotation.matrix()),
            0.0000000001,
            100,
            rotation,
        );

        next_frame.drone_frame_state = DroneFrameState {
            position,
            rotation,
            linear_velocity,
            angular_velocity,
            acceleration,
        };
    }
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
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GyroModel {
    // pub low_pass_filter: [LowPassFilter; 3],
}

impl DroneComponent for GyroModel {
    fn set_new_state(
        &self,
        current_frame: &SimulationFrame,
        next_frame: &mut SimulationFrame,
        dt: f64,
    ) {
        let rotation = next_frame.drone_frame_state.rotation;
        let frame_angular_velocity = next_frame.drone_frame_state.angular_velocity;
        let cutoff_freq = 300.;
        let (gyro_vel_x, gyro_x_e_pow) = current_frame.gyro_state.low_pass_filters[0].update(
            frame_angular_velocity[0],
            dt,
            cutoff_freq,
        );
        let (gyro_vel_y, gyro_y_e_pow) = current_frame.gyro_state.low_pass_filters[1].update(
            frame_angular_velocity[1],
            dt,
            cutoff_freq,
        );
        let (gyro_vel_z, gyro_z_e_pow) = current_frame.gyro_state.low_pass_filters[2].update(
            frame_angular_velocity[2],
            dt,
            cutoff_freq,
        );
        let angular_velocity =
            rotation.transpose() * Vector3::new(gyro_vel_x, gyro_vel_y, gyro_vel_z);
        let acceleration = rotation.transpose() * next_frame.drone_frame_state.acceleration;
        next_frame.gyro_state = GyroState {
            rotation: UnitQuaternion::from(rotation),
            acceleration,
            angular_velocity,
            low_pass_filters: [
                LowPassFilter::new(gyro_vel_x, gyro_x_e_pow),
                LowPassFilter::new(gyro_vel_y, gyro_y_e_pow),
                LowPassFilter::new(gyro_vel_z, gyro_z_e_pow),
            ],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Drone {
    // data
    pub current_frame: SimulationFrame,
    pub next_frame: SimulationFrame,

    // models
    pub battery_model: BatteryModel,
    pub rotor_model: RotorModel,
    pub drone_model: DroneModel,
    pub gyro_model: GyroModel,
}

impl Drone {
    pub fn reset(&mut self, initial_frame: SimulationFrame) {
        self.current_frame = initial_frame.clone();
        self.next_frame = initial_frame.clone();
    }

    pub fn set_motor_pwms(&mut self, pwms: MotorInput) {
        let rotor_state = &mut self.current_frame.rotors_state;
        for i in 0..4 {
            rotor_state.0[i].pwm = pwms[i];
        }
    }

    pub fn update(&mut self, dt: f64) {
        self.battery_model
            .set_new_state(&self.current_frame, &mut self.next_frame, dt);
        self.rotor_model
            .set_new_state(&self.current_frame, &mut self.next_frame, dt);
        self.drone_model
            .set_new_state(&self.current_frame, &mut self.next_frame, dt);
        self.gyro_model
            .set_new_state(&self.current_frame, &mut self.next_frame, dt);

        std::mem::swap(&mut self.current_frame, &mut self.next_frame);
    }

    pub fn battery_update(&self) -> BatteryUpdate {
        let cell_count = self.battery_model.quad_bat_cell_count;
        let battery_state = &self.current_frame.battery_state;
        BatteryUpdate {
            cell_count,
            bat_voltage_sag: battery_state.bat_voltage_sag,
            bat_voltage: battery_state.bat_voltage,
            amperage: battery_state.amperage,
            m_ah_drawn: battery_state.m_ah_drawn,
        }
    }

    pub fn motor_input(&self) -> MotorInput {
        let rotor_state = &self.current_frame.rotors_state;
        MotorInput {
            input: [
                rotor_state.0[0].pwm,
                rotor_state.0[1].pwm,
                rotor_state.0[2].pwm,
                rotor_state.0[3].pwm,
            ],
        }
    }

    pub fn position(&self) -> Vector3<f64> {
        self.current_frame.drone_frame_state.position
    }
}
