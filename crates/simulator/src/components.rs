use core::f64;
use std::f64::consts::PI;

use nalgebra::{Rotation3, Vector3};

/// Describe's all the components that the initial quad model has. Current components are the
/// battery, the rotors (motor + propeller), and the possibly the imu's. The component's inner
/// working is described by a model which iteract with a state to generate a new state. This new
/// state can be later used when calculating the rigid body of the drone. Also new and improved
/// models are welcome, since the current one is derived via heristics.
use crate::{low_pass_filter::LowPassFilter, rng_gen_range, sample_curve::SampleCurve};

/// This describes how a component works. Given a component state it calculates the next state that
/// of the drone component
trait ComponentModel {
    type ComponentState; // The state of the model
    type ComponentUpdate; // The update that a component receives

    fn get_new_state(
        &mut self,
        state: &Self::ComponentState,
        update: Self::ComponentUpdate,
    ) -> Self::ComponentState;
}

/// The componen trait represent a component of the drone. In the simulation pipleline, each
/// component is updated first returning the new state for the component. The new state can be used
/// later on during the rigid body update to have more precise integraion (to be implemented)
trait Component {
    type Model: ComponentModel;
    type ComponentUpdate = <Self::Model as ComponentModel>::ComponentUpdate;
    type ComponentState = <Self::Model as ComponentModel>::ComponentState;

    // calculates the new state based on the component udpate. Then new state can be used later on
    // during the integration
    fn update(&mut self, update: Self::ComponentUpdate) -> Self::ComponentState;
    // Sets the new state of the component
    fn set_state(&mut self, state: Self::ComponentState);
}

struct DroneComponent<Model: ComponentModel> {
    model: Model,
    state: Model::ComponentState,
}

impl<Model: ComponentModel> Component for DroneComponent<Model> {
    type Model = Model;

    fn update(&mut self, update: Self::ComponentUpdate) -> Self::ComponentState {
        self.model.get_new_state(&self.state, update)
    }

    fn set_state(&mut self, state: Self::ComponentState) {
        self.state = state;
    }
}

#[derive(Debug, Clone)]
struct BatteryModel {
    pub quad_bat_capacity: f64, // mAH I guess
    pub bat_voltage_curve: SampleCurve,
    pub quad_bat_cell_count: u8,
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

pub struct BatteryUpdate {
    dt: f64,
    pwm_sum: f64,
    current_sum: f64,
}

impl ComponentModel for BatteryModel {
    type ComponentState = BatteryState;
    type ComponentUpdate = BatteryUpdate;

    fn get_new_state(
        &mut self,
        state: &Self::ComponentState,
        update: Self::ComponentUpdate,
    ) -> Self::ComponentState {
        let bat_charge = state.capacity / self.quad_bat_capacity;
        let bat_voltage = f64::max(
            self.bat_voltage_curve.sample(1. - bat_charge) * self.quad_bat_cell_count as f64,
            0.1,
        );
        let power_factor_squared = f64::max(0., update.pwm_sum / 4.).powi(2);
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
        let currentm_as = f64::max(update.current_sum / 3.6, m_a_min);
        let capacity = state.capacity - currentm_as * update.dt;
        BatteryState {
            capacity,
            bat_voltage,
            bat_voltage_sag,
            amperage: currentm_as * 3.6,
            m_ah_drawn: self.quad_bat_capacity_charged - capacity,
        }
    }
}

type Battery = DroneComponent<BatteryModel>;

struct RotorState {
    pub pwm: f64, // pwm signal in percent [0,1]. Thiw will be
    // updated by BF
    pub temp: f64,     // motor core temp in deg C
    pub current: f64,  // current running through motor in Amps
    pub rpm: f64,      // motor revolutions per minute
    pub thrust: f64,   // thrust output of motor / propeller combo
    pub m_torque: f64, // motor torque
    pub p_torque: f64, // propeller torque, counter acting motor torque
}

#[derive(Debug, Clone)]
struct RotorModel {
    pub position: Vector3<f64>, // position relative to the main body
    pub prop_max_rpm: f64,
    pub prop_a_factor: f64,
    pub prop_torque_factor: f64,
    pub prop_inertia: f64,
    pub prop_thrust_factor: Vector3<f64>,
    pub motor_kv: f64, // kv
    pub motor_r: f64,  // resistence
    pub motor_io: f64, // idle current
    pub motor_dir: f64,
    pub pwm_low_pass_filter: LowPassFilter, // low pass filtered pwm value
}

impl RotorModel {
    fn prop_thrust(&self, vel_up: f64, rpm: f64) -> f64 {
        let prop_f = self.prop_thrust_factor[0] * vel_up * vel_up
            + self.prop_thrust_factor[1] * vel_up
            + self.prop_thrust_factor[2];
        let max_rpm = self.prop_max_rpm;
        let prop_a = self.prop_a_factor;
        // let rpm = self.rpm;
        let b = (prop_f - prop_a * max_rpm * max_rpm) / max_rpm;
        let result = b * rpm + prop_a * rpm * rpm;
        f64::max(result, 0.0)
    }

    // calculate the motor torque based on the motor torque constant
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
}

struct RotorUpdate {
    dt: f64,
    vbat: f64,
    rotation: Rotation3<f64>,
    linear_velocity_dir: Option<Vector3<f64>>,
    speed_factor: f64,
    vel_up: f64,
}

impl ComponentModel for RotorModel {
    type ComponentState = RotorState;
    type ComponentUpdate = RotorUpdate;

    fn get_new_state(
        &mut self,
        state: &Self::ComponentState,
        update: Self::ComponentUpdate,
    ) -> Self::ComponentState {
        let armature_volts =
            self.pwm_low_pass_filter.update(state.pwm, update.dt, 120.0) * update.vbat;
        let motor_torque = self.motor_torque(armature_volts, state.rpm);
        // This is the initial propeller thrust
        let propeller_thrust = self.prop_thrust(update.vel_up, state.rpm);
        let propeller_torque = propeller_thrust * self.prop_torque_factor;
        let domega = (motor_torque - propeller_torque) / self.prop_inertia;
        let drpm = (domega * update.dt) * 60.0 / (2.0 * PI);

        todo!()
    }
}

// // TODO: what is the difference between full_capacity and quad_bat_capacity_charged?
// #[derive(Debug, Clone)]
// struct BatteryProps {
//     //battery capacacity rating
//     pub quad_bat_capacity: f64, // mAH I guess
//     pub bat_voltage_curve: SampleCurve,
//     pub quad_bat_cell_count: u8,
//     //charged up capacity, can be lower or higher than quadBatCapacity
//     pub quad_bat_capacity_charged: f64,
//     pub max_voltage_sag: f64,
// }
//
// #[derive(Debug, Clone, Default)]
// pub struct BatteryState {
//     pub capacity: f64,
//     pub bat_voltage: f64,
//     pub bat_voltage_sag: f64,
//     pub amperage: f64,
//     pub m_ah_drawn: f64,
// }
//
// pub type Battery = DroneComponent<BatteryState, BatteryProps>;

// impl Battery {
//     pub fn update(&mut self, dt: f64, pwm_sum: f64, current_sum: f64) {
//         let bat_charge = self.state.capacity / self.model.quad_bat_capacity;
//         self.state.bat_voltage = f64::max(
//             self.model.bat_voltage_curve.sample(1. - bat_charge)
//                 * self.model.quad_bat_cell_count as f64,
//             0.1,
//         );
//         let power_factor_squared = f64::max(0., pwm_sum / 4.).powi(2);
//         let charge_factor_inv =
//             1.0 - (self.state.capacity / f64::max(self.model.quad_bat_capacity_charged, 1.));
//
//         let v_sag = self.model.max_voltage_sag * power_factor_squared
//             + (self.model.max_voltage_sag
//                 * charge_factor_inv
//                 * charge_factor_inv
//                 * power_factor_squared);
//         self.state.bat_voltage_sag = f64::clamp(
//             self.state.bat_voltage - v_sag - rng_gen_range(-0.01..0.01),
//             0.0,
//             100.,
//         );
//         let m_a_min = f64::min(0.2, rng_gen_range(-0.125..0.375))
//             / f64::max(self.state.bat_voltage_sag, 0.01);
//         let currentm_as = f64::max(current_sum / 3.6, m_a_min);
//         self.state.amperage = currentm_as * 3.6;
//         self.state.capacity -= currentm_as * dt;
//         self.state.m_ah_drawn = self.model.quad_bat_capacity_charged - self.state.capacity;
//     }
// }
//
// struct RotorProps {
//     pub position: Vector3<f64>, // position relative to the main body
//     pub prop_max_rpm: f64,
//     pub prop_a_factor: f64,
//     pub prop_torque_factor: f64,
//     pub prop_inertia: f64,
//     pub prop_thrust_factor: Vector3<f64>,
//     pub motor_kv: f64, // kv
//     pub motor_r: f64,  // resistence
//     pub motor_io: f64, // idle current
//     pub motor_dir: f64,
// }
//
// struct RotorState {
//     pub pwm: f64, // pwm signal in percent [0,1]. Thiw will be
//     // updated by BF
//     pub pwm_low_pass_filter: LowPassFilter, // low pass filtered pwm value
//     pub temp: f64,                          // motor core temp in deg C
//     pub current: f64,                       // current running through motor in Amps
//     pub rpm: f64,                           // motor revolutions per minute
//     pub thrust: f64,                        // thrust output of motor / propeller combo
//     pub m_torque: f64,                      // motor torque
//     pub p_torque: f64,                      // propeller torque, counter acting motor torque
// }
//
// pub type Rotor = DroneComponent<RotorState, RotorProps>;
//
// impl Rotor {
//     // https://en.wikipedia.org/wiki/Motor_constants#Motor_torque_constant
//     fn motor_torque(&self, armature_volts: f64) -> f64 {
//         let kv = self.model.motor_kv;
//         let back_emf_v = self.state.rpm / kv;
//         let base_current = (armature_volts - back_emf_v) / self.model.motor_r;
//         let armature_current = if base_current > 0. {
//             f64::max(0., base_current - self.model.motor_io)
//         } else {
//             f64::min(0., base_current + self.model.motor_io)
//         };
//         let torque_constant = 8.3 / kv; // why do we need to calculate this?
//         armature_current * torque_constant
//     }
//
//     fn prop_thrust(&self, vel_up: f64) -> f64 {
//         let prop_f = self.model.prop_thrust_factor[0] * vel_up * vel_up
//             + self.model.prop_thrust_factor[1] * vel_up
//             + self.model.prop_thrust_factor[2];
//         let max_rpm = self.model.prop_max_rpm;
//         let prop_a = self.model.prop_a_factor;
//         let rpm = self.state.rpm;
//         let b = (prop_f - prop_a * max_rpm * max_rpm) / max_rpm;
//         let result = b * rpm + prop_a * rpm * rpm;
//         f64::max(result, 0.0)
//     }
//
//     fn motor_thrust() {}
//
//     fn update(
//         &mut self,
//         dt: f64,
//         vbat: f64,
//         rotation: Rotation3<f64>,
//         linear_velocity_dir: Option<Vector3<f64>>,
//         speed_factor: f64,
//     ) {
//         // The effective voltage that the armature receives
//         let armature_volts = self
//             .state
//             .pwm_low_pass_filter
//             .update(self.state.pwm, dt, 120.0)
//             * vbat;
//         // The torque generated by the motor
//         let motor_torque = self.motor_torque(armature_volts);
//         // The torque generated by the propeller
//     }
// }
