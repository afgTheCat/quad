use std::f64;

use super::sample_curve::SampleCurve;
use crate::rng_gen_range;

#[derive(Debug, Clone, Default)]
struct BatteryProps {
    full_capacity: f64,
    bat_voltage_curve: SampleCurve,
    quad_bat_cell_count: f64,
    quad_bat_capacity_charged: f64,
    max_voltage_sag: f64,
}

#[derive(Debug, Clone, Default)]
struct BatteryState {
    capacity: f64,
    bat_voltage: f64,
    bat_voltage_sag: f64,
    amperage: f64,
    m_ah_drawn: f64,
}

#[derive(Debug, Clone, Default)]
pub struct Battery {
    props: BatteryProps,
    state: BatteryState,
}

impl Battery {
    fn new(
        bat_voltage: f64,
        bat_voltage_sag: f64,
        bat_capacity: f64,
        amperage: f64,
        m_ah_drawn: f64,
        quad_bat_capacity: f64,
        bat_voltage_curve: SampleCurve,
        quad_bat_cell_count: f64, // TODO: should not be an f64
        quad_bat_capacity_charged: f64,
        max_voltage_sag: f64,
    ) -> Self {
        Self {
            props: BatteryProps {
                full_capacity: quad_bat_capacity,
                bat_voltage_curve,
                quad_bat_cell_count,
                quad_bat_capacity_charged,
                max_voltage_sag,
            },
            state: BatteryState {
                capacity: bat_capacity,
                bat_voltage,
                bat_voltage_sag,
                amperage,
                m_ah_drawn,
            },
        }
    }

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

    pub fn getBatVoltageSag(&self) -> f64 {
        self.state.bat_voltage_sag
    }

    pub fn getBatVoltage(&self) -> f64 {
        self.state.bat_voltage
    }

    pub fn amperage(&self) -> f64 {
        self.state.amperage
    }

    pub fn m_ah_drawn(&self) -> f64 {
        self.state.m_ah_drawn
    }
}
