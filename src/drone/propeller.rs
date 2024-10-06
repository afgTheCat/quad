use bevy::math::DVec3;

#[derive(Debug, Clone)]
pub struct Propeller {
    prop_max_rpm: f64,
    prop_a_factor: f64,
    prop_torque_factor: f64,
    prop_inertia: f64,
    prop_thrust_factor: DVec3,
}

impl Default for Propeller {
    fn default() -> Self {
        Self {
            prop_max_rpm: 1.,
            prop_a_factor: 0.,
            prop_torque_factor: 0.,
            prop_inertia: 0.1,
            prop_thrust_factor: DVec3::ZERO,
        }
    }
}

impl Propeller {
    pub fn new(
        prop_max_rpm: f64,
        prop_a_factor: f64,
        prop_torque_factor: f64,
        prop_inertia: f64,
        prop_thrust_factor: DVec3,
    ) -> Self {
        Self {
            prop_max_rpm,
            prop_a_factor,
            prop_torque_factor,
            prop_inertia,
            prop_thrust_factor,
        }
    }

    // the thrust a propeller does
    pub fn prop_thrust(&self, vel_up: f64, rpm: f64) -> f64 {
        let prop_f = f64::max(
            0.0,
            self.prop_thrust_factor[0] * vel_up * vel_up
                + self.prop_thrust_factor[1] * vel_up
                + self.prop_thrust_factor[2],
        );
        let max_rpm = f64::max(self.prop_max_rpm, 0.01);
        let prop_a = self.prop_a_factor;
        let b = (prop_f - prop_a * max_rpm * max_rpm) / max_rpm;
        let result = b * rpm + prop_a * rpm * rpm;
        f64::max(result, 0.0)
    }

    pub fn prop_torque(&self, vel_up: f64, rpm: f64) -> f64 {
        self.prop_thrust(vel_up, rpm) * self.prop_torque_factor
    }
}

impl Propeller {
    pub fn inertia(&self) -> f64 {
        self.prop_inertia
    }
}
