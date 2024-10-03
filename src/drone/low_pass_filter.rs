use crate::constants::M_PI;
use std::cell::RefCell;

#[derive(Debug, Clone)]
pub struct LowPassFilter {
    output: RefCell<f64>,
    e_pow: RefCell<f64>,
}

impl LowPassFilter {
    pub fn new(output: f64, e_pow: f64) -> Self {
        Self {
            output: RefCell::new(output),
            e_pow: RefCell::new(e_pow),
        }
    }

    pub fn update(&self, input: f64, delta_time: f64, cutoff_frequency: f64) -> f64 {
        self.e_pow
            .replace(1.0 - f64::exp(-delta_time * 2.0 * M_PI * cutoff_frequency));
        self.output
            .replace_with(|output| (input - *output) * *self.e_pow.borrow());
        *self.output.borrow()
    }
}
