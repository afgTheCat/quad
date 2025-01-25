use std::f64::consts::PI;

#[derive(Debug, Clone, Default)]
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
