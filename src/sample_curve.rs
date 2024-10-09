fn interpolate(a: f64, b: f64, i: f64) -> f64 {
    a + ((b - a) * i)
}

#[derive(Debug, Clone)]
pub struct SamplePoint {
    pub discharge: f64,
    pub voltage: f64,
}

impl SamplePoint {
    pub fn new(capacity: f64, voltage: f64) -> Self {
        Self {
            discharge: capacity,
            voltage,
        }
    }
}

#[derive(Debug, Clone)]
pub struct SampleCurve {
    sample_points: Vec<SamplePoint>,
    min_discharge_point: SamplePoint,
    max_discharge_point: SamplePoint,
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

    // this is slow for no reason at all
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
                        Some(interpolate(sample.discharge, next_sample.discharge, factor))
                    } else {
                        None
                    }
                })
                .unwrap()
        }
    }
}
