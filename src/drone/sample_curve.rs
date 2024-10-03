use itertools::Itertools;
use std::cmp::Ordering;

fn interpolate(a: f64, b: f64, i: f64) -> f64 {
    a + ((b - a) * i)
}

#[derive(Debug, Clone)]
pub struct SamplePoint {
    i: f64,
    v: f64,
}

impl SamplePoint {
    fn new(i: f64, v: f64) -> Self {
        Self { i, v }
    }
}

#[derive(Debug, Clone)]
pub struct SampleCurve {
    sample_points: Vec<SamplePoint>,
}

impl SampleCurve {
    fn new(sample_points: Vec<SamplePoint>) -> Self {
        Self { sample_points }
    }

    pub fn sample(&self, i: f64) -> f64 {
        let last = self.sample_points.last().unwrap();
        let first = self.sample_points.first().unwrap();
        if i > last.i {
            last.v
        } else if i < first.i {
            first.v
        } else {
            let (first_sample, second_sample) = self
                .sample_points
                .iter()
                .tuple_windows()
                .find_map(|(sample, next_sample)| {
                    match (sample.i.total_cmp(&i), next_sample.i.total_cmp(&i)) {
                        (Ordering::Less, Ordering::Less) => None,
                        (Ordering::Less, Ordering::Equal | Ordering::Greater) => {
                            Some((sample, next_sample))
                        }
                        _ => unreachable!(),
                    }
                })
                .unwrap();
            let factor = (i - first_sample.i) / (second_sample.i - first_sample.i);
            interpolate(first_sample.v, second_sample.v, factor)
        }
    }
}
