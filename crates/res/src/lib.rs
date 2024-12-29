pub mod drone;
pub mod esn;
pub mod input;
pub mod izhikevich;
pub mod readout;
pub mod representation;
mod ridge;

use matfile::{Array, NumericData};
use nalgebra::DMatrix;

fn extract_double(data: Option<&Array>) -> Vec<f64> {
    let data = data.unwrap();
    let NumericData::Double { real, .. } = data.data() else {
        panic!()
    };
    real.clone()
}
// kinda retarded but whatever
fn one_hot_encode(input: Vec<f64>) -> DMatrix<f64> {
    let categories = input.iter().map(|i| i.round() as u64).collect::<Vec<_>>();
    let max_category = *categories.iter().max().unwrap();
    let mut encoded: DMatrix<f64> = DMatrix::zeros(categories.len(), max_category as usize);
    for (i, c) in categories.iter().enumerate() {
        encoded.row_mut(i)[*c as usize - 1] = 1.;
    }
    return encoded;
}
