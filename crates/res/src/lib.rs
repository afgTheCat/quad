pub mod drone;
pub mod esn;
pub mod input;
pub mod izhikevich;
pub mod readout;
pub mod representation;
mod ridge;

use input::ModelInput;
use matfile::{Array, NumericData};
use nalgebra::{DMatrix, DVector};

fn extract_double(data: Option<&Array>) -> Vec<f64> {
    let data = data.unwrap();
    let NumericData::Double { real, .. } = data.data() else {
        panic!()
    };
    real.clone()
}

fn extract_model_input(data: Option<&Array>) -> ModelInput {
    let data = data.unwrap();
    let size = data.size();
    let NumericData::Double { real, .. } = data.data() else {
        panic!()
    };

    let total_ep = size[0];
    let total_time = size[1];
    let total_vars = size[2];

    let inputs = (0..total_ep)
        .map(|ep| {
            DMatrix::from_rows(
                &(0..total_time)
                    .map(|t| {
                        DVector::from_iterator(
                            total_vars,
                            (0..total_vars)
                                .map(|v| real[v * total_ep * total_time + t * total_ep + ep]),
                        )
                        .transpose()
                    })
                    .collect::<Vec<_>>(),
            )
        })
        .collect::<Vec<_>>();

    ModelInput {
        episodes: size[0],
        time: size[1],
        vars: size[2],
        inputs,
    }
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
