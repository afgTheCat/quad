pub mod rc;
pub mod readout;
pub mod representation;
pub mod reservoir;

use matfile::{Array, NumericData};
use nalgebra::{DMatrix, DVector};

pub struct ModelInput {
    pub episodes: usize,
    pub time: usize,
    pub vars: usize,
    // per episode
    pub inputs: Vec<DMatrix<f64>>,
}

impl ModelInput {
    // return dim: [EPISODES, VARS]
    fn input_at_time(&self, t: usize) -> DMatrix<f64> {
        let mut input_at_t: DMatrix<f64> = DMatrix::zeros(self.episodes, self.vars);
        for (i, ep) in self.inputs.iter().enumerate() {
            input_at_t.set_row(i, &ep.row(t));
        }
        input_at_t
    }

    pub fn truncate(&mut self) {
        self.episodes = 1;
        self.inputs = vec![self.inputs[0].clone()];
    }
}

pub trait RcInput {
    // number of episodes, time steps and vars
    fn shape(&self) -> (usize, usize, usize);
    // input at time t
    fn input_at_time(&self, t: usize) -> DMatrix<f64>;
}

struct ReservoirStates {
    pub states: Vec<DMatrix<f64>>,
    pub time: usize,
}

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

#[cfg(test)]
mod test {
    use crate::{
        esn::{extract_double, extract_model_input, one_hot_encode, rc::RcModel},
        ridge::RidgeRegression,
    };
    use smartcore::metrics::{f1::F1, Metrics};

    // This example reproduces the classification example from the Multivariate classification
    // example. Ridge regression is also implemented by hand as I do not trust the already existing
    // implementations
    #[test]
    fn reproduce_test() {
        let file = std::fs::File::open("/Users/afgthecat/projects/quad/data/JpVow.mat").unwrap();
        let mat_file = matfile::MatFile::parse(file).unwrap();

        // [T]
        let mut Xtr = extract_model_input(mat_file.find_by_name("X"));
        // NOTE:
        Xtr.truncate();
        let Ytr = extract_double(mat_file.find_by_name("Y"));

        let Xte = extract_model_input(mat_file.find_by_name("Xte"));
        let Yte = extract_double(mat_file.find_by_name("Yte"));

        let mut rc_model = RcModel::new(500, 0.3, 0.99, 0.2, 1., RidgeRegression::new(1.));

        rc_model.esn.set_input_weights(Xtr.vars);

        let Ytr = one_hot_encode(Ytr);

        rc_model.fit(Xtr, Ytr);
        // let pred = rc_model
        //     .predict(Xte)
        //     .iter()
        //     .map(|x| *x as f64 + 1.)
        //     .collect::<Vec<_>>();
        // let f1 = F1::new_with(1.).get_score(&Yte, &pred);
        // println!("f1: {f1:?}");
    }
}
