use crate::ridge::RidgeRegression;
use crate::ModelInput;
use nalgebra::DMatrix;
use nalgebra::DVector;

pub enum RepresentationType {
    LastState,
    Output(f64),
}

pub trait Repr {
    fn repr(&mut self, input: ModelInput, res_states: Vec<DMatrix<f64>>) -> DMatrix<f64>;
}

pub struct OutputRepr {
    embedding: RidgeRegression,
}

impl Repr for OutputRepr {
    fn repr(&mut self, input: ModelInput, res_states: Vec<DMatrix<f64>>) -> DMatrix<f64> {
        let mut coeff_tr = vec![];
        let mut biases_tr = vec![];

        for (x, res_state) in input.inputs.iter().zip(res_states) {
            let (coeff, intercept) = self.embedding.fit_multiple(
                res_state.rows(0, input.time - 1).into_owned(),
                x.rows(1, input.time - 1).into_owned(),
            );
            coeff_tr.push(DVector::from(coeff.as_slice().to_vec()).transpose());
            biases_tr.push(intercept.transpose());
        }
        let coeff_tr = DMatrix::from_rows(&coeff_tr);
        let biases_tr = DMatrix::from_rows(&biases_tr);

        DMatrix::from_fn(
            coeff_tr.nrows(),
            coeff_tr.ncols() + biases_tr.ncols(),
            |r, c| {
                if c < coeff_tr.ncols() {
                    coeff_tr[(r, c)]
                } else {
                    biases_tr[(r, c - coeff_tr.ncols())]
                }
            },
        )
    }
}

impl OutputRepr {
    pub fn new(alpha: f64) -> Self {
        Self {
            embedding: RidgeRegression::new(alpha),
        }
    }
}

pub struct LastStateRepr;

impl Repr for LastStateRepr {
    fn repr(&mut self, input: ModelInput, res_states: Vec<DMatrix<f64>>) -> DMatrix<f64> {
        let last_states = res_states
            .iter()
            .map(|ep| ep.row(input.time - 1))
            .collect::<Vec<_>>();
        DMatrix::from_rows(&last_states)
    }
}

impl LastStateRepr {
    pub fn new() -> Self {
        Self {}
    }
}
