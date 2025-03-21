use crate::input::RcInput;
use nalgebra::DMatrix;
use nalgebra::DVector;
use ridge::RidgeRegression;

pub enum RepresentationType {
    LastState,
    Output(f64),
}

// This should be sync and send for comfort
pub trait Repr: Sync + Send {
    fn repr(&mut self, input: Box<dyn RcInput>, res_states: Vec<DMatrix<f64>>) -> DMatrix<f64>;
}

pub struct OutputRepr {
    embedding: RidgeRegression,
}

impl Repr for OutputRepr {
    fn repr(&mut self, input: Box<dyn RcInput>, res_states: Vec<DMatrix<f64>>) -> DMatrix<f64> {
        let mut coeff_tr = vec![];
        let mut biases_tr = vec![];
        let (_, time_steps, _) = input.shape();

        for (x, res_state) in input.inputs().iter().zip(res_states) {
            let (coeff, intercept) = self.embedding.fit_multiple_svd(
                res_state.rows(0, time_steps - 1).into(),
                &x.rows(1, time_steps - 1).into(),
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

#[derive(Debug, Default)]
pub struct LastStateRepr;

impl Repr for LastStateRepr {
    fn repr(&mut self, input: Box<dyn RcInput>, res_states: Vec<DMatrix<f64>>) -> DMatrix<f64> {
        let (_, time_steps, _) = input.shape();
        let last_states = res_states
            .iter()
            .map(|ep| ep.row(time_steps - 1))
            .collect::<Vec<_>>();
        DMatrix::from_rows(&last_states)
    }
}
