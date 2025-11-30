use crate::input::RcInput;
use nalgebra::DMatrix;
use nalgebra::DVector;
use nalgebra::RowDVector;
use ridge::RidgeRegression;
use serde::Deserialize;
use serde::Serialize;

pub enum RepresentationType {
    LastState,
    Output(f64),
    AllStates,
    BufferedStates(usize),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Representation {
    LastState(LastStateRepr),
    Output(OutputRepr),
    AllStateForSingle(AllStatesForSingleEp),
    BufferedStates(BufferedStatesForSingleEp),
}

impl Representation {
    pub fn repr(&mut self, input: Box<dyn RcInput>, res_states: Vec<DMatrix<f64>>) -> DMatrix<f64> {
        match self {
            Self::LastState(ls) => ls.repr(input, res_states),
            Self::Output(o) => o.repr(input, res_states),
            Self::AllStateForSingle(r) => r.repr(input, res_states),
            Self::BufferedStates(r) => r.repr(input, res_states),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OutputRepr {
    embedding: RidgeRegression,
}

impl OutputRepr {
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

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LastStateRepr;

impl LastStateRepr {
    fn repr(&mut self, input: Box<dyn RcInput>, res_states: Vec<DMatrix<f64>>) -> DMatrix<f64> {
        let (_, time_steps, _) = input.shape();
        let last_states = res_states
            .iter()
            .map(|ep| ep.row(time_steps - 1))
            .collect::<Vec<_>>();
        DMatrix::from_rows(&last_states)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AllStatesForSingleEp;

impl AllStatesForSingleEp {
    fn repr(&mut self, _input: Box<dyn RcInput>, res_states: Vec<DMatrix<f64>>) -> DMatrix<f64> {
        assert_eq!(res_states.len(), 1, "Learns from a single episode");
        res_states[0].clone()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BufferedStatesForSingleEp(pub usize);

impl BufferedStatesForSingleEp {
    fn repr(&mut self, input: Box<dyn RcInput>, res_states: Vec<DMatrix<f64>>) -> DMatrix<f64> {
        let lag = self.0;
        let (eps, time, n_units) = input.shape();

        // Each episode's features will be stacked
        let mut features_all_eps = Vec::new();

        for ep in 0..eps {
            let states = &res_states[ep]; // [time × n_units]
            let mut features = DMatrix::zeros(time, n_units * (lag + 1));

            for t in 0..time {
                for k in 0..=lag {
                    let src_t = if t >= k { t - k } else { 0 };
                    let dst_start = k * n_units;

                    features
                        .slice_mut((t, dst_start), (1, n_units))
                        .copy_from(&states.slice((src_t, 0), (1, n_units)));
                }
            }
            features_all_eps.push(features);
        }

        let mut all_rows: Vec<RowDVector<f64>> = Vec::new();

        for episode_matrix in features_all_eps {
            for row in episode_matrix.row_iter() {
                all_rows.push(row.clone().into()); // row is 1 × n, so RowDVector
            }
        }

        DMatrix::from_rows(&all_rows)
    }
}
