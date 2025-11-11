use linfa::dataset::DatasetBase;
use linfa::traits::Fit;
use linfa_elasticnet::{ElasticNetParams, MultiTaskElasticNetParams};
use nalgebra::{DMatrix, DVector, RawStorage};
use ndarray::{Array1, Array2, Axis};

#[derive(Debug, Clone)]
pub struct ENetSol {
    pub coeff: DMatrix<f64>,
    pub intercept: DVector<f64>,
}

#[derive(Debug, Clone)]
pub struct ElasticNetWrapper {
    pub penalty: f64,
    pub l1_ratio: f64,
    pub sol: Option<ENetSol>,
}

impl ElasticNetWrapper {
    pub fn new_ridge(alpha: f64) -> Self {
        Self {
            penalty: alpha,
            l1_ratio: 0.0,
            sol: None,
        }
    }

    pub fn new_elastic(penalty: f64, l1_ratio: f64) -> Self {
        Self {
            penalty,
            l1_ratio,
            sol: None,
        }
    }

    pub fn fit(&mut self, x: &DMatrix<f64>, y: &DVector<f64>, save: bool) -> (DVector<f64>, f64) {
        let (xa, ya) = (dmatrix_to_array2(x), dvector_to_array1(y));
        let ds = DatasetBase::from((xa, ya));

        let params = ElasticNetParams::new()
            .penalty(self.penalty)
            .l1_ratio(self.l1_ratio)
            .with_intercept(true);

        let fitted = params.fit(&ds).expect("elastic net fit failed");

        let w = fitted.hyperplane().to_owned(); // Array1<f64>
        let b = fitted.intercept(); // f64

        let coeff = DVector::from_row_slice(w.as_slice().expect("contiguous"));
        let intercept = b;

        if save {
            self.sol = Some(ENetSol {
                coeff: DMatrix::from_rows(&[coeff.transpose()]), // (1, features)
                intercept: DVector::from_element(1, intercept),  // (1,)
            });
        }

        (coeff, intercept)
    }

    pub fn fit_multiple(
        &mut self,
        x: &DMatrix<f64>,
        y: &DMatrix<f64>,
    ) -> (DMatrix<f64>, DVector<f64>) {
        let xa = dmatrix_to_array2(x);
        let ya = dmatrix_to_array2(y);

        let ds = DatasetBase::new(xa, ya);
        let params = MultiTaskElasticNetParams::new()
            .penalty(self.penalty)
            .l1_ratio(self.l1_ratio)
            .with_intercept(true);

        let fitted = params.fit(&ds).expect("multi-task elastic net fit failed");

        let w: Array2<f64> = fitted.hyperplane().to_owned();
        let b: Array1<f64> = fitted.intercept().to_owned();

        let coeff_nm = array2_to_dmatrix(&w.t().to_owned()); // transpose to (n_tasks, features)
        let intercept_nm = DVector::from_row_slice(b.as_slice().unwrap());

        self.sol = Some(ENetSol {
            coeff: coeff_nm.clone(),
            intercept: intercept_nm.clone(),
        });
        (coeff_nm, intercept_nm)
    }

    pub fn predict(&self, x: DMatrix<f64>) -> DMatrix<f64> {
        let ENetSol { coeff, intercept } = self.sol.as_ref().expect("model not fitted");
        let y = x * coeff.transpose(); // (n_samples, targets)
        let y = DMatrix::from_iterator(y.nrows(), y.ncols(), y.iter().cloned())
            .reshape_generic(y.data.shape().0, y.data.shape().1)
            .row_iter()
            .map(|row| {
                let mut r = row.transpose();
                r.iter_mut()
                    .enumerate()
                    .for_each(|(j, v)| *v += intercept[j]);
                r.transpose()
            })
            .collect::<Vec<_>>();

        DMatrix::from_rows(&y)
    }
}

fn dmatrix_to_array2(m: &DMatrix<f64>) -> Array2<f64> {
    let (r, c) = m.shape();
    let mut a = Array2::<f64>::zeros((r, c));
    for i in 0..r {
        for j in 0..c {
            a[[i, j]] = m[(i, j)];
        }
    }
    a
}

fn dvector_to_array1(v: &DVector<f64>) -> Array1<f64> {
    Array1::from_iter(v.iter().cloned())
}

fn array2_to_dmatrix(a: &Array2<f64>) -> DMatrix<f64> {
    let (r, c) = a.dim();
    let mut buf = Vec::with_capacity(r * c);
    for row in a.axis_iter(Axis(0)) {
        buf.extend(row.iter().copied());
    }
    DMatrix::from_row_slice(r, c, &buf)
}
