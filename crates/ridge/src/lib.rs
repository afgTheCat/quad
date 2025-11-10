pub mod ridge2;

use nalgebra::{DMatrix, DVector, RawStorage, SVD};

#[derive(Debug)]
pub struct RidgeRegressionSol {
    pub coeff: DMatrix<f64>,
    pub intercept: DVector<f64>,
}

// Super basic ridge regression model using svd
#[derive(Debug)]
pub struct RidgeRegression {
    pub alpha: f64,
    pub sol: Option<RidgeRegressionSol>,
}

impl RidgeRegression {
    pub fn new(alpha: f64) -> Self {
        Self { alpha, sol: None }
    }

    pub fn fit_svd(
        &mut self,
        x: &DMatrix<f64>,
        y: &DVector<f64>,
        save: bool,
    ) -> (DVector<f64>, f64) {
        let y_mean = y.mean();
        let x_mean = DVector::from(x.column_iter().map(|col| col.mean()).collect::<Vec<_>>());

        let y_centered = y.map(|x| x - y_mean);
        let mut x_centered = x.clone();
        for (i, mut col) in x_centered.column_iter_mut().enumerate() {
            col.add_scalar_mut(-x_mean[i]);
        }

        let SVD {
            u,
            v_t,
            singular_values,
        } = SVD::new(x_centered, true, true);

        let d = singular_values.map(|x| x / (x.powi(2) + self.alpha));
        let coeff = v_t.unwrap().transpose()
            * d.zip_map(&(u.unwrap().transpose() * y_centered), |x, y| x * y);
        let intercept = y_mean - x_mean.dot(&coeff);

        if save {
            self.sol = Some(RidgeRegressionSol {
                coeff: DMatrix::from_rows(&[coeff.transpose()]),
                intercept: DVector::from_element(1, intercept),
            })
        }

        (coeff, intercept)
    }

    pub fn fit_on_decomposed_svd(
        &mut self,
        u: &DMatrix<f64>,
        v_t: &DMatrix<f64>,
        d: &DVector<f64>,
        mut y: DVector<f64>,
        x_mean: &DVector<f64>,
        save: bool,
    ) -> (DVector<f64>, f64) {
        let y_mean = y.mean();
        for elem in y.iter_mut() {
            *elem -= -y_mean
        }

        let coeff = v_t.transpose() * d.zip_map(&(u.transpose() * y), |x, y| x * y);
        let intercept = y_mean - x_mean.dot(&coeff);

        if save {
            self.sol = Some(RidgeRegressionSol {
                coeff: DMatrix::from_rows(&[coeff.transpose()]),
                intercept: DVector::from_element(1, intercept),
            })
        }

        (coeff, intercept)
    }

    pub fn fit_multiple_svd(
        &mut self,
        mut x: DMatrix<f64>,
        y: &DMatrix<f64>,
    ) -> (DMatrix<f64>, DVector<f64>) {
        let (_, data_features) = x.data.shape();
        let (_, target_dim) = y.data.shape();
        let mut coeff_mult: DMatrix<f64> = DMatrix::zeros(target_dim.0, data_features.0);
        let mut intercept_mult: DVector<f64> = DVector::zeros(target_dim.0);

        let x_mean = DVector::from(x.column_iter().map(|col| col.mean()).collect::<Vec<_>>());
        for (i, mut col) in x.column_iter_mut().enumerate() {
            col.add_scalar_mut(-x_mean[i]);
        }

        let SVD {
            u,
            v_t,
            singular_values,
        } = SVD::new(x, true, true);

        let d = singular_values.map(|sig| sig / (sig.powi(2) + self.alpha));
        let v_t = v_t.unwrap();
        let u = u.unwrap();

        for (i, col) in y.column_iter().enumerate() {
            let (coeff, intercept) =
                self.fit_on_decomposed_svd(&u, &v_t, &d, col.into(), &x_mean, false);
            coeff_mult.set_row(i, &coeff.transpose());
            intercept_mult[i] = intercept;
        }
        self.sol = Some(RidgeRegressionSol {
            coeff: coeff_mult.clone(),
            intercept: intercept_mult.clone(),
        });
        (coeff_mult, intercept_mult)
    }

    // makes a prediction
    pub fn predict(&self, x: DMatrix<f64>) -> DMatrix<f64> {
        let Some(RidgeRegressionSol { coeff, intercept }) = self.sol.as_ref() else {
            panic!()
        };

        DMatrix::from_rows(
            &x.row_iter()
                .map(|data_point| {
                    DVector::from(
                        intercept
                            .iter()
                            .zip(coeff.row_iter())
                            .map(|(ic, c)| ic + data_point.dot(&c))
                            .collect::<Vec<_>>(),
                    )
                    .transpose()
                })
                .collect::<Vec<_>>(),
        )
    }
}
