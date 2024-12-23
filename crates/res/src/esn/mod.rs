pub mod rc;
pub mod reservoir;
use nalgebra::DMatrix;

pub struct ModelInput {
    pub episodes: usize,
    pub time: usize,
    pub vars: usize,
    pub inputs: Vec<DMatrix<f64>>,
}

impl ModelInput {
    // return dim: [EPISIDES, VARS]
    fn input_at_time(&self, t: usize) -> DMatrix<f64> {
        let mut input_at_t: DMatrix<f64> = DMatrix::zeros(self.episodes, self.vars);
        for (i, ep) in self.inputs.iter().enumerate() {
            input_at_t.set_row(i, &ep.row(t));
        }
        input_at_t
    }
}

#[cfg(test)]
mod test {
    use super::ModelInput;
    use crate::{esn::rc::RcModel, ridge::RidgeRegression};
    use matfile::{Array, NumericData};
    use nalgebra::DMatrix;

    fn extract_double(data: Option<&Array>) -> Vec<f64> {
        let data = data.unwrap();
        let size = data.size();
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

        let inputs = (0..size[0])
            .map(|ep| {
                DMatrix::from_row_slice(
                    size[1],
                    size[2],
                    &real[ep * size[1] * size[2]..(ep + 1) * size[1] * size[2]],
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

    // This example reproduces the classification example from the Multivariate classification
    // example. Ridge regression is also implemented by hand as I do not trust the already existing
    // implementations
    #[test]
    fn reproduce_test() {
        let file = std::fs::File::open("/home/gaborfeher/ascent/quad/data/JpVow.mat").unwrap();
        let mat_file = matfile::MatFile::parse(file).unwrap();

        // [T]
        let Xtr = extract_model_input(mat_file.find_by_name("X"));
        let Ytr = extract_double(mat_file.find_by_name("Y"));

        let Xte = extract_model_input(mat_file.find_by_name("Xte"));
        let Yte = extract_double(mat_file.find_by_name("Yte"));

        let mut rc_model = RcModel::new(
            500,
            0.3,
            0.99,
            0.2,
            RidgeRegression::new(1.),
            RidgeRegression::new(1.),
        );

        rc_model.esn_model.set_input_weights(&Xtr);

        let Ytr = one_hot_encode(Ytr);
        let Yte = one_hot_encode(Yte);

        rc_model.fit(Xtr, Ytr);

        let pred = rc_model.predict(Xte);
        println!("{pred:?}");

        // println!("{Yte:?}");
    }
}
