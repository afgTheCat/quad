use matfile::MatFile;
use nalgebra::DMatrix;
use ridge::{ridge2::ElasticNetWrapper, RidgeRegression};
use smartcore::metrics::{f1::F1, Metrics};

use crate::{
    esn::Esn,
    extract_double,
    input::{RcInput, TSInput},
    one_hot_encode,
    representation::{LastStateRepr, OutputRepr, Representation, RepresentationType},
};

pub struct RcModel {
    pub esn: Esn,
    representation: Representation,
    pub readout: RidgeRegression,
}

// Recresation of 'Reservoir computing approaches for representation and classification of multivariate time series'
impl RcModel {
    pub fn new(
        n_internal_units: usize,
        connectivity: f64,
        spectral_radius: f64,
        input_scaling: f64,
        representation: RepresentationType,
        readout: RidgeRegression,
    ) -> Self {
        let esn = Esn::new(
            n_internal_units,
            connectivity,
            spectral_radius,
            input_scaling,
        );
        let representation = match representation {
            RepresentationType::LastState => Representation::LastState(LastStateRepr::default()),
            RepresentationType::Output(alpha) => Representation::Output(OutputRepr::new(alpha)),
        };
        Self {
            esn,
            representation,
            readout,
        }
    }

    pub fn fit(&mut self, input: Box<dyn RcInput>, categories: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        // This is going to create create a linear model that is capable of predicting the next
        // input based on the reseroirs current state. Actually what we have here are flattened
        // linear models for all 270 training step!
        let input_repr = self.representation.repr(input, res_states);
        // Next we create a mapping from the representation to the categories. Why though? I have
        // no idea! I am not sure if this is ok tbh!
        self.readout.fit_multiple_svd(input_repr, &categories);
    }

    pub fn predict(&mut self, input: Box<dyn RcInput>) -> Vec<usize> {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        let logits = self.readout.predict(input_repr);
        logits
            .row_iter()
            .map(|row| row.transpose().argmax().0)
            .collect::<Vec<_>>()
    }
}

pub struct RcModel2 {
    pub esn: Esn,
    representation: Representation,
    pub readout: ElasticNetWrapper,
}

impl RcModel2 {
    pub fn new(
        n_internal_units: usize,
        connectivity: f64,
        spectral_radius: f64,
        input_scaling: f64,
        representation: RepresentationType,
        readout: ElasticNetWrapper,
    ) -> Self {
        let esn = Esn::new(
            n_internal_units,
            connectivity,
            spectral_radius,
            input_scaling,
        );
        let representation = match representation {
            RepresentationType::LastState => Representation::LastState(LastStateRepr::default()),
            RepresentationType::Output(alpha) => Representation::Output(OutputRepr::new(alpha)),
        };
        Self {
            esn,
            representation,
            readout,
        }
    }

    pub fn fit(&mut self, input: Box<dyn RcInput>, categories: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.fit_multiple(&input_repr, &categories);
    }

    pub fn predict(&mut self, input: Box<dyn RcInput>) -> Vec<usize> {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        let logits = self.readout.predict(input_repr);
        logits
            .row_iter()
            .map(|row| row.transpose().argmax().0)
            .collect::<Vec<_>>()
    }
}

// I guess we should have this a bit better
fn fit_and_predict(model: &mut RcModel, mat_file: MatFile) {
    let xtr = TSInput::from_mat_array(mat_file.find_by_name("X").unwrap());
    let ytr = one_hot_encode(extract_double(mat_file.find_by_name("Y")));

    let xte = TSInput::from_mat_array(mat_file.find_by_name("Xte").unwrap());
    let yte = extract_double(mat_file.find_by_name("Yte"));

    model.esn.set_input_weights(xtr.vars);
    model.fit(Box::new(xtr), ytr);
    let pred = model
        .predict(Box::new(xte))
        .iter()
        .map(|x| *x as f64 + 1.)
        .collect::<Vec<_>>();
    let f1 = F1::new_with(1.).get_score(&yte, &pred);
    println!("f1: {f1:?}");
}

fn fit_and_predict2(model: &mut RcModel2, mat_file: MatFile) {
    let xtr = TSInput::from_mat_array(mat_file.find_by_name("X").unwrap());
    let ytr = one_hot_encode(extract_double(mat_file.find_by_name("Y")));

    let xte = TSInput::from_mat_array(mat_file.find_by_name("Xte").unwrap());
    let yte = extract_double(mat_file.find_by_name("Yte"));

    model.esn.set_input_weights(xtr.vars);
    model.fit(Box::new(xtr), ytr);
    let pred = model
        .predict(Box::new(xte))
        .iter()
        .map(|x| *x as f64 + 1.)
        .collect::<Vec<_>>();
    let f1 = F1::new_with(1.).get_score(&yte, &pred);
    println!("f1: {f1:?}");
}

#[cfg(test)]
mod test {
    use ridge::{ridge2::ElasticNetWrapper, RidgeRegression};

    use super::{fit_and_predict, fit_and_predict2, RcModel};
    use crate::{representation::RepresentationType, verification::RcModel2};

    // This example reproduces the classification example from the Multivariate classification
    // example. Ridge regression is also implemented by hand as I do not trust the already existing
    // implementations
    #[test]
    fn reproduce_test1() {
        let file = std::fs::File::open("/home/gabor/projects/quad/data/JpVow.mat").unwrap();
        let mat_file = matfile::MatFile::parse(file).unwrap();

        let mut rc_model = RcModel::new(
            500,
            0.3,
            0.99,
            0.2,
            RepresentationType::Output(1.),
            RidgeRegression::new(1.),
        );

        fit_and_predict(&mut rc_model, mat_file);
    }

    #[test]
    fn reproduce_test2() {
        let file = std::fs::File::open("/home/gabor/projects/quad/data/JpVow.mat").unwrap();
        let mat_file = matfile::MatFile::parse(file).unwrap();

        let mut rc_model = RcModel2::new(
            500,
            0.3,
            0.99,
            0.2,
            RepresentationType::Output(1.),
            ElasticNetWrapper::new_ridge(1.),
        );
        fit_and_predict2(&mut rc_model, mat_file);
    }
}
