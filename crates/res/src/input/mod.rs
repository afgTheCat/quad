use nalgebra::DMatrix;

pub trait RcInput {
    // number of episodes, time steps and vars
    fn shape(&self) -> (usize, usize, usize);
    // input at time t
    fn input_at_time(&self, t: usize) -> DMatrix<f64>;
    // inputs, probably should be an iterator
    fn inputs(&self) -> &Vec<DMatrix<f64>>;
}

pub struct ModelInput {
    pub episodes: usize,
    pub time: usize,
    pub vars: usize,
    // per episode
    pub inputs: Vec<DMatrix<f64>>,
}

impl ModelInput {
    pub fn truncate(&mut self) {
        self.episodes = 1;
        self.inputs = vec![self.inputs[0].clone()];
    }
}

impl RcInput for ModelInput {
    fn shape(&self) -> (usize, usize, usize) {
        (self.episodes, self.time, self.vars)
    }

    fn input_at_time(&self, t: usize) -> DMatrix<f64> {
        let mut input_at_t: DMatrix<f64> = DMatrix::zeros(self.episodes, self.vars);
        for (i, ep) in self.inputs.iter().enumerate() {
            input_at_t.set_row(i, &ep.row(t));
        }
        input_at_t
    }

    fn inputs(&self) -> &Vec<DMatrix<f64>> {
        &self.inputs
    }
}
