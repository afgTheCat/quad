use nalgebra::DMatrix;

pub trait RcInput {
    // number of episodes, time steps and vars
    fn shape(&self) -> (usize, usize, usize);
    // input at time t
    fn input_at_time(&self, t: usize) -> DMatrix<f64>;
}
