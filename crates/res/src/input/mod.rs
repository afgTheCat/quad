use std::f64;

use db::simulation::DBFlightLog;
// use db::{FlightLog, FlightLogEvent};
use matfile::{Array, NumericData};
use nalgebra::{DMatrix, DVector};

pub trait RcInput {
    // number of episodes, time steps and vars
    fn shape(&self) -> (usize, usize, usize);
    // input at time t
    fn input_at_time(&self, t: usize) -> DMatrix<f64>;
    // inputs, probably should be an iterator
    fn inputs(&self) -> &Vec<DMatrix<f64>>;
}

pub struct TSInput {
    pub episodes: usize,
    pub time: usize,
    pub vars: usize,
    // per episode
    pub inputs: Vec<DMatrix<f64>>,
}

impl TSInput {
    pub fn truncate(&mut self) {
        self.episodes = 1;
        self.inputs = vec![self.inputs[0].clone()];
    }

    pub fn from_mat_array(data: &Array) -> Self {
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

        TSInput {
            episodes: size[0],
            time: size[1],
            vars: size[2],
            inputs,
        }
    }
}

impl RcInput for TSInput {
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

// The thing is that they are variable in size => each episode is going to be each time step
#[derive(Debug, Clone)]
pub struct FlightInput {
    episodes: usize,
    time: usize, // configure how many esns we want here OR just
    vars: usize, // I guess this can be something else
    data: Vec<DMatrix<f64>>,
}

pub fn db_fl_to_rc_input(fl: &DBFlightLog) -> DVector<f64> {
    DVector::from_row_slice(&[
        fl.battery_voltage_sag,
        fl.battery_voltage,
        fl.amperage,
        fl.mah_drawn,
        fl.rot_quat_x,
        fl.rot_quat_y,
        fl.rot_quat_z,
        fl.rot_quat_w,
        fl.linear_acceleration_x,
        fl.linear_acceleration_y,
        fl.linear_acceleration_z,
        fl.angular_velocity_x,
        fl.angular_velocity_y,
        fl.angular_velocity_z,
        fl.throttle,
        fl.roll,
        fl.yaw,
        fl.pitch,
    ])
}

impl FlightInput {
    pub fn new(flight_logs: Vec<Vec<DBFlightLog>>) -> Self {
        let episodes = flight_logs.len();
        let time = flight_logs.iter().map(|x| x.len()).max().unwrap();
        let data = flight_logs
            .iter()
            .map(|fl| {
                let columns = fl.iter().map(db_fl_to_rc_input).collect::<Vec<_>>();
                let m = DMatrix::from_columns(&columns).transpose();
                println!("{:?}", m.shape());
                m
            })
            .collect();
        Self {
            episodes,
            time,
            vars: 18, // TODO: do not hardcode in the future
            data,
        }
    }
}

impl RcInput for FlightInput {
    fn shape(&self) -> (usize, usize, usize) {
        (self.episodes, self.time, self.vars)
    }

    fn input_at_time(&self, t: usize) -> DMatrix<f64> {
        let mut input_at_t: DMatrix<f64> = DMatrix::zeros(self.episodes, self.vars);
        for (i, ep) in self.data.iter().enumerate() {
            input_at_t.set_row(i, &ep.row(t));
        }
        input_at_t
    }

    fn inputs(&self) -> &Vec<DMatrix<f64>> {
        &self.data
    }
}
