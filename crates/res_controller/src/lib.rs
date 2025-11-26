use nalgebra::DMatrix;
use res::{
    input::RcInput,
    representation::{LastStateRepr, OutputRepr, Representation, RepresentationType},
    reservoir::Esn,
};
use ridge::RidgeRegression;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone)]
pub struct DroneRc {
    pub esn: Esn,
    pub representation: Representation,
    // Could be changed to ElasticNetWrapper
    pub readout: RidgeRegression,
}

impl DroneRc {
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

    // this is the same, maybe it works maybe it does not
    pub fn fit(&mut self, input: Box<dyn RcInput>, rc_data: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.fit_multiple_svd(input_repr, &rc_data);
    }

    // a blast from the past! we are basically fitting on the res states. In fact we are fitting on the
    // first res state. Let's see what it can do! Will this error? Will this not? I think I did
    // this when I did not understand what I am supposed to do!
    pub fn old_fit(&mut self, input: Box<dyn RcInput>, data_points: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        self.readout
            .fit_multiple_svd(res_states[0].clone(), &data_points);
    }

    pub fn predict(&mut self, input: Box<dyn RcInput>) -> DMatrix<f64> {
        let res_states = self.esn.compute_state_matricies(&input);
        self.readout.predict(res_states[0].clone())
    }
}

// pub fn db_fl_to_rc_output(fl: &DBFlightLog) -> DVector<f64> {
//     DVector::from_row_slice(&[
//         fl.motor_input_1,
//         fl.motor_input_2,
//         fl.motor_input_3,
//         fl.motor_input_4,
//     ])
// }
//
// pub fn snapshot_fl_input(snapshot: &SnapShot) -> DVector<f64> {
//     DVector::from_row_slice(&[
//         snapshot.battery_update.bat_voltage_sag,
//         snapshot.battery_update.bat_voltage,
//         snapshot.battery_update.amperage,
//         snapshot.battery_update.m_ah_drawn,
//         snapshot.gyro_update.rotation[0],
//         snapshot.gyro_update.rotation[1],
//         snapshot.gyro_update.rotation[2],
//         snapshot.gyro_update.rotation[3],
//         snapshot.gyro_update.linear_acc[0],
//         snapshot.gyro_update.linear_acc[1],
//         snapshot.gyro_update.linear_acc[2],
//         snapshot.gyro_update.angular_velocity[0],
//         snapshot.gyro_update.angular_velocity[1],
//         snapshot.gyro_update.angular_velocity[2],
//         // TODO: double check this
//         snapshot.motor_input[0], // snapshot.throttle,
//         snapshot.motor_input[1], // snapshot.roll,
//         snapshot.motor_input[2], // snapshot.yaw,
//         snapshot.motor_input[3], // snapshot.pitch,
//     ])
// }
//
// fn snapshots_to_flight_input(flight_logs: Vec<FlightLog>) -> FlightInput {
//     let episodes = flight_logs.len();
//     let time = flight_logs.iter().map(|x| x.steps.len()).max().unwrap();
//     let data = flight_logs
//         .iter()
//         .map(|fl| {
//             let columns = fl
//                 .steps
//                 .iter()
//                 .map(|f| snapshot_fl_input(f))
//                 .collect::<Vec<_>>();
//             let m = DMatrix::from_columns(&columns).transpose();
//             m
//         })
//         .collect();
//     FlightInput {
//         episodes,
//         time,
//         vars: 18, // TODO: do not hardcode in the future
//         data,
//     }
// }

// pub fn train_thing() {
//     let replay_id = "only_up";
//     let mut sim_context = SimContext::default();
//     // load from files
//     sim_context.set_loader(&sim_context::LoaderType::File);
//     sim_context.set_logger(sim_context::LoggerType::File);
//
//     let flight_log = sim_context.load_replay(replay_id);
//     let mut drone_rc = DroneRc::new(
//         500,
//         0.3,
//         0.99,
//         0.2,
//         RepresentationType::Output(1.),
//         RidgeRegression::new(1.),
//     );
//     drone_rc.esn.set_input_weights(18);
//     let input = snapshots_to_flight_input(vec![flight_log.clone()]);
//     let data_points = DMatrix::from_columns(
//         &flight_log
//             .steps
//             .iter()
//             .map(snapshot_fl_input)
//             .collect::<Vec<_>>(),
//     )
//     .transpose();
//
//     drone_rc.old_fit(Box::new(input.clone()), data_points);
//     // save the trained reservoir controller
//     sim_context.insert_drone_rc("trained_on_only_up", ResController::new(drone_rc.clone()));
//
//     // let drone_rc_db = sim_context.select_reservoir("only_up_2");
//     let new_rc_mode = sim_context.load_drone_rc("trained_on_only_up");
//     let predicted_points = new_rc_mode.model.lock().unwrap().predict(Box::new(input));
//     let mut rec_flight_logs = vec![];
//     for (i, motor_inputs) in predicted_points.row_iter().enumerate() {
//         let mut fl = flight_log.steps[i].clone();
//         println!("{}", *motor_inputs.get(0).unwrap());
//         println!("{}", *motor_inputs.get(1).unwrap());
//         println!("{}", *motor_inputs.get(2).unwrap());
//         println!("{}", *motor_inputs.get(3).unwrap());
//         println!("---------------------------------");
//         let motor_inputs: MotorInput = MotorInput {
//             input: [
//                 *motor_inputs.get(0).unwrap(),
//                 *motor_inputs.get(1).unwrap(),
//                 *motor_inputs.get(2).unwrap(),
//                 *motor_inputs.get(3).unwrap(),
//             ],
//         };
//         fl.motor_input = motor_inputs;
//         rec_flight_logs.push(fl);
//     }
//     sim_context.insert_logs(FlightLog::new(
//         "recalculated_inputs".into(),
//         rec_flight_logs,
//     ));
// }
//
// #[cfg(test)]
// mod test {
//     use crate::train_thing;
//
//     // resurrecting the old only up test!
//     #[test]
//     fn old_res_training_thing() {
//         train_thing();
//     }
// }

// TODO: we need to fix this!
// fn train_on_many() {
//     let db = AscentDb::new("/home/gabor/ascent/quad/data.sqlite");
//     let simulation_id = db.select_simulation_ids();
//     let tr_ids = simulation_id
//         .iter()
//         .filter(|id| id.starts_with("ds_id_1_tr"))
//         .cloned()
//         .collect::<Vec<_>>();
//
//     let mut drone_rc = DroneRc::new(
//         500,
//         0.3,
//         0.99,
//         0.2,
//         RepresentationType::Output(1.),
//         RidgeRegression::new(1.),
//     );
//     drone_rc.esn.set_input_weights(18);
//     for tr_id in tr_ids {
//         let flight_log = db.get_simulation_data(&tr_id);
//         let input = FlightInput::new_from_db_fl_log(vec![flight_log.clone()]);
//         let data_points = DMatrix::from_columns(
//             &flight_log.iter().map(db_fl_to_rc_input).collect::<Vec<_>>(),
//         )
//         .transpose();
//         drone_rc.fit(Box::new(input.clone()), data_points);
//     }
//
//     drone_rc.save_model_to_db("combined_2".into(), &db);
// }
