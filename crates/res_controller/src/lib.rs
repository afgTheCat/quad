use db_common::{DBFlightLog, DBNewFlightLog};
use nalgebra::{DMatrix, DVector};
use res::{drone::DroneRc, input::FlightInput, representation::RepresentationType};
use ridge::RidgeRegression;
use sim_context::{LoaderType, SimContext};

pub fn db_fl_to_rc_output(fl: &DBFlightLog) -> DVector<f64> {
    DVector::from_row_slice(&[
        fl.motor_input_1,
        fl.motor_input_2,
        fl.motor_input_3,
        fl.motor_input_4,
    ])
}

pub fn train_thing(replay_id: &str) {
    let mut sim_context = SimContext::default();
    let flight_log = sim_context.loader.load_replay(replay_id);
    let mut drone_rc = DroneRc::new(
        500,
        0.3,
        0.99,
        0.2,
        RepresentationType::Output(1.),
        RidgeRegression::new(1.),
    );
    drone_rc.esn.set_input_weights(18);
    let input = FlightInput::new_from_db_fl_log(vec![flight_log.clone()]);
    let data_points = DMatrix::from_columns(
        &flight_log
            .iter()
            .map(db_fl_to_rc_output)
            .collect::<Vec<_>>(),
    )
    .transpose();

    drone_rc.fit(Box::new(input.clone()), data_points);
    let db_data = drone_rc.to_new_db("only_up".into());
    sim_context.insert_reservoir(db_data);

    let drone_rc_db = sim_context.select_reservoir("only_up_3");
    let mut new_rc_mode = DroneRc::from_db(drone_rc_db);
    let predicted_points = new_rc_mode.predict(Box::new(input));

    let mut rec_flight_logs = vec![];

    // TODO This is fake, what would be better is to launch a simulation with the controller.
    for (i, motor_outputs) in predicted_points.row_iter().enumerate() {
        let fl = DBNewFlightLog {
            simulation_id: "fake_simulation".into(),
            start_seconds: flight_log[i].start_seconds,
            end_seconds: flight_log[i].end_seconds,
            motor_input_1: *motor_outputs.get(0).unwrap(),
            motor_input_2: *motor_outputs.get(1).unwrap(),
            motor_input_3: *motor_outputs.get(2).unwrap(),
            motor_input_4: *motor_outputs.get(3).unwrap(),
            battery_voltage_sag: flight_log[i].battery_voltage_sag,
            battery_voltage: flight_log[i].battery_voltage,
            amperage: flight_log[i].amperage,
            mah_drawn: flight_log[i].mah_drawn,
            cell_count: flight_log[i].cell_count,
            rot_quat_x: flight_log[i].rot_quat_x,
            rot_quat_y: flight_log[i].rot_quat_y,
            rot_quat_z: flight_log[i].rot_quat_z,
            rot_quat_w: flight_log[i].rot_quat_w,
            linear_acceleration_x: flight_log[i].linear_acceleration_x,
            linear_acceleration_y: flight_log[i].linear_acceleration_y,
            linear_acceleration_z: flight_log[i].linear_acceleration_z,
            angular_velocity_x: flight_log[i].angular_velocity_x,
            angular_velocity_y: flight_log[i].angular_velocity_y,
            angular_velocity_z: flight_log[i].angular_velocity_z,
            throttle: flight_log[i].throttle,
            roll: flight_log[i].roll,
            pitch: flight_log[i].pitch,
            yaw: flight_log[i].yaw,
        };
        rec_flight_logs.push(fl);
    }

    sim_context.insert_logs("rec", rec_flight_logs);
}

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
