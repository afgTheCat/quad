use db_common::DBFlightLog;
use flight_controller::{MotorInput, controllers::res_controller::ResController};
use loggers::{FlightLog, SnapShot};
use nalgebra::{DMatrix, DVector};
use res::{input::FlightInput, representation::RepresentationType};
use res_controller::DroneRc;
use ridge::RidgeRegression;
use sim_context::SimContext;

pub fn db_fl_to_rc_output(fl: &DBFlightLog) -> DVector<f64> {
    DVector::from_row_slice(&[
        fl.motor_input_1,
        fl.motor_input_2,
        fl.motor_input_3,
        fl.motor_input_4,
    ])
}

pub fn snapshot_fl_input(snapshot: &SnapShot) -> DVector<f64> {
    DVector::from_row_slice(&[
        snapshot.battery_update.bat_voltage_sag,
        snapshot.battery_update.bat_voltage,
        snapshot.battery_update.amperage,
        snapshot.battery_update.m_ah_drawn,
        snapshot.gyro_update.rotation[0],
        snapshot.gyro_update.rotation[1],
        snapshot.gyro_update.rotation[2],
        snapshot.gyro_update.rotation[3],
        snapshot.gyro_update.linear_acc[0],
        snapshot.gyro_update.linear_acc[1],
        snapshot.gyro_update.linear_acc[2],
        snapshot.gyro_update.angular_velocity[0],
        snapshot.gyro_update.angular_velocity[1],
        snapshot.gyro_update.angular_velocity[2],
        // TODO: double check this
        snapshot.motor_input[0], // snapshot.throttle,
        snapshot.motor_input[1], // snapshot.roll,
        snapshot.motor_input[2], // snapshot.yaw,
        snapshot.motor_input[3], // snapshot.pitch,
    ])
}

fn snapshots_to_flight_input(flight_logs: Vec<FlightLog>) -> FlightInput {
    let episodes = flight_logs.len();
    let time = flight_logs.iter().map(|x| x.steps.len()).max().unwrap();
    let data = flight_logs
        .iter()
        .map(|fl| {
            let columns = fl
                .steps
                .iter()
                .map(|f| snapshot_fl_input(f))
                .collect::<Vec<_>>();
            let m = DMatrix::from_columns(&columns).transpose();
            m
        })
        .collect();
    FlightInput {
        episodes,
        time,
        vars: 18, // TODO: do not hardcode in the future
        data,
    }
}

pub fn train_thing() {
    let replay_id = "only_up";
    let mut sim_context = SimContext::default();
    // load from files
    sim_context.set_loader(&sim_context::LoaderType::File);
    sim_context.set_logger(sim_context::LoggerType::File);

    let flight_log = sim_context.load_replay(replay_id);
    let mut drone_rc = DroneRc::new(
        500,
        0.3,
        0.99,
        0.2,
        RepresentationType::Output(1.),
        RidgeRegression::new(1.),
    );
    drone_rc.esn.set_input_weights(18);
    let input = snapshots_to_flight_input(vec![flight_log.clone()]);
    let data_points = DMatrix::from_columns(
        &flight_log
            .steps
            .iter()
            .map(snapshot_fl_input)
            .collect::<Vec<_>>(),
    )
    .transpose();

    drone_rc.old_fit(Box::new(input.clone()), data_points);
    // save the trained reservoir controller
    sim_context.insert_drone_rc("trained_on_only_up", ResController::new(drone_rc.clone()));

    // let drone_rc_db = sim_context.select_reservoir("only_up_2");
    let new_rc_mode = sim_context.load_drone_rc("trained_on_only_up");
    let predicted_points = new_rc_mode.model.lock().unwrap().predict(Box::new(input));
    let mut rec_flight_logs = vec![];
    for (i, motor_inputs) in predicted_points.row_iter().enumerate() {
        let mut fl = flight_log.steps[i].clone();
        println!("{}", *motor_inputs.get(0).unwrap());
        println!("{}", *motor_inputs.get(1).unwrap());
        println!("{}", *motor_inputs.get(2).unwrap());
        println!("{}", *motor_inputs.get(3).unwrap());
        println!("---------------------------------");
        let motor_inputs: MotorInput = MotorInput {
            input: [
                *motor_inputs.get(0).unwrap(),
                *motor_inputs.get(1).unwrap(),
                *motor_inputs.get(2).unwrap(),
                *motor_inputs.get(3).unwrap(),
            ],
        };
        fl.motor_input = motor_inputs;
        rec_flight_logs.push(fl);
    }
    sim_context.insert_logs(FlightLog::new(
        "recalculated_inputs".into(),
        rec_flight_logs,
    ));
}

#[cfg(test)]
mod test {
    use crate::train_thing;

    // resurrecting the old only up test!
    #[test]
    fn old_res_training_thing() {
        train_thing();
    }
}
