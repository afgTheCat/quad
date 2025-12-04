use std::time::Duration;

use db_common::DBFlightLog;
use drone::Drone;
use flight_controller::MotorInput;
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

// Normalizes all inputs between -1 and 1
pub fn snapshot_fl_input(snapshot: &SnapShot, drone: &Drone) -> DVector<f64> {
    DVector::from_row_slice(&[
        snapshot.battery_update.bat_voltage_sag
            / (drone.current_frame.battery_state.bat_voltage_sag
                * drone.battery_model.quad_bat_cell_count as f64),
        snapshot.battery_update.bat_voltage
            / (drone.current_frame.battery_state.bat_voltage
                * drone.battery_model.quad_bat_cell_count as f64),
        snapshot.battery_update.amperage / 60., // TODO: should be calculated
        snapshot.battery_update.m_ah_drawn / drone.battery_model.quad_bat_capacity,
        snapshot.gyro_update.rotation[0],
        snapshot.gyro_update.rotation[1],
        snapshot.gyro_update.rotation[2],
        snapshot.gyro_update.rotation[3],
        snapshot.gyro_update.linear_acc[0] / 40.,
        snapshot.gyro_update.linear_acc[1] / 40.,
        snapshot.gyro_update.linear_acc[2] / 40.,
        snapshot.gyro_update.angular_velocity[0] / 20.,
        snapshot.gyro_update.angular_velocity[1] / 20.,
        snapshot.gyro_update.angular_velocity[2] / 20.,
        snapshot.channels.throttle,
        snapshot.channels.roll,
        snapshot.channels.yaw,
        snapshot.channels.pitch,
    ])
}

fn snapshots_to_flight_input(flight_logs: Vec<FlightLog>, drone: &Drone) -> FlightInput {
    let episodes = flight_logs.len();
    let time = flight_logs.iter().map(|x| x.steps.len()).max().unwrap();
    let mut data = vec![];
    for fl in flight_logs.iter() {
        let columns = fl
            .steps
            .iter()
            .map(|f| snapshot_fl_input(f, drone))
            .collect::<Vec<_>>();
        data.push(DMatrix::from_columns(&columns).transpose());
    }
    let vars = data[0].shape().1;
    FlightInput {
        episodes,
        time,
        vars,
        data,
    }
}

fn recreate_replay(
    sim_context: &mut SimContext,
    controller_id: &str,
    replay_id: &str,
    inserted_replay_id: Option<String>,
) {
    let drone = sim_context.load_drone().unwrap();
    let inserted_replay_id = if let Some(inserted_replay_id) = inserted_replay_id {
        inserted_replay_id
    } else {
        format!("{replay_id}_recreated")
    };
    let flight_log = sim_context.load_replay(replay_id);
    let input = snapshots_to_flight_input(vec![flight_log.clone()], &drone);
    let new_rc_model = sim_context.load_drone_rc(controller_id);
    let predicted_points = new_rc_model.predict(Box::new(input));
    let mut rec_flight_logs = vec![];
    for (i, motor_inputs) in predicted_points.row_iter().enumerate() {
        let mut fl = flight_log.steps[i].clone();
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
    sim_context.insert_logs(FlightLog::new(inserted_replay_id.into(), rec_flight_logs));
}

pub fn simple_training_strategy() {
    let only_up_trajectory = "only_up";
    let controller_id = "trained_on_only_up";

    let mut sim_context = SimContext::default();
    sim_context.set_loader(&sim_context::LoaderType::File);
    sim_context.set_logger(sim_context::LoggerType::File);

    // does not change
    let drone = sim_context.load_drone().unwrap();
    let mut flight_log = sim_context.load_replay(only_up_trajectory);
    flight_log.downsample(Duration::from_millis(1));
    let mut drone_rc = DroneRc::new(
        500,
        0.3,
        0.99,
        0.2,
        RepresentationType::AllStates,
        RidgeRegression::new(1.),
    );
    let input = snapshots_to_flight_input(vec![flight_log.clone()], &drone);
    drone_rc.esn.set_input_weights(input.vars);

    let data_points = DMatrix::from_columns(
        &flight_log
            .steps
            .iter()
            .map(|e| DVector::from_row_slice(&e.motor_input.input))
            .collect::<Vec<_>>(),
    )
    .transpose();

    drone_rc.fit(Box::new(input.clone()), data_points);
    recreate_replay(&mut sim_context, controller_id, only_up_trajectory, None);
    recreate_replay(&mut sim_context, controller_id, "only_up2", None);
}

pub fn buffered_training_strategy() {
    let only_up_trajectory = "only_up";
    let controller_id = "buffered_trained_on_only_up";

    let mut sim_context = SimContext::default();
    sim_context.set_loader(&sim_context::LoaderType::File);
    sim_context.set_logger(sim_context::LoggerType::File);

    // does not change
    let drone = sim_context.load_drone().unwrap();
    let mut flight_log = sim_context.load_replay(only_up_trajectory);

    flight_log.downsample(Duration::from_millis(1));
    let mut drone_rc = DroneRc::new(
        500,
        0.3,
        0.99,
        0.2,
        RepresentationType::AllStates,
        RidgeRegression::new(1.),
    );
    let input = snapshots_to_flight_input(vec![flight_log.clone()], &drone);
    drone_rc.esn.set_input_weights(input.vars);
    let data_points = DMatrix::from_columns(
        &flight_log
            .steps
            .iter()
            .map(|e| DVector::from_row_slice(&e.motor_input.input))
            .collect::<Vec<_>>(),
    )
    .transpose();

    drone_rc.fit(Box::new(input.clone()), data_points);
    sim_context.insert_drone_rc(controller_id, drone_rc);

    recreate_replay(
        &mut sim_context,
        controller_id,
        only_up_trajectory,
        Some("my_recreation".into()),
    );
}

#[cfg(test)]
mod test {
    use crate::{buffered_training_strategy, simple_training_strategy};

    // resurrecting the old only up test!
    #[test]
    fn old_res_training_thing() {
        simple_training_strategy();
    }

    #[test]
    fn new_test_training_thing() {
        buffered_training_strategy();
    }
}
