use std::time::Duration;

use db_common::DBFlightLog;
use drone::Drone;
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
        // snapshot.battery_update.bat_voltage_sag
        //     / (drone.current_frame.battery_state.bat_voltage_sag
        //         * drone.battery_model.quad_bat_cell_count as f64),
        // snapshot.battery_update.bat_voltage
        //     / (drone.current_frame.battery_state.bat_voltage
        //         * drone.battery_model.quad_bat_cell_count as f64),
        // snapshot.battery_update.amperage / 60., // TODO: should be calculated
        // snapshot.battery_update.m_ah_drawn / drone.battery_model.quad_bat_capacity,
        // snapshot.gyro_update.rotation[0],
        // snapshot.gyro_update.rotation[1],
        // snapshot.gyro_update.rotation[2],
        // snapshot.gyro_update.rotation[3],
        // snapshot.gyro_update.linear_acc[0] / 40.,
        // snapshot.gyro_update.linear_acc[1] / 40.,
        // snapshot.gyro_update.linear_acc[2] / 40.,
        // snapshot.gyro_update.angular_velocity[0] / 20.,
        // snapshot.gyro_update.angular_velocity[1] / 20.,
        // snapshot.gyro_update.angular_velocity[2] / 20.,
        snapshot.channels.throttle,
        snapshot.channels.roll,
        snapshot.channels.yaw,
        snapshot.channels.pitch,
    ])
}

pub struct SingleFlightTrainingStrategy {
    train_flight_log_id: String,
    trained_controller_id: String,
    recreated_replay_id: String,
    representation_type: RepresentationType,
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
    flight_log_id: &str,
    new_fliht_log: &str,
) {
    sim_context.set_controller(sim_context::ControllerType::Reservoir(controller_id.into()));
    sim_context.set_logger(sim_context::LoggerType::File(new_fliht_log.into()));
    let mut simulator = sim_context.try_load_simulator().unwrap();
    let mut current_time = Duration::ZERO;
    let flight_log = sim_context.load_flight_log(flight_log_id);
    for SnapShot {
        duration: time_elapsed,
        channels,
        ..
    } in flight_log.steps
    {
        let delta = time_elapsed - current_time;
        simulator.simulate_delta(delta, channels);
        current_time = time_elapsed;
    }
}

pub fn train_on_flight(strategy: SingleFlightTrainingStrategy) {
    let mut sim_context = SimContext::default();
    sim_context.set_loader(&sim_context::LoaderType::File);

    // does not change
    let drone = sim_context.load_drone().unwrap();
    let mut flight_log = sim_context.load_flight_log(&strategy.train_flight_log_id);

    flight_log.downsample(Duration::from_millis(10));
    let mut drone_rc = DroneRc::new(
        500,
        0.3,
        0.99,
        0.2,
        strategy.representation_type,
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
    sim_context.insert_drone_rc(&strategy.trained_controller_id, drone_rc);

    recreate_replay(
        &mut sim_context,
        &strategy.trained_controller_id,
        &strategy.train_flight_log_id,
        &strategy.recreated_replay_id,
    );
}

#[cfg(test)]
mod test {
    use sim_context::SimContext;

    use crate::{SingleFlightTrainingStrategy, recreate_replay, train_on_flight};

    #[test]
    fn new_test_training_thing() {
        let strategy = SingleFlightTrainingStrategy {
            train_flight_log_id: "only_up".into(),
            trained_controller_id: "controller_trained_on_only_up".into(),
            recreated_replay_id: "only_up_recreation_buffered_states".into(),
            representation_type: res::representation::RepresentationType::BufferedStates(10),
        };
        train_on_flight(strategy);
    }

    #[test]
    fn just_the_controller() {
        const CONTROLLER_ID: &str = "buffered_trained_on_only_up";
        const REPLAY_ID: &str = "only_up";
        const NEW_REPLAY_ID: &str = "hmmmmm";

        let mut sim_context = SimContext::default();
        sim_context.set_loader(&sim_context::LoaderType::File);
        sim_context.set_logger(sim_context::LoggerType::File(NEW_REPLAY_ID.into()));
        recreate_replay(&mut sim_context, CONTROLLER_ID, REPLAY_ID, "wwwww");
    }
}
