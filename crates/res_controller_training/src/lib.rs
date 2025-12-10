use std::time::Duration;

use drone::Drone;
use flight_controller::FlightController;
use flight_controller::FlightControllerUpdate;
use loggers::{FlightLog, SnapShot};
use nalgebra::{DMatrix, DVector};
use res::{input::FlightInput, representation::RepresentationType};
use res_controller::{DroneRc, snapshot_to_reservoir_input};
use ridge::RidgeRegression;
use sim_context::SimContext;

pub struct SingleFlightTrainingStrategy {
    train_flight_log: String,
    trained_controller_id: String,
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
            .map(|f| snapshot_to_reservoir_input(f, drone))
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

pub fn recreate_replay(
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

pub fn train_on_flight(sim_context: &mut SimContext, strategy: &SingleFlightTrainingStrategy) {
    let drone = sim_context.load_drone().unwrap();
    let mut flight_log = sim_context.load_flight_log(&strategy.train_flight_log);

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
}

pub struct EvaluateSingleTrainingStrategy {
    test_flight_log: String,
    test_flight_controller: String,
}

pub fn evaluate_single_episode_training(
    sim_context: &mut SimContext,
    EvaluateSingleTrainingStrategy {
        test_flight_log,
        test_flight_controller,
    }: &EvaluateSingleTrainingStrategy,
) {
    let flight_log = sim_context
        .loader
        .lock()
        .unwrap()
        .load_flight_log(&test_flight_log);
    let controller = sim_context
        .loader
        .lock()
        .unwrap()
        .load_res_controller(&test_flight_controller);

    let mut predicted_motor_inputs = vec![];
    let mut actual_motor_inputs = vec![];
    for snapshot in flight_log.steps {
        actual_motor_inputs.push(snapshot.motor_input);
        // TODO: this should be the accurate time elapsed, but we don't really use this right now
        let duration = Duration::default();
        let update = FlightControllerUpdate {
            battery_update: snapshot.battery_update,
            gyro_update: snapshot.gyro_update,
            channels: snapshot.channels,
        };
        let prediction = controller.update(duration.as_secs_f64(), update);
        predicted_motor_inputs.push(prediction);
    }
}

#[cfg(test)]
mod test {
    use crate::{SingleFlightTrainingStrategy, recreate_replay, train_on_flight};
    use sim_context::SimContext;

    #[test]
    fn up_only_training() {
        let mut sim_context = SimContext::default();
        sim_context.set_loader(&sim_context::LoaderType::File);
        let strategy = SingleFlightTrainingStrategy {
            train_flight_log: "up_only".into(),
            trained_controller_id: "up_only_controller".into(),
            representation_type: res::representation::RepresentationType::BufferedStates(10),
        };
        train_on_flight(&mut sim_context, &strategy);
        recreate_replay(
            &mut sim_context,
            &strategy.trained_controller_id,
            &strategy.train_flight_log,
            "up_only_recreation",
        );
    }

    #[test]
    fn yaw_training() {
        let mut sim_context = SimContext::default();
        sim_context.set_loader(&sim_context::LoaderType::File);
        let strategy = SingleFlightTrainingStrategy {
            train_flight_log: "yaw_only".into(),
            trained_controller_id: "yaw_only_controller".into(),
            representation_type: res::representation::RepresentationType::BufferedStates(10),
        };
        train_on_flight(&mut sim_context, &strategy);
        recreate_replay(
            &mut sim_context,
            &strategy.trained_controller_id,
            &strategy.train_flight_log,
            "yaw_only_controller",
        );
    }
}
