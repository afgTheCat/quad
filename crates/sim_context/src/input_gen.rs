use flight_controller::Channels;
use rand::{distributions::Bernoulli, prelude::Distribution, thread_rng};
use std::time::Duration;

use crate::SimContext;

// TODO: check if the data set is going to be rich enough
fn generate_axis(milisecs: u128) -> Vec<f64> {
    let bernoulli = Bernoulli::new(0.5).unwrap();
    let mut rng = thread_rng();
    let axis = (0..milisecs).fold((0., 0., vec![]), |acc, _| {
        let (mut pos, mut vel, mut all_pos) = acc;
        vel += if bernoulli.sample(&mut rng) {
            0.0001
        } else {
            -0.0001
        };
        pos += vel;
        if !(-1. ..=1.).contains(&pos) {
            pos = f64::clamp(pos, -1000., 1000.);
            vel = 0.;
        }
        pos = pos.clamp(-1., 1.);
        all_pos.push(pos);
        (pos, vel, all_pos)
    });
    axis.2
}

// Generates inputs for each axis with 1ms delay
pub fn generate_all_axis(duration: Duration) -> Vec<Channels> {
    let milisecs = duration.as_millis();
    let inputs = [0; 4].map(|_| generate_axis(milisecs));
    (0..milisecs)
        .map(|ms| Channels {
            throttle: inputs[0][ms as usize],
            roll: inputs[1][ms as usize],
            pitch: inputs[2][ms as usize],
            yaw: inputs[3][ms as usize],
        })
        .collect()
}

// pub fn set_up_simulation(
//     db: Arc<AscentDb>,
//     loader: &impl SimulationLoader,
//     logger: LogType,
//     flight_controller: impl FlightController,
// ) -> Simulator {
//     let drone = loader.load_drone("default_config");
//     let current_battery_update = drone.battery_update();
//     let current_gyro = drone.current_frame.gyro_state.gyro_update();
//
//     let logger: Arc<Mutex<dyn Logger>> = match logger {
//         LogType::DB { sim_id } => Arc::new(Mutex::new(DBLogger::new(
//             sim_id.clone(),
//             MotorInput::default(),
//             current_battery_update,
//             current_gyro,
//             Channels::default(),
//             db.clone(),
//         ))),
//         LogType::Rerun { sim_id } => Arc::new(Mutex::new(RerunLogger::new(sim_id.clone()))),
//         LogType::Empty => Arc::new(Mutex::new(EmptyLogger::default())),
//     };
//
//     let flight_controller = Arc::new(flight_controller);
//
//     Simulator {
//         drone: drone.clone(),
//         flight_controller: flight_controller.clone(),
//         time_accu: Duration::default(),
//         time: Duration::new(0, 0),
//         dt: Duration::from_nanos(5000), // TODO: update this
//         fc_time_accu: Duration::default(),
//         logger,
//     }
// }

pub fn build_data_set(
    data_set_id: String,
    training_duration: Duration,
    training_size: usize,
    test_size: usize,
) {
    let mut context = SimContext::default();
    // let db = Arc::new(AscentDb::new("/home/gabor/ascent/quad/data.sqlite"));
    let training_inputs = (0..training_size)
        .map(|_| generate_all_axis(training_duration))
        .collect::<Vec<_>>();
    let test_inputs = (0..test_size)
        .map(|_| generate_all_axis(training_duration))
        .collect::<Vec<_>>();
    // let sim_loader = SimLoader::new(db.clone());

    // TODO: do this on multiple cores
    for (ep, inputs) in training_inputs.into_iter().enumerate() {
        let mut simulation = context.try_load_simulator().unwrap();
        simulation.init(); // tr_id.clone()
        for input in inputs {
            simulation.simulate_delta(Duration::from_millis(1), input);
        }
    }

    for (ep, inputs) in test_inputs.into_iter().enumerate() {
        let mut simulation = context.try_load_simulator().unwrap();
        // let mut simulation = set_up_simulation(
        //     db.clone(),
        //     &sim_loader,
        //     LogType::DB {
        //         sim_id: format!("{}_te_{}", data_set_id, ep),
        //     },
        //     BFController::default(),
        // );
        simulation.init(); // tr_id.clone()
        for input in inputs {
            simulation.simulate_delta(Duration::from_millis(1), input);
        }
    }
}

#[cfg(test)]
mod test {
    use std::time::Duration;

    use crate::{
        input_gen::{build_data_set, generate_all_axis},
        SimContext,
    };

    #[test]
    fn build_data_set_1() {
        let ds_id_1 = "ds_id_1";
        let tr_duration = Duration::from_secs(5);
        let tr_size = 10;
        let te_size = 10;
        build_data_set(ds_id_1.into(), tr_duration, tr_size, te_size);
    }

    #[test]
    fn generate_fake_flight() {
        let mut simulation = SimContext::default().try_load_simulator().unwrap();
        let duration = Duration::from_secs(5);
        simulation.init();
        let inputs_per_milisecs = generate_all_axis(duration);
        for input in inputs_per_milisecs {
            simulation.simulate_delta(Duration::from_millis(1), input);
        }
    }
}
