// What we want to do is to have the 4 degrees of freedom
// explored in a semi realistic manner => then generate data from it

use crate::loader::{SimLoader, SimulationLoader};
use crate::loggers::{EmptyLogger, Logger, RerunLogger};
use crate::{loggers::DBLogger, Simulator};
use db::AscentDb;
use flight_controller::controllers::bf_controller::BFController;
use flight_controller::{Channels, FlightController, MotorInput};
use rand::{distributions::Bernoulli, prelude::Distribution, thread_rng};
use rayon::iter::IntoParallelRefIterator;
use rayon::iter::ParallelIterator;
use std::sync::Mutex;
use std::{sync::Arc, thread, time::Duration};

pub enum LogType {
    DB { sim_id: String },
    Rerun { sim_id: String },
    Empty,
}

pub fn set_up_simulation(
    db: Arc<AscentDb>,
    loader: &impl SimulationLoader,
    logger: LogType,
    flight_controller: impl FlightController,
) -> Simulator {
    let drone = loader.load_drone(1);
    let current_battery_update = drone.battery_update();
    let current_gyro = drone.current_frame.gyro_state.gyro_update();

    let logger: Arc<Mutex<dyn Logger>> = match logger {
        LogType::DB { sim_id } => Arc::new(Mutex::new(DBLogger::new(
            sim_id.clone(),
            MotorInput::default(),
            current_battery_update,
            current_gyro,
            Channels::default(),
            db.clone(),
        ))),
        LogType::Rerun { sim_id } => Arc::new(Mutex::new(RerunLogger::new(sim_id.clone()))),
        LogType::Empty => Arc::new(Mutex::new(EmptyLogger::default())),
    };

    let flight_controller = Arc::new(flight_controller);

    Simulator {
        drone: drone.clone(),
        flight_controller: flight_controller.clone(),
        time_accu: Duration::default(),
        time: Duration::new(0, 0),
        dt: Duration::from_nanos(5000), // TODO: update this
        fc_time_accu: Duration::default(),
        logger,
    }
}

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

fn build_episode(db: Arc<AscentDb>, episode_name: String, training_duration: Duration) {
    let inputs = generate_all_axis(training_duration);
    let sim_loader = SimLoader::new(db.clone());
    // TODO: we should pool instead of just creating a simulation each time
    let mut simulation = set_up_simulation(
        db.clone(),
        &sim_loader,
        LogType::DB {
            sim_id: episode_name,
        },
        BFController::default(),
    );
    simulation.init();

    // TODO: we may want to better controll this
    for input in inputs {
        simulation.simulate_delta(Duration::from_millis(1), input);
    }
    // simulation.write_remaining_logs();
}

pub fn build_data_set2(
    data_set_id: String,
    training_duration: Duration,
    training_size: usize,
    test_size: usize,
) {
    let db = Arc::new(AscentDb::new("/home/gabor/ascent/quad/data.sqlite"));
    let handles = (0..training_size)
        .map(|ep| format!("{}_tr_{}", data_set_id, ep))
        .chain((0..test_size).map(|ep| format!("{}_te_{}", data_set_id, ep)))
        .map(|id| {
            let ep_db = db.clone();
            thread::spawn(move || build_episode(ep_db, id, training_duration))
        })
        .collect::<Vec<_>>();

    for handle in handles {
        handle.join().unwrap();
    }
}

// Use thread pooling to increase the efficency of things
pub fn build_data_set3(
    data_set_id: String,
    training_duration: Duration,
    training_size: usize,
    test_size: usize,
) {
    let db = Arc::new(AscentDb::new("/home/gabor/ascent/quad/data.sqlite"));
    let episode_ids = (0..training_size)
        .map(|ep| format!("{}_tr_{}", data_set_id, ep))
        .chain((0..test_size).map(|ep| format!("{}_te_{}", data_set_id, ep)))
        .collect::<Vec<_>>();
    episode_ids.par_iter().for_each(|episode_id| {
        build_episode(db.clone(), episode_id.clone(), training_duration);
    });
}

// TODO: we could do this on multiple threads as well!
pub fn build_data_set(
    data_set_id: String,
    training_duration: Duration,
    training_size: usize,
    test_size: usize,
) {
    let db = Arc::new(AscentDb::new("/home/gabor/ascent/quad/data.sqlite"));
    let training_inputs = (0..training_size)
        .map(|_| generate_all_axis(training_duration))
        .collect::<Vec<_>>();
    let test_inputs = (0..test_size)
        .map(|_| generate_all_axis(training_duration))
        .collect::<Vec<_>>();
    let sim_loader = SimLoader::new(db.clone());

    // TODO: do this on multiple cores
    for (ep, inputs) in training_inputs.into_iter().enumerate() {
        let mut simulation = set_up_simulation(
            db.clone(),
            &sim_loader,
            LogType::DB {
                sim_id: format!("{}_tr_{}", data_set_id, ep),
            },
            BFController::default(),
        );
        simulation.init(); // tr_id.clone()
        for input in inputs {
            simulation.simulate_delta(Duration::from_millis(1), input);
        }
    }

    for (ep, inputs) in test_inputs.into_iter().enumerate() {
        let mut simulation = set_up_simulation(
            db.clone(),
            &sim_loader,
            LogType::DB {
                sim_id: format!("{}_te_{}", data_set_id, ep),
            },
            BFController::default(),
        );
        simulation.init(); // tr_id.clone()
        for input in inputs {
            simulation.simulate_delta(Duration::from_millis(1), input);
        }
    }
}

#[cfg(test)]
mod test {
    use super::{build_data_set, generate_all_axis, set_up_simulation};
    use crate::{input_gen::LogType, loader::SimLoader};
    use db::AscentDb;
    use flight_controller::controllers::bf_controller::BFController;
    use rayon::iter::IntoParallelRefMutIterator;
    use rayon::iter::ParallelIterator;
    use std::{sync::Arc, time::Duration};

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
        let db = Arc::new(AscentDb::new("/home/gabor/ascent/quad/data.sqlite"));
        let sim_loader = SimLoader::new(db.clone());

        let mut simulation = set_up_simulation(
            db.clone(),
            &sim_loader,
            LogType::DB {
                sim_id: "test_generated_simulation".into(),
            },
            BFController::default(),
        );
        let duration = Duration::from_secs(5);
        simulation.init();
        let inputs_per_milisecs = generate_all_axis(duration);
        for input in inputs_per_milisecs {
            simulation.simulate_delta(Duration::from_millis(1), input);
        }
    }

    #[test]
    fn test_unique_loading() {
        let db = Arc::new(AscentDb::new("/home/gabor/ascent/quad/data.sqlite"));
        let sim_loader = SimLoader::new(db.clone());

        let reference_controller1 = BFController::default();
        let reference_controller2 = BFController::default();
        let controller1 = BFController::default();
        let controller2 = BFController::default();

        let test_flight_duration = Duration::from_secs(5);
        let inputs1 = generate_all_axis(test_flight_duration);
        let inputs2 = generate_all_axis(test_flight_duration);

        let mut reference_simulation1 = set_up_simulation(
            db.clone(),
            &sim_loader,
            LogType::DB {
                sim_id: "reference_sim1".into(),
            },
            reference_controller1,
        );
        reference_simulation1.init();

        for input in &inputs1 {
            reference_simulation1.simulate_delta(Duration::from_millis(1), input.clone());
        }

        let mut reference_simulation2 = set_up_simulation(
            db.clone(),
            &sim_loader,
            LogType::DB {
                sim_id: "reference_sim2".into(),
            },
            reference_controller2,
        );
        reference_simulation2.init();

        for input in &inputs2 {
            reference_simulation2.simulate_delta(Duration::from_millis(1), input.clone());
        }

        let mut simulation1 = set_up_simulation(
            db.clone(),
            &sim_loader,
            LogType::DB {
                sim_id: "generated_sim1".into(),
            },
            controller1,
        );
        simulation1.init();

        let mut simulation2 = set_up_simulation(
            db.clone(),
            &sim_loader,
            LogType::DB {
                sim_id: "generated_sim2".into(),
            },
            controller2,
        );
        simulation2.init();

        [(&mut simulation1, inputs1), (&mut simulation2, inputs2)]
            .par_iter_mut()
            .for_each(|(sim, inputs)| {
                for input in inputs {
                    sim.simulate_delta(Duration::from_millis(1), input.clone());
                }
            });
    }
}
