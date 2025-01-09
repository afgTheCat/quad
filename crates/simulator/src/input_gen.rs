// What we want to do is to have the 4 degrees of freedom
// explored in a semi realistic manner => then generate data from it

use crate::{
    logger::SimLogger, low_pass_filter::LowPassFilter, BatteryModel, BatteryState, Drone,
    DroneFrameState, DroneModel, GyroModel, GyroState, RotorModel, RotorState, RotorsState,
    SampleCurve, SamplePoint, SimulationFrame, Simulator,
};
use db::AscentDb;
use flight_controller::{
    controllers::bf_controller::BFController, BatteryUpdate, Channels, MotorInput,
};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use rand::{distributions::Bernoulli, prelude::Distribution, thread_rng};
use rayon::iter::IntoParallelRefIterator;
use rayon::iter::ParallelIterator;
use std::{fmt::format, sync::Arc, thread, time::Duration, usize};

pub const PROP_BLADE_MESH_NAMES: [(f64, Vector3<f64>); 4] = [
    (
        -1.,
        Vector3::new(
            0.14055216312408447,
            0.013523973524570465,
            0.11647607386112213,
        ),
    ),
    (
        1.,
        Vector3::new(
            0.14055214822292328,
            0.013523973524570465,
            -0.11647609621286392,
        ),
    ),
    (
        1.,
        Vector3::new(
            -0.14055216312408447,
            0.013523973524570465,
            0.11647608131170273,
        ),
    ),
    (
        -1.,
        Vector3::new(
            -0.14055216312408447,
            0.013523973524570465,
            -0.11647607386112213,
        ),
    ),
];

fn set_up_simulation() -> Simulator {
    let rotors_state =
        RotorsState(
            PROP_BLADE_MESH_NAMES.map(|(rotor_dir, motor_pos)| RotorState {
                current: 0.,
                rpm: 0.,
                motor_torque: 0.,
                effective_thrust: 0.,
                pwm: 0.,
                rotor_dir,
                motor_pos: motor_pos.clone(),
                pwm_low_pass_filter: LowPassFilter::default(),
            }),
        );

    let bat_voltage_curve = SampleCurve::new(vec![
        SamplePoint::new(-0.06, 4.4),
        SamplePoint::new(0.0, 4.2),
        SamplePoint::new(0.01, 4.05),
        SamplePoint::new(0.04, 3.97),
        SamplePoint::new(0.30, 3.82),
        SamplePoint::new(0.40, 3.7),
        SamplePoint::new(1.0, 3.49),
        SamplePoint::new(1.01, 3.4),
        SamplePoint::new(1.03, 3.3),
        SamplePoint::new(1.06, 3.0),
        SamplePoint::new(1.08, 0.0),
    ]);

    let battery_model = BatteryModel {
        quad_bat_capacity: 850.,
        bat_voltage_curve,
        quad_bat_cell_count: 4,
        quad_bat_capacity_charged: 850.,
        max_voltage_sag: 1.4,
    };

    let battery_state = BatteryState {
        capacity: 850.,
        bat_voltage: 4.2,
        bat_voltage_sag: 4.2,
        amperage: 0.,
        m_ah_drawn: 0.,
    };

    let drone_state = DroneFrameState {
        position: Vector3::zeros(),
        rotation: Rotation3::identity(), // stargin position
        linear_velocity: Vector3::zeros(),
        angular_velocity: Vector3::new(0., 0., 0.),
        acceleration: Vector3::zeros(),
    };

    let gyro_state = GyroState {
        rotation: UnitQuaternion::identity(),
        acceleration: Vector3::zeros(),
        angular_velocity: Vector3::zeros(),
        low_pass_filters: [
            LowPassFilter::default(),
            LowPassFilter::default(),
            LowPassFilter::default(),
        ],
    };

    let initial_frame = SimulationFrame {
        battery_state,
        rotors_state,
        drone_state,
        gyro_state,
    };

    let drone_model = DroneModel {
        frame_drag_area: Vector3::new(0.0082, 0.0077, 0.0082),
        frame_drag_constant: 1.45,
        mass: 0.2972,
        inv_tensor: Matrix3::from_diagonal(&Vector3::new(750., 5150.0, 750.0)),
    };

    let gyro_model = GyroModel {};

    let rotor_model = RotorModel {
        prop_max_rpm: 36000.0,
        pwm_low_pass_filter: [
            LowPassFilter::default(),
            LowPassFilter::default(),
            LowPassFilter::default(),
            LowPassFilter::default(),
        ],
        motor_kv: 3200., // kv
        motor_r: 0.13,   // resistence
        motor_io: 0.23,  // idle current
        prop_thrust_factor: Vector3::new(-5e-05, -0.0025, 4.75),
        prop_torque_factor: 0.0056,
        prop_a_factor: 7.43e-10,
        prop_inertia: 3.5e-07,
    };

    let logger = SimLogger::new(
        MotorInput::default(),
        BatteryUpdate {
            bat_voltage_sag: initial_frame.battery_state.bat_voltage_sag,
            bat_voltage: initial_frame.battery_state.bat_voltage,
            amperage: initial_frame.battery_state.amperage,
            m_ah_drawn: initial_frame.battery_state.m_ah_drawn,
            cell_count: battery_model.quad_bat_cell_count,
        },
        initial_frame.gyro_state.gyro_update(),
        Channels::default(),
    );

    let drone = Drone {
        current_frame: initial_frame.clone(),
        next_frame: initial_frame.clone(),
        battery_model: battery_model.clone(),
        rotor_model,
        drone_model,
        gyro_model,
    };

    let flight_controller = Arc::new(BFController::new());

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
        if pos < -1. || pos > 1. {
            pos = f64::min(f64::max(pos, -1000.), 1000.);
            vel = 0.;
        }
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
    // TODO: we should pool instead of just creating a simulation each time
    let mut simulation = set_up_simulation();
    simulation.init(episode_name.clone());
    // TODO: we may want to better controll this
    for input in inputs {
        simulation.simulate_delta(Duration::from_millis(1), input);
    }
    db.write_flight_logs(&episode_name, &simulation.logger.data);
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
    let db = AscentDb::new("/home/gabor/ascent/quad/data.sqlite");
    let training_inputs = (0..training_size)
        .map(|_| generate_all_axis(training_duration))
        .collect::<Vec<_>>();
    let test_inputs = (0..test_size)
        .map(|_| generate_all_axis(training_duration))
        .collect::<Vec<_>>();

    // TODO: do this on multiple cores
    for (ep, inputs) in training_inputs.into_iter().enumerate() {
        let tr_id = format!("{}_tr_{}", data_set_id, ep);
        let mut simulation = set_up_simulation();
        simulation.init(tr_id.clone());
        for input in inputs {
            simulation.simulate_delta(Duration::from_millis(1), input);
        }
        db.write_flight_logs(&tr_id, &simulation.logger.data);
    }

    for (ep, inputs) in test_inputs.into_iter().enumerate() {
        let tr_id = format!("{}_te_{}", data_set_id, ep);
        let mut simulation = set_up_simulation();
        simulation.init(tr_id.clone());
        for input in inputs {
            simulation.simulate_delta(Duration::from_millis(1), input);
        }
        db.write_flight_logs(&tr_id, &simulation.logger.data);
    }
}

#[cfg(test)]
mod test {
    use super::{build_data_set, generate_all_axis, set_up_simulation};
    use db::AscentDb;
    use std::time::Duration;

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
        let db = AscentDb::new("/home/gabor/ascent/quad/data.sqlite");
        let mut simulation = set_up_simulation();
        let duration = Duration::from_secs(5);
        simulation.init("test_generated_simulation".into());
        let inputs_per_milisecs = generate_all_axis(duration);
        for input in inputs_per_milisecs {
            simulation.simulate_delta(Duration::from_millis(1), input);
        }
        let simulation_id = simulation
            .logger
            .simulation_id
            .as_ref()
            .unwrap()
            .to_string();
        db.write_flight_logs(&simulation_id, &simulation.logger.data);
    }
}
