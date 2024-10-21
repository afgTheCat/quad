// #[cfg(feature = "legacy_sim")]
// use quboid::build_app;

#[cfg(feature = "legacy_sim")]
use quboid_bevy::build_app;

#[cfg(not(feature = "legacy_sim"))]
use quad_bevy::build_app;

fn main() {
    let mut app = build_app();
    app.run();
}

#[cfg(test)]
mod test {
    use flight_controller::{
        controllers::bf_controller::BFController, BatteryUpdate, Channels, FlightController,
        FlightControllerUpdate, GyroUpdate,
    };

    #[test]
    fn main_sim_bf_controller() {
        let mut inputs = vec![];
        let controller = BFController;
        controller.init();
        controller.set_armed();

        let battery_update = BatteryUpdate {
            bat_voltage_sag: 1.,
            bat_voltage: 1.,
            amperage: 1.,
            m_ah_drawn: 1.,
            cell_count: 4,
        };

        let gyro_update = GyroUpdate {
            rotation: [1., 0., 0., 0.],
            acc: [0., 0., 0.],
            gyro: [0., 0., 0.],
        };

        let channels = Channels {
            throttle: 0.,
            yaw: 0.,
            pitch: 0.,
            roll: 0.,
        };

        for _ in 0..100 {
            let flight_controller_update = FlightControllerUpdate {
                battery_update,
                gyro_update,
                channels,
            };
            let motor_input = controller.update(flight_controller_update);
            inputs.push(motor_input);
        }

        println!("{inputs:?}");
    }
}
