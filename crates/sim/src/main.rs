// #[cfg(feature = "legacy_sim")]
// use quboid::build_app;

#[cfg(feature = "legacy_sim")]
use quboid_bevy::build_app;

#[cfg(not(feature = "legacy_sim"))]
use quad_bevy::build_app;

use flight_controller::{
    controllers::bf_controller::BFController, BatteryUpdate, Channels, FlightController,
    FlightControllerUpdate, GyroUpdate,
};

fn main() {
    let mut app = build_app();
    app.run();

    // let controller = BFController;
    // controller.init();
    // controller.set_armed();
    //
    // let battery_update = BatteryUpdate {
    //     bat_voltage_sag: 1.,
    //     bat_voltage: 1.,
    //     amperage: 1.,
    //     m_ah_drawn: 1.,
    //     cell_count: 4,
    // };
    //
    // let gyro_update = GyroUpdate {
    //     rotation: [0., 0., 0., 0.],
    //     acc: [0., 0., 0.],
    //     gyro: [0., 0., 0.],
    // };
    //
    // let channels = Channels {
    //     throttle: 0.5,
    //     yaw: 0.5,
    //     pitch: 0.5,
    //     roll: 0.5,
    // };
    //
    // for _ in 0..1000 {
    //     let flight_controller_update = FlightControllerUpdate {
    //         battery_update,
    //         gyro_update,
    //         channels,
    //     };
    //     let motor_input = controller.update(flight_controller_update);
    //     println!("motor input: {motor_input:?}");
    // }
}

#[cfg(test)]
mod test {
    use flight_controller::{
        controllers::bf_controller::BFController, BatteryUpdate, Channels, FlightController,
        FlightControllerUpdate, GyroUpdate,
    };

    #[test]
    fn bf_controller() {
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
            rotation: [0., 0., 0., 0.],
            acc: [0., 0., 0.],
            gyro: [0., 0., 0.],
        };

        let channels = Channels {
            throttle: 0.5,
            yaw: 0.5,
            pitch: 0.5,
            roll: 0.5,
        };

        for _ in 0..10 {
            let flight_controller_update = FlightControllerUpdate {
                battery_update,
                gyro_update,
                channels,
            };
            controller.update(flight_controller_update);
        }
    }
}
