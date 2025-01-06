// Flight log table
diesel::table! {
    flight_log (id) {
        id -> BigInt,
        simulation_id -> Text,
        start_seconds -> Double,
        end_seconds -> Double,
        motor_input_1 -> Double,
        motor_input_2 -> Double,
        motor_input_3 -> Double,
        motor_input_4 -> Double,
        battery_voltage_sag -> Double,
        battery_voltage -> Double,
        amperage -> Double,
        mah_drawn -> Double,
        cell_count -> BigInt,
        rot_quat_x -> Double,
        rot_quat_y -> Double,
        rot_quat_z -> Double,
        rot_quat_w -> Double,
        linear_acceleration_x -> Double,
        linear_acceleration_y -> Double,
        linear_acceleration_z -> Double,
        angular_velocity_x -> Double,
        angular_velocity_y -> Double,
        angular_velocity_z -> Double,
        throttle -> Double,
        roll -> Double,
        pitch -> Double,
        yaw -> Double,
    }
}

diesel::table! {
    rc_model (id) {
        id -> BigInt,
        rc_id -> Text,
        n_internal_units -> BigInt,
        input_scaling -> Double,
        internal_weights -> Text,
        input_weights -> Nullable<Text>,
        alpha -> Double,
        readout_coeff -> Nullable<Text>,
        readout_intercept -> Nullable<Text>
    }
}
