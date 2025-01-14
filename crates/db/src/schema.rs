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

diesel::table! {
    sample_curve(id) {
        id -> BigInt,
        min_discharge_point -> Double,
        max_discharge_point -> Double
    }
}

diesel::table! {
    sample_point(id) {
        id -> BigInt,
        sample_curve_id -> BigInt, // references sample curve
        discharge -> Double,
        voltage -> Double
    }
}

diesel::table! {
    low_pass_filter(id) {
        id -> BigInt,
        output -> Double,
        e_pow -> Double
    }
}

diesel::table! {
    rotor_state(id) {
        id -> BigInt,
        current -> Double,
        rpm -> Double,
        motor_torque -> Double,
        effective_thrust -> Double,
        pwm -> Double,
        rotor_dir -> Double,
        motor_pos_x -> Double,
        motor_pos_y -> Double,
        motor_pos_z -> Double,
        pwm_low_pass_filter -> BigInt, // references low pass filter
    }
}

diesel::table! {
    simulation_frame(id) {
        id -> BigInt,
        capacity -> Double,
        bat_voltage -> Double,
        bat_voltage_sag -> Double,
        amperage -> Double,
        m_ah_drawn -> Double,
        rotor_1_state -> BigInt, // references rotor state
        rotor_2_state -> BigInt, // references rotor state
        rotor_3_state -> BigInt, // references rotor state
        rotor_4_state -> BigInt, // references rotor state

        position_x -> Double,
        position_y -> Double,
        position_z -> Double,

        rotation_x -> Double,
        rotation_y -> Double,
        rotation_z -> Double,
        rotation_w -> Double,

        linear_velocity_x -> Double,
        linear_velocity_y -> Double,
        linear_velocity_z -> Double,

        angular_velocity_x -> Double,
        angular_velocity_y -> Double,
        angular_velocity_z -> Double,

        acceleration_x -> Double,
        acceleration_y -> Double,
        acceleration_z -> Double,
    }
}

diesel::table! {
    drone_model(id) {
        id -> BigInt,
        quad_bat_capacity -> Double,
        bat_voltage_curve -> BigInt, // references sample curve
        quad_bat_cell_count -> BigInt,
        quad_bat_capacity_charged -> Double,
        max_voltage_sag -> Double,
        prop_max_rpm -> Double,
        motor_1_lpf -> BigInt,
        motor_2_lpf -> BigInt,
        motor_3_lpf -> BigInt,
        motor_4_lpf -> BigInt,
        motor_kv -> Double,
        motor_r -> Double,
        motor_io -> Double,
        prop_thrust_factor1 -> Double,
        prop_thrust_factor2 -> Double,
        prop_thrust_factor3 -> Double,
        prop_torque_factor -> Double,
        prop_a_factor -> Double,
        prop_inertia -> Double,
        frame_drag_area1 -> Double,
        frame_drag_area2 -> Double,
        frame_drag_area3 -> Double,
        frame_drag_constant -> Double,
        mass -> Double,
        inv_tensor_diag1 -> Double,
        inv_tensor_diag2 -> Double,
        inv_tensor_diag3 -> Double,
    }
}

// diesel::joinable!(rotor_state -> low_pass_filter (pwm_low_pass_filter));
// diesel::joinable!(simulation_frame -> rotor_state (rotor_1_state));
// diesel::joinable!(simulation_frame -> rotor_state (rotor_2_state));
// diesel::joinable!(simulation_frame -> rotor_state (rotor_3_state));
// diesel::joinable!(simulation_frame -> rotor_state (rotor_4_state));

// diesel::allow_tables_to_appear_in_same_query!(low_pass_filter, rotor_state, simulation_frame,);
