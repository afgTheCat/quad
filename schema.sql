-- TODO: drone model, simulation model etc

-- flight log table
CREATE TABLE IF NOT EXISTS flight_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    simulation_id TEXT,
    start_seconds REAL,
    end_seconds REAL,
    motor_input_1 REAL,
    motor_input_2 REAL,
    motor_input_3 REAL,
    motor_input_4 REAL,
    battery_voltage_sag REAL,
    battery_voltage REAL,
    amperage REAL,
    mah_drawn REAL,
    cell_count INTEGER,
    rot_quat_x REAL,
    rot_quat_y REAL,
    rot_quat_z REAL,
    rot_quat_w REAL,
    linear_acceleration_x REAL,
    linear_acceleration_y REAL,
    linear_acceleration_z REAL,
    angular_velocity_x REAL,
    angular_velocity_y REAL,
    angular_velocity_z REAL,
    throttle REAL,
    roll REAL,
    pitch REAL,
    yaw REAL
);
