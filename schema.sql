-- TODO: drone model, simulation model etc

-- flight log table
CREATE TABLE IF NOT EXISTS flight_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    simulation_id TEXT NOT NULL,
    start_seconds REAL NOT NULL,
    end_seconds REAL NOT NULL,
    motor_input_1 REAL NOT NULL,
    motor_input_2 REAL NOT NULL,
    motor_input_3 REAL NOT NULL,
    motor_input_4 REAL NOT NULL
);

