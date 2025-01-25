use crate::{sim::SimulationData, VisualizerState};
use bevy::prelude::{NextState, Res, ResMut};
use bevy_egui::{egui, EguiContexts};
use egui_extras::{Column, TableBuilder};

pub fn simulation_ui(
    mut ctx: EguiContexts,
    sim_data: &SimulationData,
    mut next_visualizer_state: ResMut<NextState<VisualizerState>>,
) {
    // let side_panel = egui::SidePanel::left("Simulation").default_width(100.);
    egui::SidePanel::left("Simulation")
        // .default_width(500.)
        .min_width(300.)
        .show(ctx.ctx_mut(), |ui| {
            if ui.button("Back to menu").clicked() {
                next_visualizer_state.set(VisualizerState::Menu);
            }

            // TODO: make this more modern
            TableBuilder::new(ui)
            .column(Column::auto().resizable(true))
            .column(Column::remainder())
            .header(20.0, |mut header| {
                header.col(|ui| {
                    ui.heading("Name");
                });
                header.col(|ui| {
                    ui.heading("Data");
                });
            })
            .body(|mut body| {
                // Rust macros are hygenic, so we need to declare the macro in the scope where body
                // is already defined
                macro_rules! display_debug_data {
                    ($column_template:literal, $column_value:expr, $column_data_length:literal) => {
                        for i in 0..$column_data_length {
                            let column_name = format!($column_template, i);
                            display_debug_data!(column_name, $column_value[i]);
                        }
                    };

                    ($column_name:expr, $column_value:expr) => {
                        body.row(30.0, |mut row| {
                            row.col(|ui| {
                                ui.label($column_name);
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.2?}", $column_value));
                            });
                        });
                    };
                }
                // Display all the data that we want to show
                // TODO: maybe readd this
                // display_debug_data!("Determinant", ui_data.sim_info.rotation.determinant());

                // display_debug_data!("Rotation matrix", sim_data.sim_info.rotation);
                display_debug_data!("Velocity", sim_data.sim_info.linear_velocity);
                display_debug_data!("Acceleration", sim_data.sim_info.acceleration);
                display_debug_data!("Angular velocity", sim_data.sim_info.angular_velocity);
                display_debug_data!("Motor thrust {}", sim_data.sim_info.thrusts, 4);
                display_debug_data!("Motor rpm {}", sim_data.sim_info.rpms, 4);
                display_debug_data!("Motor pwm {}", sim_data.sim_info.pwms, 4);
                display_debug_data!("Bat voltage", sim_data.sim_info.bat_voltage);
                display_debug_data!("Bat voltage sag", sim_data.sim_info.bat_voltage_sag);
                display_debug_data!("Thrust", sim_data.sim_info.bat_voltage_sag);

                display_debug_data!("Throttle", sim_data.channels.throttle);
                display_debug_data!("Yaw", sim_data.channels.yaw);
                display_debug_data!("Pitch", sim_data.channels.pitch);
                display_debug_data!("Roll", sim_data.channels.roll);
            });
        });
}
