use std::ops::{Deref, DerefMut};

use bevy::prelude::{Component, Query};
use bevy_egui::{egui::Window, EguiContexts};
use egui_extras::{Column, TableBuilder};
use simulator::SimulationDebugInfo;

#[derive(Component, Default)]
pub struct UiInfo(pub SimulationDebugInfo);

impl Deref for UiInfo {
    type Target = SimulationDebugInfo;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for UiInfo {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

// Displays all the debug information we need during the simulation
pub fn update_ui(mut ctx: EguiContexts, sim_info: Query<&UiInfo>) {
    let ui_sim_info = sim_info.single();
    Window::new("Simulation info").show(ctx.ctx_mut(), |ui| {
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
                                ui.label(format!("{}", $column_value));
                            });
                        });
                    };
                }
                // Display all the data that we want to show
                // TODO: maybe readd this
                // display_debug_data!("Determinant", ui_sim_info.rotation.determinant());
                display_debug_data!("Rotation matrix", ui_sim_info.rotation);
                display_debug_data!("Velocity", ui_sim_info.linear_velocity);
                display_debug_data!("Acceleration", ui_sim_info.acceleration);
                display_debug_data!("Angular velocity", ui_sim_info.angular_velocity);
                display_debug_data!("Motor thrust {}", ui_sim_info.angular_velocity, 4);
                display_debug_data!("Motor rpm {}", ui_sim_info.rpms, 4);
                display_debug_data!("Motor pwm {}", ui_sim_info.pwms, 4);
                display_debug_data!("Bat voltage", ui_sim_info.bat_voltage);
                display_debug_data!("Bat voltage sag", ui_sim_info.bat_voltage_sag);
            });
    });
}
