use crate::VisualizerState;
use bevy::prelude::{NextState, Res, ResMut, Resource, State};
use bevy_egui::{
    egui::{self, Ui, Window as EguiWindow},
    EguiContexts,
};
use egui_extras::{Column, TableBuilder};
use simulator::SimulationDebugInfo;

#[derive(Resource, Default)]
pub struct UiData {
    pub sim_info: SimulationDebugInfo,
}

impl UiData {
    pub fn set_sim_info(&mut self, sim_info: SimulationDebugInfo) {
        self.sim_info = sim_info;
    }
}

fn menu_toggle(ui: &mut Ui, mut next_visualizer_state: ResMut<NextState<VisualizerState>>) {
    if ui.button("Live").clicked() {
        next_visualizer_state.set(VisualizerState::SimulationInit);
    }
    if ui.button("Replayer").clicked() {
        next_visualizer_state.set(VisualizerState::ReplayInit);
    }
}

pub fn draw_ui(
    mut ctx: EguiContexts,
    state: Res<State<VisualizerState>>,
    ui_data: ResMut<UiData>,
    next_visualizer_state: ResMut<NextState<VisualizerState>>,
) {
    let state = (*state).clone();
    let window = EguiWindow::new("Visualizer")
        .default_size(egui::vec2(960f32, 540f32))
        .resizable(true);

    window.show(ctx.ctx_mut(), |ui| {
        menu_toggle(ui, next_visualizer_state);
        // probably there is a better way
        match state {
            VisualizerState::Menu => {}
            VisualizerState::SimulationInit | VisualizerState::Simulation => {
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
                                        ui.label(format!("{:?}", $column_value));
                                    });
                                });
                            };
                        }
                        // Display all the data that we want to show
                        // TODO: maybe readd this
                        // display_debug_data!("Determinant", ui_data.sim_info.rotation.determinant());
                        display_debug_data!("Rotation matrix", ui_data.sim_info.rotation);
                        display_debug_data!("Velocity", ui_data.sim_info.linear_velocity);
                        display_debug_data!("Acceleration", ui_data.sim_info.acceleration);
                        display_debug_data!("Angular velocity", ui_data.sim_info.angular_velocity);
                        display_debug_data!("Motor thrust {}", ui_data.sim_info.thrusts, 4);
                        display_debug_data!("Motor rpm {}", ui_data.sim_info.rpms, 4);
                        display_debug_data!("Motor pwm {}", ui_data.sim_info.pwms, 4);
                        display_debug_data!("Bat voltage", ui_data.sim_info.bat_voltage);
                        display_debug_data!("Bat voltage sag", ui_data.sim_info.bat_voltage_sag);
                    });

                // TODO: switch state here
            }
            VisualizerState::Replay | VisualizerState::ReplayInit => {}
        }
        ui.allocate_space(ui.available_size());
    });
}
