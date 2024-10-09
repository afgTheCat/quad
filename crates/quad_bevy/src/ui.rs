use bevy::{
    math::{DMat3, DVec3},
    prelude::{Component, Query, Transform},
};
use bevy_egui::{egui::Window, EguiContexts};
use egui_extras::{Column, TableBuilder};

#[derive(Component, Default)]
pub struct UiSimulationInfo {
    rotation_matrix: DMat3,
    position: DVec3,
    velocity: DVec3,
    acceleration: DVec3,
}

impl UiSimulationInfo {
    pub fn update_state(
        &mut self,
        rotation_marix: DMat3,
        position: DVec3,
        velocity: DVec3,
        acceleration: DVec3,
    ) {
        self.rotation_matrix = rotation_marix;
        self.position = position;
        self.velocity = velocity;
        self.acceleration = acceleration;
    }
}

pub fn update_ui(mut ctx: EguiContexts, mut query: Query<&UiSimulationInfo>) {
    let ui_sim_info = query.single();
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
                body.row(30.0, |mut row| {
                    row.col(|ui| {
                        ui.label("Determinant");
                    });
                    row.col(|ui| {
                        ui.label(format!("{}", ui_sim_info.rotation_matrix.determinant()));
                    });
                });
                body.row(30.0, |mut row| {
                    row.col(|ui| {
                        ui.label("Rotation matrix");
                    });
                    row.col(|ui| {
                        ui.label(format!("{}", ui_sim_info.rotation_matrix));
                    });
                });
                body.row(30.0, |mut row| {
                    row.col(|ui| {
                        ui.label("Velocity");
                    });
                    row.col(|ui| {
                        ui.label(format!("{}", ui_sim_info.velocity));
                    });
                });
                body.row(30.0, |mut row| {
                    row.col(|ui| {
                        ui.label("Acceleration");
                    });
                    row.col(|ui| {
                        ui.label(format!("{}", ui_sim_info.acceleration));
                    });
                });
            });
    });
}
