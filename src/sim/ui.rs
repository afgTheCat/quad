use bevy::prelude::{Query, Transform};
use bevy_egui::{
    egui::{Grid, Window},
    EguiContexts,
};

use super::drone::Drone;

struct UiState {}

pub fn update_ui(mut ctx: EguiContexts, mut query: Query<(&mut Transform, &mut Drone)>) {
    Window::new("Simulation info").show(ctx.ctx_mut(), |ui| {
        Grid::new("Drone info").show(ui, |ui| {
            ui.label("First row, first column");
            ui.label("First row, second column");
            ui.end_row();
        });
        // TableBuilder::new(ui)
        //     .column(Column::auto().resizable(true))
        //     .column(Column::remainder())
        //     .header(20.0, |mut header| {
        //         header.col(|ui| {
        //             ui.heading("First column");
        //         });
        //         header.col(|ui| {
        //             ui.heading("Second column");
        //         });
        //     })
        //     .body(|mut body| {
        //         body.row(30.0, |mut row| {
        //             row.col(|ui| {
        //                 ui.label("Hello");
        //             });
        //             row.col(|ui| {
        //                 ui.button("world!");
        //             });
        //         });
        //     });
    });
}
