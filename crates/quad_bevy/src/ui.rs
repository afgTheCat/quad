use bevy::{
    math::{DMat3, DVec3, DVec4},
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
    angular_velocity: DVec3,
    motor_thrusts: DVec4,
    motor_rpm: DVec4,
}

impl UiSimulationInfo {
    pub fn update_state(
        &mut self,
        rotation_marix: DMat3,
        position: DVec3,
        velocity: DVec3,
        acceleration: DVec3,
        angular_velocity: DVec3,
        motor_thrusts: DVec4,
        motor_rpm: DVec4,
    ) {
        self.rotation_matrix = rotation_marix;
        self.position = position;
        self.velocity = velocity;
        self.acceleration = acceleration;
        self.angular_velocity = angular_velocity;
        self.motor_thrusts = motor_thrusts;
        self.motor_rpm = motor_rpm;
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
                body.row(30.0, |mut row| {
                    row.col(|ui| {
                        ui.label("Angular velocity");
                    });
                    row.col(|ui| {
                        ui.label(format!("{}", ui_sim_info.angular_velocity));
                    });
                });
                for i in 0..4 {
                    body.row(30.0, |mut row| {
                        row.col(|ui| {
                            ui.label(format!("Motor thrust {i}"));
                        });
                        row.col(|ui| {
                            ui.label(format!("{}", ui_sim_info.motor_thrusts[i]));
                        });
                    });
                }

                for i in 0..4 {
                    body.row(30.0, |mut row| {
                        row.col(|ui| {
                            ui.label(format!("Motor rpm {i}"));
                        });
                        row.col(|ui| {
                            ui.label(format!("{}", ui_sim_info.motor_rpm[i]));
                        });
                    });
                }
            });
    });
}
