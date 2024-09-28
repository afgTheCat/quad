use bevy::{
    asset::{Assets, Handle},
    math::Vec3,
    prelude::{Component, Cuboid, Mesh, Query, ResMut},
};
use bevy_egui::{
    egui::{self},
    EguiContexts,
};

use crate::rigid_body::{RigidBody, SimulationContext};

fn toggle_ui(ui: &mut egui::Ui, on: &mut bool) -> egui::Response {
    let desired_size = ui.spacing().interact_size.y * egui::vec2(2.0, 1.0);
    let (rect, mut response) = ui.allocate_exact_size(desired_size, egui::Sense::click());
    if response.clicked() {
        *on = !*on;
        response.mark_changed();
    }
    response.widget_info(|| {
        egui::WidgetInfo::selected(egui::WidgetType::Checkbox, ui.is_enabled(), *on, "")
    });

    if ui.is_rect_visible(rect) {
        let how_on = ui.ctx().animate_bool_responsive(response.id, *on);
        let visuals = ui.style().interact_selectable(&response, *on);
        let rect = rect.expand(visuals.expansion);
        let radius = 0.5 * rect.height();
        ui.painter()
            .rect(rect, radius, visuals.bg_fill, visuals.bg_stroke);
        let circle_x = egui::lerp((rect.left() + radius)..=(rect.right() - radius), how_on);
        let center = egui::pos2(circle_x, rect.center().y);
        ui.painter()
            .circle(center, 0.75 * radius, visuals.bg_fill, visuals.fg_stroke);
    }

    response
}

#[derive(Component)]
pub struct UiState {
    pub sides: Vec3,
    pub angular_momentum: Vec3,
}

pub fn ui(
    mut contexts: EguiContexts,
    mut query: Query<(&mut RigidBody, &mut SimulationContext, &mut UiState)>,
    meshes: ResMut<Assets<Mesh>>,
) {
    let (mut rigid_body, mut simulation_context, mut ui_state) = query.single_mut();
    egui::Window::new("Simulation cotroll").show(contexts.ctx_mut(), |ui| {
        ui.horizontal(|ui| {
            ui.label("Simulation running");
            toggle_ui(ui, &mut simulation_context.simulation_running);
        });
        ui.end_row();

        ui.label("Simulation variables");

        ui.horizontal(|ui| {
            ui.label("X length");
            ui.add(egui::Slider::new(&mut ui_state.sides.x, 0.1..=10.0));
        });

        ui.horizontal(|ui| {
            ui.label("Y length");
            ui.add(egui::Slider::new(&mut ui_state.sides.y, 0.1..=10.0));
        });

        ui.horizontal(|ui| {
            ui.label("Z length");
            ui.add(egui::Slider::new(&mut ui_state.sides.z, 0.1..=10.0));
        });

        ui.horizontal(|ui| {
            ui.label("X anular momentum");
            ui.add(egui::Slider::new(
                &mut ui_state.angular_momentum.x,
                0.1..=10.0,
            ));
        });

        ui.horizontal(|ui| {
            ui.label("Y anular momentum");
            ui.add(egui::Slider::new(
                &mut ui_state.angular_momentum.y,
                0.1..=10.0,
            ));
        });

        ui.horizontal(|ui| {
            ui.label("Z anular momentum");
            ui.add(egui::Slider::new(
                &mut ui_state.angular_momentum.z,
                0.1..=10.0,
            ));
        });

        if ui.add(egui::Button::new("Submit")).clicked() {
            rigid_body.reset_rigid_body(ui_state.sides, ui_state.angular_momentum, meshes);
        }
    });
}
