use super::rigid_body::{
    angular_velocity, cube_from_inertia, inertia_cuboid_diag, CubeRigidBody, SimulationContext,
};
use bevy::{
    asset::Assets,
    input::{keyboard::KeyboardInput, ButtonState},
    math::{DMat3, DVec3, Quat, Vec3},
    prelude::{Camera, Component, EventReader, KeyCode, Mesh, Query, ResMut, Transform},
};
use bevy_egui::{
    egui::{self},
    EguiContexts,
};

pub fn handle_keyboard_events(
    mut camera_query: Query<(&mut Camera, &mut Transform)>,
    mut evr_kbd: EventReader<KeyboardInput>,
) {
    let mut moving_dir = Vec3::new(0., 0., 0.);
    for ev in evr_kbd.read() {
        match (ev.key_code, ev.state) {
            (KeyCode::ArrowLeft, ButtonState::Pressed) => moving_dir[0] -= 1.,
            (KeyCode::ArrowRight, ButtonState::Pressed) => moving_dir[0] += 1.,
            (KeyCode::ArrowUp, ButtonState::Pressed) => moving_dir[2] -= 1.,
            (KeyCode::ArrowDown, ButtonState::Pressed) => moving_dir[2] += 1.,
            _ => {}
        }
    }

    let (_, mut camera_transform) = camera_query.single_mut();
    camera_transform.translation += moving_dir;
}

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
    pub angular_momentum: DVec3,
    pub inertia_matrix_diag: DVec3,
}

impl UiState {
    fn recalculate_inertia_matrix_diag(&mut self) {
        self.inertia_matrix_diag = inertia_cuboid_diag(self.sides)
    }

    fn recalculate_sides(&mut self) {
        self.sides = cube_from_inertia(self.inertia_matrix_diag).as_vec3();
    }
}

pub fn update_ui(
    mut contexts: EguiContexts,
    mut query: Query<(
        &mut CubeRigidBody,
        &mut SimulationContext,
        &mut UiState,
        &mut Transform,
    )>,
    meshes: ResMut<Assets<Mesh>>,
) {
    let (mut rigid_body, mut simulation_context, mut ui_state, mut transform) = query.single_mut();
    egui::Window::new("Sim info").show(contexts.ctx_mut(), |ui| {
        ui.label(format!("mass: {}", rigid_body.mass()));
        let inertia_diag = rigid_body.inertia_diag();
        ui.label(format!(
            "inertia diag: ({}, {}, {})",
            inertia_diag[0], inertia_diag[1], inertia_diag[2]
        ));

        ui.horizontal(|ui| {
            ui.label("Simulation running");
            toggle_ui(ui, &mut simulation_context.simulation_running);
        });

        ui.horizontal(|ui| {
            ui.label("Dialation");
            let slider = ui.add(egui::Slider::new(
                &mut simulation_context.dialation,
                0.1..=1.5,
            ));
            if slider.lost_focus() || slider.drag_stopped() {
                ui_state.recalculate_inertia_matrix_diag();
            }
        });

        ui.end_row();

        let angular_velocity = angular_velocity(
            &transform,
            rigid_body.inv_inertia_tensor,
            rigid_body.angular_momentum,
        );
        let angular_velocity_log = format!(
            "Angular velocity: {:.4}, {:.4}, {:.4}, sum: {:.4}",
            angular_velocity[0],
            angular_velocity[1],
            angular_velocity[2],
            angular_velocity.length()
        );
        ui.label(angular_velocity_log);

        if ui.add(egui::Button::new("Reset rotation")).clicked() {
            transform.rotation = Quat::IDENTITY;
            rigid_body.rotation = DMat3::IDENTITY;
        }

        let rotation_determinant = format!("rot matrix det: {}", rigid_body.rotation.determinant());
        ui.label(rotation_determinant);
    });

    egui::Window::new("Simulation cotroll").show(contexts.ctx_mut(), |ui| {
        ui.label("Simulation variables");

        ui.horizontal(|ui| {
            ui.label("Inertia x");
            let slider = ui.add(egui::Slider::new(
                &mut ui_state.inertia_matrix_diag.x,
                0.001..=1.0,
            ));
            if slider.lost_focus() || slider.drag_stopped() {
                ui_state.recalculate_sides();
            }
        });

        ui.horizontal(|ui| {
            ui.label("Inertia y");
            let slider = ui.add(egui::Slider::new(
                &mut ui_state.inertia_matrix_diag.y,
                0.001..=1.0,
            ));
            if slider.lost_focus() || slider.drag_stopped() {
                ui_state.recalculate_sides();
            }
        });

        ui.horizontal(|ui| {
            ui.label("inertia z");
            let slider = ui.add(egui::Slider::new(
                &mut ui_state.inertia_matrix_diag.z,
                0.001..=1.0,
            ));
            if slider.lost_focus() || slider.drag_stopped() {
                ui_state.recalculate_sides();
            }
        });

        ui.horizontal(|ui| {
            ui.label("X length");
            let slider = ui.add(egui::Slider::new(&mut ui_state.sides.x, 0.1..=10.0));
            if slider.lost_focus() || slider.drag_stopped() {
                ui_state.recalculate_inertia_matrix_diag();
            }
        });

        ui.horizontal(|ui| {
            ui.label("Y length");
            let slider = ui.add(egui::Slider::new(&mut ui_state.sides.y, 0.1..=10.0));
            if slider.lost_focus() || slider.drag_stopped() {
                ui_state.recalculate_inertia_matrix_diag();
            }
        });

        ui.horizontal(|ui| {
            ui.label("Z length");
            let slider = ui.add(egui::Slider::new(&mut ui_state.sides.z, 0.1..=10.0));
            if slider.lost_focus() || slider.drag_stopped() {
                ui_state.recalculate_inertia_matrix_diag();
            }
        });

        ui.horizontal(|ui| {
            ui.label("X anular momentum");
            ui.add(egui::Slider::new(
                &mut ui_state.angular_momentum.x,
                0.0..=10.0,
            ));
        });

        ui.horizontal(|ui| {
            ui.label("Y anular momentum");
            ui.add(egui::Slider::new(
                &mut ui_state.angular_momentum.y,
                0.0..=10.0,
            ));
        });

        ui.horizontal(|ui| {
            ui.label("Z anular momentum");
            ui.add(egui::Slider::new(
                &mut ui_state.angular_momentum.z,
                0.0..=10.0,
            ));
        });

        if ui.add(egui::Button::new("Submit")).clicked() {
            rigid_body.set_rigid_body_props(ui_state.sides, ui_state.angular_momentum, meshes);
        }
    });
}
