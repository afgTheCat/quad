use crate::VisualizerState;
use bevy::prelude::{NextState, ResMut};
use bevy_egui::{egui, EguiContexts};

pub fn replay_ui(
    mut ctx: EguiContexts,
    mut next_visualizer_state: ResMut<NextState<VisualizerState>>,
) {
    egui::SidePanel::left("Visualizer").show(ctx.ctx_mut(), |ui| {
        if ui.button("Back to menu").clicked() {
            next_visualizer_state.set(VisualizerState::Menu);
        }
    });
}
