pub mod menu;
pub mod replay;
pub mod simulation;

use crate::{
    sim::SimulationData,
    ui::menu::{menu_ui, UIState},
    Context, VisualizerState,
};
use bevy::prelude::{NextState, Res, ResMut, State};
use bevy_egui::EguiContexts;
use replay::replay_ui;
use simulation::simulation_ui;

pub fn draw_ui(
    ctx: EguiContexts,
    mut context: ResMut<Context>,
    mut ui_state: ResMut<UIState>,
    state: Res<State<VisualizerState>>,
    sim_data: Option<Res<SimulationData>>,
    next_visualizer_state: ResMut<NextState<VisualizerState>>,
) {
    let state = (*state).clone();
    match state {
        VisualizerState::Loading => {} // no ui to show
        VisualizerState::Menu => {
            menu_ui(
                ctx,
                ui_state.as_mut(),
                context.as_mut(),
                next_visualizer_state,
            );
        }
        VisualizerState::Simulation => {
            if let Some(sim_data) = sim_data {
                simulation_ui(ctx, sim_data.as_ref(), next_visualizer_state);
            }
        }
        VisualizerState::Replay => {
            replay_ui(ctx, next_visualizer_state);
        }
    }
}
