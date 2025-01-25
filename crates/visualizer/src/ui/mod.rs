use crate::{VisualizerData, VisualizerState, DB};
use bevy::prelude::{NextState, Res, ResMut, State};
use bevy_egui::EguiContexts;
use menu::menu_ui;
use replay::replay_ui;
use simulation::simulation_ui;

pub mod menu;
pub mod replay;
pub mod simulation;

pub fn prefetch_menu_items(mut sim_data: ResMut<VisualizerData>, db: Res<DB>) {
    sim_data.simulation_ids = db.select_simulation_ids();
    sim_data.reservoir_ids = db.select_reservoir_ids();
}

pub fn draw_ui(
    ctx: EguiContexts,
    state: Res<State<VisualizerState>>,
    sim_data: ResMut<VisualizerData>,
    next_visualizer_state: ResMut<NextState<VisualizerState>>,
) {
    let state = (*state).clone();
    match state {
        VisualizerState::Loading => {} // no ui to show
        VisualizerState::Menu => {
            menu_ui(ctx, sim_data, next_visualizer_state);
        }
        VisualizerState::Simulation => {
            simulation_ui(ctx, sim_data, next_visualizer_state);
        }
        VisualizerState::Replay => {
            replay_ui(ctx, next_visualizer_state);
        }
    }
}
