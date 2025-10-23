pub mod menu;
pub mod replay;
pub mod simulation;

use crate::{sim::SimulationData, Loader, VisualizerData, VisualizerState};
use bevy::prelude::{NextState, Res, ResMut, State};
use bevy_egui::EguiContexts;
use db2::LoaderTrait;
use menu::menu_ui;
use replay::replay_ui;
use simulation::simulation_ui;

pub fn prefetch_menu_items(mut sim_data: ResMut<VisualizerData>, db: Res<Loader>) {
    sim_data.simulation_ids = db.get_simulation_ids();
    sim_data.reservoir_ids = db.get_reservoir_ids();
}

pub fn draw_ui(
    ctx: EguiContexts,
    state: Res<State<VisualizerState>>,
    vis_data: Option<ResMut<VisualizerData>>,
    sim_data: Option<Res<SimulationData>>,
    next_visualizer_state: ResMut<NextState<VisualizerState>>,
) {
    let state = (*state).clone();
    match state {
        VisualizerState::Loading => {} // no ui to show
        VisualizerState::Menu => {
            if let Some(mut vis_data) = vis_data {
                menu_ui(ctx, vis_data.as_mut(), next_visualizer_state);
            }
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
