use bevy::prelude::{NextState, ResMut};
use bevy_egui::{
    egui::{
        self, epaint::RectShape, pos2, Color32, Frame, Rect, Rounding, Shape, Stroke, Ui, UiBuilder,
    },
    EguiContexts,
};

use crate::{VisualizerData, VisualizerState};

#[derive(Clone, Eq, PartialEq, Hash, Debug)]
pub enum Logger {
    DBLogger,
    Rerun,
    NullLogger,
}

impl Logger {
    fn to_str(&self) -> String {
        match self {
            Self::DBLogger => "DB logger".into(),
            Self::Rerun => "Rerun".into(),
            Self::NullLogger => "NullLogger".into(),
        }
    }
}

#[derive(Clone, Eq, PartialEq, Hash, Debug, Default)]
pub enum Controller {
    #[default]
    Betafligt,
    Reservoir(String), // reservoir controller id
}

impl Controller {
    fn to_str(&self) -> String {
        match self {
            Self::Betafligt => "Betaflight".into(),
            Self::Reservoir(res_id) => format!("Reservoir controller {}", res_id),
        }
    }
}

#[derive(Debug, Default, Clone)]
pub enum SelectionConfig {
    #[default]
    Menu,
    Replay {
        replay_id: Option<String>,
        controller: Option<Controller>,
    },
    Simulation {
        logger: Option<Logger>,
        controller: Option<Controller>,
    },
}

impl SelectionConfig {
    pub fn to_state_str(&self) -> String {
        match &self {
            Self::Menu => "Menu".into(),
            Self::Replay { .. } => "Replay".into(),
            Self::Simulation { .. } => "Simulation".into(),
        }
    }

    pub fn to_controller_str(&self) -> String {
        match &self {
            Self::Menu => "No controller selected".into(),
            Self::Replay { controller, .. } | Self::Simulation { controller, .. } => {
                match controller {
                    Some(c) => c.to_str(),
                    _ => "No controller selected".into(),
                }
            }
        }
    }

    pub fn to_logger_str(&self) -> String {
        match &self {
            Self::Menu | Self::Replay { .. } => "No logger selected".into(),
            Self::Simulation { logger, .. } => match logger {
                Some(logger) => logger.to_str(),
                _ => "No logger selected".into(),
            },
        }
    }

    pub fn to_replay_id_str(&self) -> String {
        match &self {
            Self::Menu | Self::Simulation { .. } => "No replay id selected".into(),
            Self::Replay { replay_id, .. } => match replay_id {
                Some(replay_id) => replay_id.into(),
                _ => "No logger selected".into(),
            },
        }
    }

    pub fn to_simulation(&self) -> Self {
        match self {
            Self::Menu => Self::Simulation {
                logger: None,
                controller: None,
            },
            Self::Replay { controller, .. } => Self::Simulation {
                logger: None,
                controller: controller.clone(),
            },
            _ => self.clone(),
        }
    }

    pub fn to_replay(&self) -> Self {
        match self {
            Self::Menu => Self::Replay {
                controller: None,
                replay_id: None,
            },
            Self::Replay { .. } => self.clone(),
            Self::Simulation { controller, .. } => Self::Replay {
                controller: controller.clone(),
                replay_id: None,
            },
        }
    }

    pub fn set_controller(&mut self, set_controller: Controller) {
        match self {
            Self::Replay { controller, .. } | Self::Simulation { controller, .. } => {
                *controller = Some(set_controller)
            }
            _ => unreachable!(),
        }
    }

    pub fn set_replay_id(&mut self, set_replay_id: String) {
        match self {
            Self::Replay { replay_id, .. } => *replay_id = Some(set_replay_id),
            _ => unreachable!(),
        }
    }

    pub fn set_logger(&mut self, set_logger: Logger) {
        match self {
            Self::Simulation { logger, .. } => *logger = Some(set_logger),
            _ => unreachable!(),
        }
    }
}

pub fn main_menu_toggle(
    ui: &mut Ui,
    visualizer_data: &mut VisualizerData,
    mut next_visualizer_state: ResMut<NextState<VisualizerState>>,
) {
    ui.vertical_centered(|ui| {
        ui.label(
            egui::RichText::new("ASCENT VISUALIZER")
                .heading()
                .color(egui::Color32::from_rgb(255, 255, 255)),
        )
    });

    ui.horizontal(|ui| {
        ui.label("Mode:");
        ui.menu_button(visualizer_data.selection_config.to_state_str(), |ui| {
            if ui.button("Simulation").clicked() {
                visualizer_data.selection_config = visualizer_data.selection_config.to_simulation();
            }
            if ui.button("Replay").clicked() {
                visualizer_data.selection_config = visualizer_data.selection_config.to_replay();
            }
        });
    });

    ui.horizontal(|ui| {
        ui.label("Controller:");
        if matches!(visualizer_data.selection_config, SelectionConfig::Menu) {
            ui.label("Select mode first");
        } else {
            ui.menu_button(visualizer_data.selection_config.to_controller_str(), |ui| {
                if ui.button("Betaflight").clicked() {
                    visualizer_data
                        .selection_config
                        .set_controller(Controller::Betafligt);
                }
                for res_id in visualizer_data.reservoir_ids.iter() {
                    if ui
                        .button(format!("Reservoir controller {}", res_id))
                        .clicked()
                    {
                        visualizer_data
                            .selection_config
                            .set_controller(Controller::Reservoir(res_id.into()));
                    }
                }
            });
        }
    });

    ui.horizontal(|ui| {
        ui.label("Logger:");
        let is_simulation = matches!(
            visualizer_data.selection_config,
            SelectionConfig::Simulation { .. }
        );
        if is_simulation {
            ui.menu_button(visualizer_data.selection_config.to_logger_str(), |ui| {
                if ui.button("DB logger").clicked() {
                    visualizer_data
                        .selection_config
                        .set_logger(Logger::DBLogger);
                }
                if ui.button("Rerun").clicked() {
                    visualizer_data.selection_config.set_logger(Logger::Rerun);
                }
                if ui.button("Null logger").clicked() {
                    visualizer_data
                        .selection_config
                        .set_logger(Logger::NullLogger);
                }
            });
        } else {
            ui.label("Only available in simulation mode");
        }
    });

    ui.horizontal(|ui| {
        ui.label("Replay id:");
        let is_replay = matches!(
            visualizer_data.selection_config,
            SelectionConfig::Replay { .. }
        );
        if is_replay {
            ui.menu_button(visualizer_data.selection_config.to_replay_id_str(), |ui| {
                for replay_id in visualizer_data.simulation_ids.iter() {
                    if ui.button(format!("Replay {}", replay_id)).clicked() {
                        visualizer_data
                            .selection_config
                            .set_replay_id(replay_id.into());
                    }
                }
            });
        } else {
            ui.label("Only available in replay mode");
        }
    });

    if ui.button("Apply").clicked() {
        match &visualizer_data.selection_config {
            SelectionConfig::Simulation {
                logger: Some(..),
                controller: Some(..),
            } => {
                next_visualizer_state.set(VisualizerState::Simulation);
            }
            SelectionConfig::Replay {
                replay_id: Some(..),
                controller: Some(..),
            } => {
                next_visualizer_state.set(VisualizerState::Replay);
            }
            _ => {
                // We should warn of incomplete config
            }
        }
    }
}

pub fn menu_ui(
    mut ctx: EguiContexts,
    sim_data: &mut VisualizerData,
    next_visualizer_state: ResMut<NextState<VisualizerState>>,
) {
    let frame = Frame {
        fill: Color32::TRANSPARENT,
        ..Default::default()
    };

    egui::CentralPanel::default()
        .frame(frame)
        .show(ctx.ctx_mut(), |ui| {
            let available_height = ui.available_height();
            let available_width = ui.available_width();

            let switch_box_min = pos2(available_width / 4., available_height / 4.);
            let switch_box_max = pos2(available_width * 3. / 4., available_height * 3. / 4.);
            let switch_box = Rect::from_min_max(switch_box_min, switch_box_max);
            ui.painter().add(Shape::Rect(RectShape::new(
                switch_box,
                Rounding::default(),
                Color32::from_rgb(59, 59, 59),
                Stroke::new(1., Color32::from_rgb(127, 0, 255)),
            )));

            let info_box_min = pos2(available_width / 4. + 10., available_height / 4. + 10.);
            let info_box_max = pos2(
                available_width * 3. / 4. - 10.,
                available_height * 3. / 4. - 10.,
            );
            let info_box = Rect::from_min_max(info_box_min, info_box_max);
            ui.allocate_new_ui(UiBuilder::new().max_rect(info_box), |ui| {
                main_menu_toggle(ui, sim_data, next_visualizer_state);
            });
        });
}
