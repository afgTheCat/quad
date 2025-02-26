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
    DB,
    Rerun,
    Null,
}

#[derive(Clone, Eq, PartialEq, Hash, Debug, Default)]
pub enum Controller {
    #[default]
    Betafligt,
    Reservoir(String), // reservoir controller id
    NullController,
}

#[derive(Debug, Default, Clone, PartialEq)]
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
        let selection_config = &mut visualizer_data.selection_config;
        egui::ComboBox::from_id_salt("Mode selector")
            .selected_text(selection_config.to_state_str())
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    selection_config,
                    selection_config.to_simulation(),
                    "Simulation",
                );
                ui.selectable_value(selection_config, selection_config.to_replay(), "Replay");
            });
    });

    ui.horizontal(|ui| {
        ui.label("Controller:");
        match &mut visualizer_data.selection_config {
            SelectionConfig::Simulation { controller, .. }
            | SelectionConfig::Replay { controller, .. } => {
                let label = match controller {
                    Some(x) => format!("{x:?}"),
                    None => "Not selected".to_string(),
                };
                egui::ComboBox::from_id_salt("Controller selector")
                    .selected_text(label)
                    .show_ui(ui, |ui| {
                        ui.selectable_value(controller, Some(Controller::Betafligt), "Betaflight");
                        for res_id in visualizer_data.reservoir_ids.iter() {
                            ui.selectable_value(
                                controller,
                                Some(Controller::Reservoir(res_id.into())),
                                format!("Resrevoid controller {}", res_id),
                            );
                        }
                        ui.selectable_value(controller, Some(Controller::NullController), "Null");
                    });
            }
            _ => {
                ui.label("Select mode first");
            }
        }
    });

    ui.horizontal(|ui| {
        ui.label("Logger:");
        if let SelectionConfig::Simulation { logger, .. } = &mut visualizer_data.selection_config {
            let label = match logger {
                Some(x) => format!("{x:?}"),
                None => "Not selected".to_string(),
            };
            egui::ComboBox::from_id_salt("Logger selector")
                .selected_text(label)
                .show_ui(ui, |ui| {
                    ui.selectable_value(logger, Some(Logger::DB), "DB logger");
                    ui.selectable_value(logger, Some(Logger::Rerun), "Rerun");
                    ui.selectable_value(logger, Some(Logger::Null), "None");
                });
        } else {
            ui.label("Only available in simulation mode");
        }
    });

    ui.horizontal(|ui| {
        ui.label("Replay id:");
        if let SelectionConfig::Replay { replay_id, .. } = &mut visualizer_data.selection_config {
            let label = match replay_id {
                Some(x) => format!("Replay: {}", x),
                None => "Not selected".to_string(),
            };
            egui::ComboBox::from_id_salt("Replay selector")
                .selected_text(label)
                .show_ui(ui, |ui| {
                    for id in visualizer_data.simulation_ids.iter() {
                        ui.selectable_value(replay_id, Some(id.into()), format!("Replay {}", id));
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
