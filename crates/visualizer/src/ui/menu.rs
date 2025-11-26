use crate::VisualizerState;
use bevy::{
    ecs::{resource::Resource, system::ResMut},
    state::state::NextState,
};
use bevy_egui::{
    egui::{
        self, epaint::RectShape, pos2, Color32, CornerRadius, Frame, Rect, Shape, Stroke, Ui,
        UiBuilder,
    },
    EguiContexts,
};
use sim_context::{ControllerType, LoaderType, LoggerType, SimContext};

#[derive(Resource, Clone, PartialEq)]
pub enum UIState {
    Replay {
        replay_id: Option<String>,
        controller: ControllerType,
        loader: LoaderType,
    },
    Simulation {
        logger: LoggerType,
        controller: ControllerType,
        loader: LoaderType,
        simulation_name: String,
    },
}

impl Default for UIState {
    fn default() -> Self {
        Self::Simulation {
            logger: LoggerType::default(),
            controller: ControllerType::default(),
            loader: LoaderType::default(),
            simulation_name: Default::default(),
        }
    }
}

impl UIState {
    pub fn to_state_str(&self) -> String {
        match &self {
            Self::Replay { .. } => "Replay".into(),
            Self::Simulation { .. } => "Simulation".into(),
        }
    }

    pub fn to_simulation(&self) -> Self {
        match self {
            Self::Replay {
                controller, loader, ..
            } => Self::Simulation {
                logger: LoggerType::default(),
                controller: controller.clone(),
                loader: loader.clone(),
                simulation_name: Default::default(),
            },
            _ => self.clone(),
        }
    }

    pub fn to_replay(&self) -> Self {
        match self {
            Self::Replay { .. } => self.clone(),
            Self::Simulation {
                controller, loader, ..
            } => Self::Replay {
                controller: controller.clone(),
                replay_id: None,
                loader: loader.clone(),
            },
        }
    }
}

pub fn main_menu_toggle(
    ui: &mut Ui,
    ui_state: &mut UIState,
    context: &mut SimContext,
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
        egui::ComboBox::from_id_salt("Mode selector")
            .selected_text(ui_state.to_state_str())
            .show_ui(ui, |ui| {
                ui.selectable_value(ui_state, ui_state.to_simulation(), "Simulation");
                ui.selectable_value(ui_state, ui_state.to_replay(), "Replay");
            });
    });

    ui.horizontal(|ui| {
        ui.label("Simulation name:");
        if let UIState::Simulation {
            simulation_name, ..
        } = ui_state
        {
            ui.text_edit_singleline(simulation_name);
        } else {
            ui.label("Only available in simulation mode");
        }
    });

    ui.horizontal(|ui| {
        ui.label("Controller:");
        match ui_state {
            UIState::Simulation { controller, .. } => {
                let label = format!("{controller:?}");
                egui::ComboBox::from_id_salt("Controller selector")
                    .selected_text(label)
                    .show_ui(ui, |ui| {
                        ui.selectable_value(controller, ControllerType::Betafligt, "Betaflight");
                        for res_id in context.reservoir_controller_ids.iter() {
                            ui.selectable_value(
                                controller,
                                ControllerType::Reservoir(res_id.into()),
                                format!("Resrevoid controller {}", res_id),
                            );
                        }
                        ui.selectable_value(controller, ControllerType::NullController, "Null");
                    });
            }
            UIState::Replay { .. } => {
                ui.label("Only available in simulation mode");
            }
        }
    });

    ui.horizontal(|ui| {
        ui.label("Logger:");
        if let UIState::Simulation { logger, .. } = ui_state {
            let label = format!("{logger:?}");
            egui::ComboBox::from_id_salt("Logger selector")
                .selected_text(label)
                .show_ui(ui, |ui| {
                    ui.selectable_value(logger, LoggerType::Db, "DB logger");
                    ui.selectable_value(logger, LoggerType::File, "File");
                    ui.selectable_value(logger, LoggerType::Rerun, "Rerun");
                    ui.selectable_value(logger, LoggerType::Empty, "None");
                });
        } else {
            ui.label("Only available in simulation mode");
        }
    });

    ui.horizontal(|ui| {
        ui.label("Replay id:");
        if let UIState::Replay { replay_id, .. } = ui_state {
            let label = match replay_id {
                Some(x) => format!("Replay: {}", x),
                None => "Not selected".to_string(),
            };
            egui::ComboBox::from_id_salt("Replay selector")
                .selected_text(label)
                .show_ui(ui, |ui| {
                    for id in context.replay_ids.iter() {
                        ui.selectable_value(replay_id, Some(id.into()), format!("Replay {}", id));
                    }
                });
        } else {
            ui.label("Only available in replay mode");
        }
    });

    ui.horizontal(|ui| {
        ui.label("Loader:");
        let loader = match ui_state {
            UIState::Replay { loader, .. } => loader,
            UIState::Simulation { loader, .. } => loader,
        };
        let label = match loader {
            LoaderType::DB => format!("Loader"),
            LoaderType::File => format!("File"),
            LoaderType::DefaultLoader => format!("Default"),
        };
        egui::ComboBox::from_id_salt("Loader selector")
            .selected_text(label)
            .show_ui(ui, |ui| {
                if ui.selectable_value(loader, LoaderType::DB, "DB").clicked() {
                    context.set_loader(&LoaderType::DB);
                    context.refresh_cache();
                };
                if ui
                    .selectable_value(loader, LoaderType::File, "File")
                    .clicked()
                {
                    context.set_loader(&LoaderType::File);
                    context.refresh_cache();
                };
                if ui
                    .selectable_value(loader, LoaderType::DefaultLoader, "Default")
                    .clicked()
                {
                    context.set_loader(&LoaderType::DefaultLoader);
                    context.refresh_cache();
                };
            });
    });

    if ui.button("Apply").clicked() {
        match &ui_state {
            UIState::Simulation {
                logger,
                controller,
                loader,
                simulation_name,
            } => {
                if simulation_name != "" {
                    context.set_simulation_id(simulation_name.clone());
                }
                context.set_loader(loader);
                // needs to be created
                context.set_logger(logger.clone());
                // needs to be created
                context.set_controller(controller.clone());
                next_visualizer_state.set(VisualizerState::Simulation);
            }
            UIState::Replay {
                replay_id: Some(replay_id),
                loader,
                ..
            } => {
                context.set_loader(loader);
                context.set_replay_id(replay_id.to_owned());
                // context.set_controller(controller.clone());
                next_visualizer_state.set(VisualizerState::Replay);
            }
            _ => {
                // We should warn of incomplete config
            }
        }
    }
}

pub fn menu_ui(
    mut egui_ctx: EguiContexts,
    ui_state: &mut UIState,
    context: &mut SimContext,
    next_visualizer_state: ResMut<NextState<VisualizerState>>,
) {
    let frame = Frame {
        fill: Color32::TRANSPARENT,
        ..Default::default()
    };
    egui::CentralPanel::default()
        .frame(frame)
        .show(egui_ctx.ctx_mut(), |ui| {
            let available_height = ui.available_height();
            let available_width = ui.available_width();

            let switch_box_min = pos2(available_width / 4., available_height / 4.);
            let switch_box_max = pos2(available_width * 3. / 4., available_height * 3. / 4.);
            let switch_box = Rect::from_min_max(switch_box_min, switch_box_max);
            ui.painter().add(Shape::Rect(RectShape::new(
                switch_box,
                CornerRadius::default(),
                Color32::from_rgb(59, 59, 59),
                Stroke::new(1., Color32::from_rgb(127, 0, 255)),
                egui::StrokeKind::Inside,
            )));

            let info_box_min = pos2(available_width / 4. + 10., available_height / 4. + 10.);
            let info_box_max = pos2(
                available_width * 3. / 4. - 10.,
                available_height * 3. / 4. - 10.,
            );
            let info_box = Rect::from_min_max(info_box_min, info_box_max);
            ui.allocate_new_ui(UiBuilder::new().max_rect(info_box), |ui| {
                main_menu_toggle(ui, ui_state, context, next_visualizer_state);
            });
        });
}
