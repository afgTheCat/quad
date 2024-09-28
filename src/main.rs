mod control;
mod rigid_body;
mod ui;

use bevy::{
    prelude::*,
    render::{
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
};
use bevy_egui::EguiPlugin;
use control::handle_keyboard_events;
use rigid_body::{main_qube, setup_rigid_body_context};
use ui::ui;

/// set up a simple 3D scene
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Add a directional light to simulate the sun
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::rgb(1.0, 1.0, 0.9), // Slightly warm sunlight
            illuminance: 100000.0, // Intensity of the light, tweak this based on your scene
            shadows_enabled: true, // Enable shadows for the sunlight
            ..Default::default()
        },
        transform: Transform {
            // Tilt the light to simulate the sun's angle (e.g., 45-degree angle)
            rotation: Quat::from_euler(EulerRot::XYZ, -std::f32::consts::FRAC_PI_4, 0.0, 0.0),
            ..Default::default()
        },
        ..Default::default()
    });

    // camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-2.5, 4.5, 9.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // Create grid lines
    let grid_size = 20;
    let line_spacing = 1.0;
    let mut grid_vertices = vec![];
    let mut grid_indices = vec![];

    for i in -grid_size..=grid_size {
        let pos = i as f32 * line_spacing;
        // Horizontal lines (along X-axis)
        grid_vertices.push([pos, -1.0, -grid_size as f32 * line_spacing]);
        grid_vertices.push([pos, -1.0, grid_size as f32 * line_spacing]);
        // Vertical lines (along Z-axis)
        grid_vertices.push([-grid_size as f32 * line_spacing, -1.0, pos]);
        grid_vertices.push([grid_size as f32 * line_spacing, -1.0, pos]);
    }

    for i in 0..grid_vertices.len() as u32 {
        grid_indices.push(i);
    }

    let mut grid_mesh = Mesh::new(PrimitiveTopology::LineList, RenderAssetUsages::RENDER_WORLD);
    grid_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, grid_vertices);
    grid_mesh.insert_indices(Indices::U32(grid_indices));

    // Spawn the grid
    commands.spawn(PbrBundle {
        mesh: meshes.add(grid_mesh),
        material: materials.add(Color::rgb(1., 1., 1.)),
        ..Default::default()
    });
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_systems(Startup, setup)
        .add_systems(Startup, setup_rigid_body_context)
        .add_systems(Update, handle_keyboard_events)
        .add_systems(Update, main_qube)
        .add_systems(Update, ui)
        .run();
}

#[cfg(test)]
mod test {
    #[test]
    fn integrate_thing() {}
}
