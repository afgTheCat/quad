mod rigid_body;
mod ui;

use bevy::{
    math::{DMat3, DVec3},
    prelude::*,
    render::{
        mesh::{Indices, PrimitiveTopology},
        render_asset::RenderAssetUsages,
    },
};
use bevy_panorbit_camera::PanOrbitCamera;
pub use rigid_body::quboid_update;
use rigid_body::{inv_rectangular_cuboid_inertia_matrix, CubeRigidBody, SimulationContext};
use ui::UiState;
pub use ui::{handle_keyboard_events, update_ui};

/// set up a simple 3D scene
pub fn setup(
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
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_translation(Vec3::new(0.0, 1.5, 5.0)),
            ..default()
        },
        PanOrbitCamera::default(),
    ));

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

    let sides = Vec3::new(1.0, 0.1, 2.);
    let angular_momentum = DVec3::new(0.2, 0., 0.002);

    let simulation_context = SimulationContext::default();
    let cuboid_mesh = meshes.add(Cuboid::from_size(sides));
    let rigid_body = CubeRigidBody {
        inv_inertia_tensor: inv_rectangular_cuboid_inertia_matrix(sides),
        density: 1.,
        sides,
        angular_momentum,
        linear_velocity: Vec3::ZERO,
        mesh_handle: cuboid_mesh.clone(),
        rotation: DMat3::IDENTITY,
    };

    let ui_state = UiState {
        sides,
        angular_momentum,
        inertia_matrix_diag: rigid_body.inertia_diag(),
    };
    let mesh_bundle = PbrBundle {
        mesh: cuboid_mesh,
        material: materials.add(Color::srgb_u8(124, 144, 255)),
        transform: Transform {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        ..default()
    };

    commands.spawn((rigid_body, mesh_bundle, simulation_context, ui_state));
}
