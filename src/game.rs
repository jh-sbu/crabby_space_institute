#![allow(clippy::too_many_arguments, clippy::type_complexity)]

use bevy::camera::Hdr;
use bevy::math::DVec3;
use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use std::f32::consts::TAU as TAU32;

use crate::model::{
    CraftBlueprint, FlightSituation, ManeuverNode, PartCatalog, PartCategory, PartInstance,
    PartModule, SasMode, Stage, StageAction, ValidationIssue, Vessel, stock_craft,
};
use crate::orbit::{
    body_definition, celestial_system, circular_ephemeris, sample_trajectory, vessel_root_state,
};
use crate::save::{QuickSave, SAVE_SCHEMA, SaveStore};
use crate::scripting::{COROUTINE_EXAMPLE, EXAMPLE_SCRIPT, ScriptRuntime};
use crate::simulation::{
    FlightTelemetry, MissionProgress, SimulationClock, activate_next_stage, craft_stats,
    on_rails_warp_is_safe, step_on_rails_patched, step_vessel, telemetry, update_mission,
};

#[cfg(feature = "mcp")]
mod mcp;

#[derive(
    States, Debug, Clone, Copy, Default, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(rename_all = "snake_case")]
enum AppMode {
    #[default]
    Menu,
    Editor,
    Flight,
}

#[derive(Resource)]
struct Catalog(PartCatalog);

#[derive(Resource)]
struct Store(SaveStore);

#[derive(Resource)]
struct Session {
    craft: CraftBlueprint,
    vessel: Option<Vessel>,
    telemetry: FlightTelemetry,
    mission: MissionProgress,
    notice: String,
    visual_dirty: bool,
}

impl Default for Session {
    fn default() -> Self {
        Self {
            craft: stock_craft(),
            vessel: None,
            telemetry: FlightTelemetry::default(),
            mission: MissionProgress::default(),
            notice: "Welcome to the Institute, pilot.".into(),
            visual_dirty: true,
        }
    }
}

#[derive(Resource)]
struct EditorState {
    selected: Option<u64>,
    symmetry: usize,
    history: Vec<CraftBlueprint>,
    future: Vec<CraftBlueprint>,
    script_name: String,
    loaded_crafts: Vec<String>,
    show_script: bool,
}

impl Default for EditorState {
    fn default() -> Self {
        Self {
            selected: Some(1),
            symmetry: 1,
            history: Vec::new(),
            future: Vec::new(),
            script_name: "guided_ascent".into(),
            loaded_crafts: Vec::new(),
            show_script: false,
        }
    }
}

#[derive(Resource, Default)]
struct ViewState {
    map: bool,
    system_map: bool,
    camera_mode: usize,
    show_help: bool,
    show_script_console: bool,
    script_log: Vec<String>,
}

#[derive(Component)]
struct MainCamera;
#[derive(Component)]
struct PlanetVisual;
#[derive(Component)]
struct PadVisual;
#[derive(Component)]
struct MapMarker;
#[derive(Component)]
struct PartVisual {
    id: u64,
    debris: Option<usize>,
}

pub fn run() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "Crabby Space Institute".into(),
            resolution: (1440, 900).into(),
            resizable: true,
            ..default()
        }),
        ..default()
    }))
    .add_plugins(EguiPlugin::default())
    .insert_resource(ClearColor(Color::srgb(0.004, 0.008, 0.02)))
    .insert_resource(Time::<Fixed>::from_hz(60.0))
    .insert_resource(Catalog(PartCatalog::default()))
    .insert_resource(Store(SaveStore::default()))
    .init_resource::<Session>()
    .init_resource::<EditorState>()
    .init_resource::<ViewState>()
    .init_resource::<SimulationClock>()
    .init_resource::<ScriptRuntime>()
    .init_state::<AppMode>()
    .add_systems(Startup, setup_world)
    .add_systems(
        Update,
        (flight_input, rebuild_visuals, update_visuals, draw_orbits).chain(),
    )
    .add_systems(
        FixedUpdate,
        simulate_flight.run_if(in_state(AppMode::Flight)),
    )
    .add_systems(EguiPrimaryContextPass, game_ui);
    #[cfg(feature = "mcp")]
    app.add_plugins(mcp::GameMcpPlugin);
    app.run();
}

fn setup_world(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            near: 0.05,
            far: 3_000_000.0,
            ..default()
        }),
        Transform::from_xyz(18.0, 12.0, 24.0).looking_at(Vec3::new(0.0, 4.0, 0.0), Vec3::Y),
        Hdr,
        MainCamera,
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 80_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.8, -0.8, 0.0)),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(1.0).mesh().ico(6).unwrap())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.04, 0.18, 0.32),
            perceptual_roughness: 0.88,
            metallic: 0.02,
            ..default()
        })),
        Transform::default(),
        Visibility::Hidden,
        PlanetVisual,
    ));
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(20.0, 0.4, 20.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.16, 0.18, 0.2),
            metallic: 0.55,
            perceptual_roughness: 0.5,
            ..default()
        })),
        Transform::from_xyz(0.0, -3.0, 0.0),
        Visibility::Hidden,
        PadVisual,
    ));
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.12).mesh().ico(3).unwrap())),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.68, 0.16))),
        Visibility::Hidden,
        MapMarker,
    ));
}

fn part_color(category: PartCategory) -> Color {
    match category {
        PartCategory::Command => Color::srgb(0.76, 0.78, 0.78),
        PartCategory::Propulsion => Color::srgb(0.24, 0.27, 0.30),
        PartCategory::Fuel => Color::srgb(0.72, 0.74, 0.71),
        PartCategory::Coupling => Color::srgb(0.86, 0.48, 0.14),
        PartCategory::Control => Color::srgb(0.15, 0.48, 0.56),
        PartCategory::Aero => Color::srgb(0.5, 0.52, 0.54),
        PartCategory::Utility => Color::srgb(0.66, 0.24, 0.18),
    }
}

fn rebuild_visuals(
    mut commands: Commands,
    mut session: ResMut<Session>,
    state: Res<State<AppMode>>,
    catalog: Res<Catalog>,
    existing: Query<Entity, With<PartVisual>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !session.visual_dirty {
        return;
    }
    for entity in &existing {
        commands.entity(entity).despawn();
    }
    let source: Vec<_> = match state.get() {
        AppMode::Editor => session
            .craft
            .parts
            .iter()
            .map(|part| (part.clone(), false, false, None))
            .collect(),
        AppMode::Flight => session
            .vessel
            .as_ref()
            .map(|vessel| {
                let mut parts: Vec<_> = vessel
                    .parts
                    .iter()
                    .filter(|part| !part.destroyed)
                    .map(|part| {
                        (
                            part.instance.clone(),
                            part.parachute_deployed,
                            part.active,
                            None,
                        )
                    })
                    .collect();
                for (debris_index, stage) in vessel.debris.iter().enumerate() {
                    parts.extend(stage.parts.iter().map(|part| {
                        (
                            part.instance.clone(),
                            part.parachute_deployed,
                            part.active,
                            Some(debris_index),
                        )
                    }));
                }
                parts
            })
            .unwrap_or_default(),
        AppMode::Menu => Vec::new(),
    };
    for (part, chute_open, active, debris) in source {
        let Some(def) = catalog.0.get(&part.definition_id) else {
            continue;
        };
        let material = materials.add(StandardMaterial {
            base_color: if active
                && matches!(
                    def.module,
                    PartModule::LiquidEngine { .. } | PartModule::SolidEngine { .. }
                ) {
                Color::srgb(0.95, 0.38, 0.08)
            } else {
                part_color(def.category)
            },
            metallic: if matches!(def.category, PartCategory::Fuel | PartCategory::Propulsion) {
                0.55
            } else {
                0.15
            },
            perceptual_roughness: 0.45,
            ..default()
        });
        let position = Vec3::from_array(part.local_position);
        let rotation = Quat::from_array(part.local_rotation);
        if chute_open && matches!(def.module, PartModule::Parachute { .. }) {
            commands.spawn((
                Mesh3d(meshes.add(Sphere::new(1.0).mesh().uv(32, 16))),
                MeshMaterial3d(material),
                Transform::from_translation(position + Vec3::Y * 3.5)
                    .with_scale(Vec3::new(3.2, 0.55, 3.2)),
                PartVisual {
                    id: part.instance_id,
                    debris,
                },
            ));
        } else {
            let mesh = if def.radial
                || matches!(
                    def.module,
                    PartModule::Fin { .. } | PartModule::LandingLeg | PartModule::Rcs { .. }
                ) {
                meshes.add(Cuboid::new(def.radius * 1.2, def.height, def.radius * 0.65))
            } else {
                meshes.add(Cylinder::new(def.radius, def.height))
            };
            commands.spawn((
                Mesh3d(mesh),
                MeshMaterial3d(material),
                Transform::from_translation(position).with_rotation(rotation),
                PartVisual {
                    id: part.instance_id,
                    debris,
                },
            ));
        }
    }
    session.visual_dirty = false;
}

fn update_visuals(
    state: Res<State<AppMode>>,
    view: Res<ViewState>,
    session: Res<Session>,
    clock: Res<SimulationClock>,
    mut camera: Query<
        &mut Transform,
        (
            With<MainCamera>,
            Without<PartVisual>,
            Without<PlanetVisual>,
            Without<PadVisual>,
            Without<MapMarker>,
        ),
    >,
    mut planet: Query<
        (&mut Transform, &mut Visibility),
        (
            With<PlanetVisual>,
            Without<MainCamera>,
            Without<PartVisual>,
            Without<PadVisual>,
            Without<MapMarker>,
        ),
    >,
    mut pad: Query<
        (&mut Transform, &mut Visibility),
        (
            With<PadVisual>,
            Without<MainCamera>,
            Without<PartVisual>,
            Without<PlanetVisual>,
            Without<MapMarker>,
        ),
    >,
    mut marker: Query<
        (&mut Transform, &mut Visibility),
        (
            With<MapMarker>,
            Without<MainCamera>,
            Without<PartVisual>,
            Without<PlanetVisual>,
            Without<PadVisual>,
        ),
    >,
    mut parts: Query<
        (&PartVisual, &mut Transform, &mut Visibility),
        (
            Without<MainCamera>,
            Without<PlanetVisual>,
            Without<PadVisual>,
            Without<MapMarker>,
        ),
    >,
) {
    let Ok(mut camera) = camera.single_mut() else {
        return;
    };
    let Ok((mut planet_transform, mut planet_visibility)) = planet.single_mut() else {
        return;
    };
    let Ok((mut pad_transform, mut pad_visibility)) = pad.single_mut() else {
        return;
    };
    let Ok((mut marker_transform, mut marker_visibility)) = marker.single_mut() else {
        return;
    };
    match state.get() {
        AppMode::Menu => {
            *planet_visibility = Visibility::Hidden;
            *pad_visibility = Visibility::Hidden;
            *marker_visibility = Visibility::Hidden;
            for (_, _, mut visibility) in &mut parts {
                *visibility = Visibility::Hidden;
            }
            *camera =
                Transform::from_xyz(18.0, 12.0, 24.0).looking_at(Vec3::new(0.0, 4.0, 0.0), Vec3::Y);
        }
        AppMode::Editor => {
            *planet_visibility = Visibility::Hidden;
            *pad_visibility = Visibility::Visible;
            *marker_visibility = Visibility::Hidden;
            *pad_transform = Transform::from_xyz(0.0, -3.2, 0.0);
            for (_, _, mut visibility) in &mut parts {
                *visibility = Visibility::Visible;
            }
            *camera =
                Transform::from_xyz(19.0, 11.0, 25.0).looking_at(Vec3::new(0.0, 4.0, 0.0), Vec3::Y);
        }
        AppMode::Flight => {
            let Some(vessel) = &session.vessel else {
                return;
            };
            let body = body_definition(&vessel.primary_body);
            if view.map {
                let scale = if view.system_map {
                    6.0 / 22.0e9
                } else {
                    2.5 / body.radius
                };
                *planet_visibility = Visibility::Visible;
                *pad_visibility = Visibility::Hidden;
                *marker_visibility = Visibility::Visible;
                let planet_scale = if view.system_map {
                    (body.radius * scale).max(0.025) as f32
                } else {
                    2.5
                };
                *planet_transform = Transform::from_scale(Vec3::splat(planet_scale));
                marker_transform.translation = if view.system_map {
                    let (root_position, _) = vessel_root_state(
                        &vessel.primary_body,
                        vessel.position_vec(),
                        vessel.velocity_vec(),
                        clock.universal_time,
                    );
                    (root_position * scale).as_vec3()
                } else {
                    (vessel.position_vec() * scale).as_vec3()
                };
                marker_transform.scale = Vec3::splat(1.0);
                for (_, _, mut visibility) in &mut parts {
                    *visibility = Visibility::Hidden;
                }
                *camera = Transform::from_xyz(0.0, 0.0, 16.0).looking_at(Vec3::ZERO, Vec3::Y);
            } else {
                *planet_visibility = Visibility::Visible;
                *pad_visibility = Visibility::Visible;
                *marker_visibility = Visibility::Hidden;
                let position = vessel.position_vec();
                *planet_transform = Transform::from_translation((-position).as_vec3())
                    .with_scale(Vec3::splat(body.radius as f32));
                let launch_site = DVec3::new(0.0, body.radius, 0.0);
                *pad_transform = Transform::from_translation((launch_site - position).as_vec3());
                *pad_visibility = if vessel.primary_body == "carapace" {
                    Visibility::Visible
                } else {
                    Visibility::Hidden
                };
                let attitude = vessel.attitude_quat().as_quat();
                let (vessel_root_position, _) = vessel_root_state(
                    &vessel.primary_body,
                    vessel.position_vec(),
                    vessel.velocity_vec(),
                    clock.universal_time,
                );
                let offset = match view.camera_mode % 3 {
                    0 => Vec3::new(18.0, 10.0, 22.0),
                    1 => Vec3::new(0.0, 4.0, 30.0),
                    _ => Vec3::new(10.0, 4.0, 12.0),
                };
                *camera = Transform::from_translation(offset).looking_at(Vec3::ZERO, Vec3::Y);
                for (visual, mut transform, mut visibility) in &mut parts {
                    *visibility = Visibility::Visible;
                    if let Some(debris_index) = visual.debris
                        && let Some(stage) = vessel.debris.get(debris_index)
                        && let Some(runtime) = stage
                            .parts
                            .iter()
                            .find(|part| part.instance.instance_id == visual.id)
                    {
                        let (debris_root_position, _) = vessel_root_state(
                            &stage.primary_body,
                            stage.position_vec(),
                            stage.velocity_vec(),
                            clock.universal_time,
                        );
                        let local = Vec3::from_array(runtime.instance.local_position);
                        let debris_attitude = stage.attitude_quat().as_quat();
                        transform.translation = (debris_root_position - vessel_root_position)
                            .as_vec3()
                            + debris_attitude * local;
                        transform.rotation =
                            debris_attitude * Quat::from_array(runtime.instance.local_rotation);
                    } else if visual.debris.is_none()
                        && let Some(runtime) = vessel
                            .parts
                            .iter()
                            .find(|part| part.instance.instance_id == visual.id)
                    {
                        let local = Vec3::from_array(runtime.instance.local_position);
                        transform.translation = attitude * local;
                        transform.rotation =
                            attitude * Quat::from_array(runtime.instance.local_rotation);
                    }
                }
            }
        }
    }
}

fn flight_input(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    wants_input: Res<EguiWantsInput>,
    state: Res<State<AppMode>>,
    mut session: ResMut<Session>,
    catalog: Res<Catalog>,
    store: Res<Store>,
    mut clock: ResMut<SimulationClock>,
    mut runtime: ResMut<ScriptRuntime>,
    mut view: ResMut<ViewState>,
) {
    if *state.get() != AppMode::Flight || wants_input.wants_keyboard_input() {
        return;
    }
    let Session {
        vessel,
        mission,
        notice,
        visual_dirty,
        ..
    } = &mut *session;
    let Some(vessel) = vessel.as_mut() else {
        return;
    };
    let axis = |negative: KeyCode, positive: KeyCode| {
        keys.pressed(positive) as i8 as f64 - keys.pressed(negative) as i8 as f64
    };
    vessel.controls.pitch = axis(KeyCode::KeyS, KeyCode::KeyW);
    vessel.controls.yaw = axis(KeyCode::KeyD, KeyCode::KeyA);
    vessel.controls.roll = axis(KeyCode::KeyE, KeyCode::KeyQ);
    let throttle_delta =
        axis(KeyCode::ControlLeft, KeyCode::ShiftLeft) * time.delta_secs_f64() * 0.45;
    if throttle_delta != 0.0 {
        vessel.controls.throttle = (vessel.controls.throttle + throttle_delta).clamp(0.0, 1.0);
    }
    if keys.just_pressed(KeyCode::KeyZ) {
        vessel.controls.throttle = 1.0;
    }
    if keys.just_pressed(KeyCode::KeyX) {
        vessel.controls.throttle = 0.0;
    }
    if keys.just_pressed(KeyCode::Space) && activate_next_stage(vessel, &catalog.0) {
        *visual_dirty = true;
        runtime.emit_event("stage");
    }
    if keys.just_pressed(KeyCode::KeyT) {
        vessel.controls.sas = if vessel.controls.sas.is_some() {
            None
        } else {
            Some(SasMode::Stability)
        };
    }
    if keys.just_pressed(KeyCode::KeyR) {
        vessel.controls.rcs = !vessel.controls.rcs;
    }
    if keys.just_pressed(KeyCode::KeyM) {
        view.map = !view.map;
    }
    if keys.just_pressed(KeyCode::KeyC) {
        view.camera_mode = (view.camera_mode + 1) % 3;
    }
    if keys.just_pressed(KeyCode::Comma) {
        clock.warp_index = clock.warp_index.saturating_sub(1);
    }
    if keys.just_pressed(KeyCode::Period) {
        clock.warp_index = (clock.warp_index + 1).min(SimulationClock::WARP_RATES.len() - 1);
    }
    if keys.just_pressed(KeyCode::F8) {
        runtime.stop();
    }
    if keys.just_pressed(KeyCode::F5) {
        let save = QuickSave {
            schema_version: SAVE_SCHEMA,
            vessel: vessel.clone(),
            clock: clock.clone(),
            mission: mission.clone(),
            script_source: runtime.source.clone(),
            script_state: runtime.snapshot_state(),
        };
        *notice = match store.0.save_quick(&save) {
            Ok(()) => "Quicksave written".into(),
            Err(error) => format!("Save failed: {error}"),
        };
    }
    if keys.just_pressed(KeyCode::F9) {
        match store.0.load_quick() {
            Ok(save) => {
                *vessel = save.vessel;
                *clock = save.clock;
                *mission = save.mission;
                let _ = runtime.load(save.script_source, save.script_state);
                *visual_dirty = true;
                *notice = "Quicksave restored".into();
            }
            Err(error) => *notice = format!("Load failed: {error}"),
        }
    }
}

fn simulate_flight(
    mut session: ResMut<Session>,
    catalog: Res<Catalog>,
    mut clock: ResMut<SimulationClock>,
    mut runtime: ResMut<ScriptRuntime>,
) {
    if clock.paused {
        return;
    }
    let Session {
        vessel,
        telemetry: current_telemetry,
        mission,
        notice,
        visual_dirty,
        ..
    } = &mut *session;
    let Some(vessel) = vessel.as_mut() else {
        return;
    };
    let before = telemetry(
        vessel,
        &catalog.0,
        clock.universal_time,
        current_telemetry.thrust,
    );
    let commands = runtime.tick(&before);
    if let Some(value) = commands.throttle {
        vessel.controls.throttle = value;
    }
    if let Some(value) = commands.pitch {
        vessel.controls.pitch = value;
    }
    if let Some(value) = commands.yaw {
        vessel.controls.yaw = value;
    }
    if let Some(value) = commands.roll {
        vessel.controls.roll = value;
    }
    if let Some(value) = commands.sas {
        vessel.controls.sas = value;
    }
    if let Some(value) = commands.rcs {
        vessel.controls.rcs = value;
    }
    if commands.stage && activate_next_stage(vessel, &catalog.0) {
        *visual_dirty = true;
    }
    if commands.deploy_parachutes {
        for part in &mut vessel.parts {
            if catalog
                .0
                .get(&part.instance.definition_id)
                .is_some_and(|def| matches!(def.module, PartModule::Parachute { .. }))
            {
                part.parachute_deployed = true;
            }
        }
        *visual_dirty = true;
    }
    if let Some((ut, prograde, normal, radial)) = commands.maneuver {
        vessel.maneuver = Some(ManeuverNode {
            ut,
            prograde,
            normal,
            radial,
        });
    }
    if let Some(rate) = commands.warp_rate {
        clock.warp_index = SimulationClock::WARP_RATES
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| (*a - rate).abs().total_cmp(&(*b - rate).abs()))
            .map(|(i, _)| i)
            .unwrap_or(0);
    }

    if vessel.controls.throttle > 0.001 || !on_rails_warp_is_safe(vessel) {
        clock.warp_index = clock.warp_index.min(2);
    }
    if let Some(node) = &vessel.maneuver
        && node.ut - clock.universal_time < 30.0
    {
        clock.warp_index = 0;
    }
    let rate = clock.rate();
    if rate <= 4.0 {
        let steps = rate as usize;
        for _ in 0..steps {
            *current_telemetry = step_vessel(vessel, &catalog.0, 1.0 / 60.0, clock.universal_time);
            clock.universal_time += 1.0 / 60.0;
        }
    } else {
        let dt = rate / 60.0;
        step_on_rails_patched(vessel, clock.universal_time, dt);
        clock.universal_time += dt;
        *current_telemetry = telemetry(vessel, &catalog.0, clock.universal_time, 0.0);
        if !on_rails_warp_is_safe(vessel) {
            clock.warp_index = clock.warp_index.min(2);
        }
    }
    update_mission(mission, vessel, current_telemetry);
    if vessel.situation == FlightSituation::Crashed {
        *notice =
            "Mission failed: command vehicle lost. The crab crew has been returned to the roster."
                .into();
    }
    if mission.recovered {
        *notice = "Mission complete — recovered safely on Carapace!".into();
    }
}

fn draw_orbits(
    mut gizmos: Gizmos,
    state: Res<State<AppMode>>,
    view: Res<ViewState>,
    session: Res<Session>,
    clock: Res<SimulationClock>,
) {
    if *state.get() == AppMode::Editor {
        let stats = craft_stats(&session.craft, &PartCatalog::default());
        gizmos.line(
            Vec3::new(-2.0, stats.center_of_mass_y as f32, 0.0),
            Vec3::new(2.0, stats.center_of_mass_y as f32, 0.0),
            Color::srgb(1.0, 0.8, 0.1),
        );
        gizmos.line(
            Vec3::new(-2.0, stats.center_of_pressure_y as f32, 0.1),
            Vec3::new(2.0, stats.center_of_pressure_y as f32, 0.1),
            Color::srgb(0.1, 0.7, 1.0),
        );
        gizmos.line(
            Vec3::new(-2.0, stats.center_of_thrust_y as f32, 0.2),
            Vec3::new(2.0, stats.center_of_thrust_y as f32, 0.2),
            Color::srgb(1.0, 0.25, 0.15),
        );
        return;
    }
    if *state.get() != AppMode::Flight || !view.map {
        return;
    }
    let Some(vessel) = &session.vessel else {
        return;
    };
    if view.system_map {
        let bodies = celestial_system();
        let scale = 6.0 / 22.0e9;
        for body in bodies.iter().filter(|body| body.parent == Some("pelagos")) {
            gizmos.circle(
                Isometry3d::IDENTITY,
                (body.semi_major_axis * scale) as f32,
                Color::srgba(0.35, 0.45, 0.6, 0.55),
            );
            let (position, _) = circular_ephemeris(body, 1.327e18, clock.universal_time);
            gizmos.sphere(
                (position * scale).as_vec3(),
                if body.id == "carapace" { 0.09 } else { 0.07 },
                Color::srgb(0.3, 0.65, 0.9),
            );
        }
        gizmos.sphere(Vec3::ZERO, 0.18, Color::srgb(1.0, 0.75, 0.24));
    } else {
        let body = body_definition(&vessel.primary_body);
        let scale = 2.5 / body.radius;
        let points = sample_trajectory(
            vessel.position_vec(),
            vessel.velocity_vec(),
            body.mu,
            body.radius,
            220,
        )
        .into_iter()
        .map(|point| (point * scale).as_vec3());
        gizmos.linestrip(points, Color::srgb(0.2, 0.95, 0.78));
        if let Some(atmosphere) = &body.atmosphere {
            gizmos.circle(
                Isometry3d::IDENTITY,
                ((body.radius + atmosphere.height) * scale) as f32,
                Color::srgba(0.35, 0.65, 1.0, 0.35),
            );
        }
        if vessel.primary_body == "carapace" {
            let moon_orbit = 12.0e6 * scale;
            if moon_orbit < 14.0 {
                gizmos.circle(
                    Isometry3d::IDENTITY,
                    moon_orbit as f32,
                    Color::srgba(0.7, 0.7, 0.75, 0.45),
                );
            }
        }
        if let Some(node) = &vessel.maneuver {
            let dt = (node.ut - clock.universal_time).max(0.0);
            let (node_position, node_velocity) = crate::orbit::propagate_universal(
                vessel.position_vec(),
                vessel.velocity_vec(),
                body.mu,
                dt,
            );
            let prograde = node_velocity.normalize_or_zero();
            let normal = node_position.cross(node_velocity).normalize_or_zero();
            let radial = node_position.normalize_or_zero();
            let post_velocity = node_velocity
                + prograde * node.prograde
                + normal * node.normal
                + radial * node.radial;
            let post = sample_trajectory(node_position, post_velocity, body.mu, body.radius, 160)
                .into_iter()
                .map(|point| (point * scale).as_vec3());
            gizmos.linestrip(post, Color::srgb(1.0, 0.45, 0.16));
            gizmos.sphere(
                (node_position * scale).as_vec3(),
                0.08,
                Color::srgb(1.0, 0.8, 0.2),
            );
        }
    }
}

fn game_ui(
    mut contexts: EguiContexts,
    state: Res<State<AppMode>>,
    mut next_state: ResMut<NextState<AppMode>>,
    mut session: ResMut<Session>,
    catalog: Res<Catalog>,
    store: Res<Store>,
    mut editor: ResMut<EditorState>,
    mut view: ResMut<ViewState>,
    mut clock: ResMut<SimulationClock>,
    mut runtime: ResMut<ScriptRuntime>,
    mut app_exit: MessageWriter<AppExit>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    let mut visuals = egui::Visuals::dark();
    visuals.panel_fill = egui::Color32::from_rgb(11, 18, 27);
    visuals.window_fill = egui::Color32::from_rgb(14, 23, 34);
    visuals.selection.bg_fill = egui::Color32::from_rgb(184, 85, 39);
    ctx.set_visuals(visuals);

    for line in runtime.drain_logs() {
        view.script_log.push(line);
        if view.script_log.len() > 200 {
            view.script_log.remove(0);
        }
    }
    let mut viewport_ui = egui::Ui::new(
        ctx.clone(),
        "viewport".into(),
        egui::UiBuilder::new()
            .layer_id(egui::LayerId::background())
            .max_rect(ctx.viewport_rect()),
    );
    match state.get() {
        AppMode::Menu => menu_ui(
            &mut viewport_ui,
            &mut next_state,
            &mut session,
            &store.0,
            &mut clock,
            &mut runtime,
            &mut view,
            &mut app_exit,
        ),
        AppMode::Editor => editor_ui(
            ctx,
            &mut viewport_ui,
            &mut next_state,
            &mut session,
            &catalog.0,
            &store.0,
            &mut editor,
            &mut runtime,
        ),
        AppMode::Flight => flight_ui(
            ctx,
            &mut viewport_ui,
            &mut next_state,
            &mut session,
            &catalog.0,
            &store.0,
            &mut view,
            &mut clock,
            &mut runtime,
        ),
    }
    Ok(())
}

fn menu_ui(
    viewport_ui: &mut egui::Ui,
    next_state: &mut NextState<AppMode>,
    session: &mut Session,
    store: &SaveStore,
    clock: &mut SimulationClock,
    runtime: &mut ScriptRuntime,
    view: &mut ViewState,
    app_exit: &mut MessageWriter<AppExit>,
) {
    egui::CentralPanel::default().show(viewport_ui, |ui| {
        ui.vertical_centered(|ui| {
            ui.add_space(110.0);
            draw_crab(ui, 80.0);
            ui.heading(egui::RichText::new("CRABBY SPACE INSTITUTE").size(40.0).strong());
            ui.label(egui::RichText::new("Per aspera ad astacum").italics().color(egui::Color32::from_rgb(227, 158, 96)));
            ui.add_space(30.0);
            if ui.add_sized([260.0, 42.0], egui::Button::new("Vehicle Assembly")).clicked() {
                session.craft = stock_craft();
                session.vessel = None;
                session.visual_dirty = true;
                session.notice = "Stock Pathfinder loaded. Modify it or launch as-is.".into();
                runtime.source = EXAMPLE_SCRIPT.into();
                next_state.set(AppMode::Editor);
            }
            if store.quicksave_exists() && ui.add_sized([260.0, 42.0], egui::Button::new("Continue Quicksave")).clicked() {
                match store.load_quick() {
                    Ok(save) => {
                        session.vessel = Some(save.vessel);
                        session.mission = save.mission;
                        *clock = save.clock;
                        if let Err(error) = runtime.load(save.script_source, save.script_state) { session.notice = format!("Flight loaded; script paused: {error}"); }
                        session.visual_dirty = true;
                        view.map = false;
                        next_state.set(AppMode::Flight);
                    }
                    Err(error) => session.notice = format!("Could not load quicksave: {error}"),
                }
            }
            if ui.add_sized([260.0, 42.0], egui::Button::new("Quit")).clicked() { app_exit.write(AppExit::Success); }
            ui.add_space(24.0);
            ui.label("3D construction · staged rockets · aerothermal return · patched-conic map · Lua autopilot");
            ui.label(egui::RichText::new(&session.notice).color(egui::Color32::LIGHT_BLUE));
        });
    });
}

#[derive(Debug)]
enum EditorAction {
    Add(String),
    Remove,
    Undo,
    Redo,
    New,
    Save,
    Load(String),
    Launch,
}

fn editor_ui(
    ctx: &egui::Context,
    viewport_ui: &mut egui::Ui,
    next_state: &mut NextState<AppMode>,
    session: &mut Session,
    catalog: &PartCatalog,
    store: &SaveStore,
    editor: &mut EditorState,
    runtime: &mut ScriptRuntime,
) {
    let mut action = None;
    egui::Panel::top("editor_top").show(viewport_ui, |ui| {
        ui.horizontal(|ui| {
            ui.heading("Vehicle Assembly Bay");
            ui.separator();
            ui.label("Name:");
            ui.text_edit_singleline(&mut session.craft.name);
            if ui.button("New").clicked() {
                action = Some(EditorAction::New);
            }
            if ui.button("Undo").clicked() {
                action = Some(EditorAction::Undo);
            }
            if ui.button("Redo").clicked() {
                action = Some(EditorAction::Redo);
            }
            if ui.button("Save craft").clicked() {
                action = Some(EditorAction::Save);
            }
            if ui.button("Lua editor").clicked() {
                editor.show_script = !editor.show_script;
            }
        });
    });

    egui::Panel::left("part_palette").resizable(true).default_size(250.0).show(viewport_ui, |ui| {
        ui.heading("Part catalog");
        ui.horizontal(|ui| {
            ui.label("Radial symmetry");
            egui::ComboBox::from_id_salt("symmetry").selected_text(format!("{}×", editor.symmetry)).show_ui(ui, |ui| {
                for value in [1, 2, 4] { ui.selectable_value(&mut editor.symmetry, value, format!("{value}×")); }
            });
        });
        ui.small("Select a parent in the craft tree, then add a part. Stack and radial nodes snap automatically.");
        ui.separator();
        egui::ScrollArea::vertical().show(ui, |ui| {
            for category in [PartCategory::Command, PartCategory::Propulsion, PartCategory::Fuel, PartCategory::Coupling, PartCategory::Control, PartCategory::Aero, PartCategory::Utility] {
                egui::CollapsingHeader::new(format!("{category:?}")).default_open(matches!(category, PartCategory::Propulsion | PartCategory::Fuel)).show(ui, |ui| {
                    for def in catalog.iter().filter(|def| def.category == category) {
                        if ui.button(&def.name).on_hover_text(&def.description).clicked() { action = Some(EditorAction::Add(def.id.clone())); }
                    }
                });
            }
        });
    });

    egui::Panel::right("craft_tree")
        .resizable(true)
        .default_size(290.0)
        .show(viewport_ui, |ui| {
            ui.heading("Craft tree");
            egui::ScrollArea::vertical()
                .max_height(330.0)
                .show(ui, |ui| {
                    for part in &session.craft.parts {
                        let name = catalog
                            .get(&part.definition_id)
                            .map(|def| def.name.as_str())
                            .unwrap_or("Unknown");
                        let depth = part_depth(&session.craft, part.instance_id);
                        ui.horizontal(|ui| {
                            ui.add_space(depth as f32 * 10.0);
                            if ui
                                .selectable_label(
                                    editor.selected == Some(part.instance_id),
                                    format!("#{:02} {name}", part.instance_id),
                                )
                                .clicked()
                            {
                                editor.selected = Some(part.instance_id);
                            }
                        });
                    }
                });
            if ui.button("Remove selected subtree").clicked() {
                action = Some(EditorAction::Remove);
            }
            ui.separator();
            ui.heading("Stages");
            for (index, stage) in session.craft.stages.iter().enumerate() {
                egui::CollapsingHeader::new(format!("{} · {}", index + 1, stage.name))
                    .default_open(true)
                    .show(ui, |ui| {
                        for stage_action in &stage.actions {
                            ui.small(stage_action_label(stage_action, &session.craft, catalog));
                        }
                    });
            }
            ui.separator();
            if editor.loaded_crafts.is_empty() {
                editor.loaded_crafts = store.list_crafts();
            }
            egui::ComboBox::from_id_salt("load_craft")
                .selected_text("Load saved craft")
                .show_ui(ui, |ui| {
                    for name in editor.loaded_crafts.clone() {
                        if ui.button(&name).clicked() {
                            action = Some(EditorAction::Load(name));
                        }
                    }
                });
        });

    let stats = craft_stats(&session.craft, catalog);
    egui::Panel::bottom("editor_bottom").show(viewport_ui, |ui| {
        ui.horizontal_wrapped(|ui| {
            stat(ui, "Parts", session.craft.parts.len() as f64, "");
            stat(ui, "Wet mass", stats.wet_mass / 1_000.0, " t");
            stat(ui, "SL TWR", stats.sea_level_twr, "");
            stat(ui, "Vacuum Δv", stats.vacuum_delta_v, " m/s");
            stat(ui, "COM", stats.center_of_mass_y, " m");
            stat(ui, "COP", stats.center_of_pressure_y, " m");
            stat(ui, "COT", stats.center_of_thrust_y, " m");
            ui.separator();
            if ui
                .add_sized([150.0, 34.0], egui::Button::new("LAUNCH"))
                .clicked()
            {
                action = Some(EditorAction::Launch);
            }
        });
        ui.label(egui::RichText::new(&session.notice).color(egui::Color32::LIGHT_BLUE));
    });

    if editor.show_script {
        egui::Window::new("Vessel Lua").default_size([620.0, 560.0]).show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("File"); ui.text_edit_singleline(&mut editor.script_name);
                if ui.button("Save .lua").clicked() {
                    session.notice = match store.save_script(&editor.script_name, &runtime.source) { Ok(path) => format!("Saved {}", path.display()), Err(error) => format!("Script save failed: {error}") };
                }
                if ui.button("Load").clicked() {
                    match store.load_script(&editor.script_name) { Ok(source) => runtime.source = source, Err(error) => session.notice = format!("Script load failed: {error}") }
                }
                if ui.button("Callback example").clicked() { runtime.source = EXAMPLE_SCRIPT.into(); }
                if ui.button("Coroutine example").clicked() { runtime.source = COROUTINE_EXAMPLE.into(); }
            });
            ui.add(egui::TextEdit::multiline(&mut runtime.source).font(egui::TextStyle::Monospace).desired_rows(25).desired_width(f32::INFINITY));
            ui.small("API: flight.*, resources.*, control.set_throttle/rotation/sas/rcs/stage/deploy_parachutes, nav.set_warp/set_maneuver, wait.*, log.info");
        });
    }

    if let Some(action) = action {
        apply_editor_action(action, next_state, session, catalog, store, editor, runtime);
    }
}

fn apply_editor_action(
    action: EditorAction,
    next_state: &mut NextState<AppMode>,
    session: &mut Session,
    catalog: &PartCatalog,
    store: &SaveStore,
    editor: &mut EditorState,
    runtime: &mut ScriptRuntime,
) {
    match action {
        EditorAction::Add(definition) => {
            push_history(editor, &session.craft);
            add_part(
                &mut session.craft,
                catalog,
                editor.selected,
                &definition,
                editor.symmetry,
            );
            auto_stages(&mut session.craft, catalog);
            editor.selected = session.craft.parts.last().map(|part| part.instance_id);
            session.visual_dirty = true;
        }
        EditorAction::Remove => {
            let Some(selected) = editor.selected else {
                return;
            };
            if session.craft.root() == Some(selected) {
                session.notice =
                    "The root command pod cannot be removed; start a new craft instead.".into();
                return;
            }
            push_history(editor, &session.craft);
            let removed = blueprint_descendants(&session.craft, selected);
            session
                .craft
                .parts
                .retain(|part| !removed.contains(&part.instance_id));
            auto_stages(&mut session.craft, catalog);
            editor.selected = session.craft.root();
            session.visual_dirty = true;
        }
        EditorAction::Undo => {
            if let Some(previous) = editor.history.pop() {
                editor.future.push(session.craft.clone());
                session.craft = previous;
                editor.selected = session.craft.root();
                session.visual_dirty = true;
            }
        }
        EditorAction::Redo => {
            if let Some(next) = editor.future.pop() {
                editor.history.push(session.craft.clone());
                session.craft = next;
                editor.selected = session.craft.root();
                session.visual_dirty = true;
            }
        }
        EditorAction::New => {
            push_history(editor, &session.craft);
            session.craft = CraftBlueprint {
                schema_version: 1,
                name: "Untitled Vessel".into(),
                parts: vec![PartInstance {
                    instance_id: 1,
                    definition_id: "pod_1".into(),
                    parent: None,
                    local_position: [0.0, 0.0, 0.0],
                    local_rotation: [0.0, 0.0, 0.0, 1.0],
                }],
                stages: Vec::new(),
                crew: vec!["Dr. Clawdia Current".into()],
                script_name: None,
            };
            editor.selected = Some(1);
            session.visual_dirty = true;
        }
        EditorAction::Save => {
            session.craft.script_name = Some(format!("{}.lua", editor.script_name));
            session.notice = match store.save_craft(&session.craft) {
                Ok(path) => format!("Craft saved to {}", path.display()),
                Err(error) => format!("Craft save failed: {error}"),
            };
            editor.loaded_crafts = store.list_crafts();
        }
        EditorAction::Load(name) => match store.load_craft(&name) {
            Ok(craft) => {
                push_history(editor, &session.craft);
                session.craft = craft;
                editor.selected = session.craft.root();
                session.visual_dirty = true;
                session.notice = format!("Loaded {name}");
            }
            Err(error) => session.notice = format!("Load failed: {error}"),
        },
        EditorAction::Launch => {
            let issues = session.craft.validate(catalog);
            if let Some(ValidationIssue::Error(error)) = issues
                .iter()
                .find(|issue| matches!(issue, ValidationIssue::Error(_)))
            {
                session.notice = format!("Cannot launch: {error}");
                return;
            }
            let warnings: Vec<_> = issues
                .iter()
                .filter_map(|issue| {
                    if let ValidationIssue::Warning(text) = issue {
                        Some(text.as_str())
                    } else {
                        None
                    }
                })
                .collect();
            session.notice = if warnings.is_empty() {
                "Flight systems ready. Space ignites the first stage.".into()
            } else {
                format!("Launched with warnings: {}", warnings.join("; "))
            };
            session.vessel = Some(Vessel::from_blueprint(&session.craft, catalog));
            session.telemetry = session
                .vessel
                .as_ref()
                .map(|vessel| telemetry(vessel, catalog, 0.0, 0.0))
                .unwrap_or_default();
            session.mission = MissionProgress::default();
            session.visual_dirty = true;
            runtime.stop();
            next_state.set(AppMode::Flight);
        }
    }
}

fn flight_ui(
    ctx: &egui::Context,
    viewport_ui: &mut egui::Ui,
    next_state: &mut NextState<AppMode>,
    session: &mut Session,
    catalog: &PartCatalog,
    store: &SaveStore,
    view: &mut ViewState,
    clock: &mut SimulationClock,
    runtime: &mut ScriptRuntime,
) {
    egui::Panel::top("flight_top").show(viewport_ui, |ui| {
        ui.horizontal(|ui| {
            if ui.button("Assembly").clicked() {
                runtime.stop();
                session.visual_dirty = true;
                next_state.set(AppMode::Editor);
            }
            if ui.selectable_label(view.map, "Map [M]").clicked() {
                view.map = !view.map;
            }
            if view.map
                && ui
                    .selectable_label(view.system_map, "System view")
                    .clicked()
            {
                view.system_map = !view.system_map;
            }
            ui.separator();
            if ui.button("◀ warp").clicked() {
                clock.warp_index = clock.warp_index.saturating_sub(1);
            }
            ui.label(format!("{}×", clock.rate()));
            if ui.button("warp ▶").clicked() {
                clock.warp_index =
                    (clock.warp_index + 1).min(SimulationClock::WARP_RATES.len() - 1);
            }
            if ui.selectable_label(clock.paused, "Pause").clicked() {
                clock.paused = !clock.paused;
            }
            ui.separator();
            if ui.button("F5 Save").clicked()
                && let Some(vessel) = &session.vessel
            {
                let save = QuickSave {
                    schema_version: SAVE_SCHEMA,
                    vessel: vessel.clone(),
                    clock: clock.clone(),
                    mission: session.mission.clone(),
                    script_source: runtime.source.clone(),
                    script_state: runtime.snapshot_state(),
                };
                session.notice = match store.save_quick(&save) {
                    Ok(()) => "Quicksave written".into(),
                    Err(error) => format!("Save failed: {error}"),
                };
            }
            if ui.button("F9 Load").clicked() {
                load_quicksave(session, store, clock, runtime);
            }
            if ui.button("Lua").clicked() {
                view.show_script_console = !view.show_script_console;
            }
            if ui.button("Help").clicked() {
                view.show_help = !view.show_help;
            }
        });
    });

    if let Some(vessel) = session.vessel.as_mut() {
        egui::Panel::left("flight_data")
            .default_size(235.0)
            .show(viewport_ui, |ui| {
                draw_crab(ui, 48.0);
                ui.heading(
                    vessel
                        .crew
                        .first()
                        .map(String::as_str)
                        .unwrap_or("Uncrewed"),
                );
                ui.label(format!(
                    "{:?} near {}",
                    vessel.situation,
                    body_definition(&vessel.primary_body).name
                ));
                ui.separator();
                telemetry_ui(ui, &session.telemetry);
                ui.separator();
                ui.label(format!(
                    "SAS: {}",
                    vessel
                        .controls
                        .sas
                        .map(|mode| format!("{mode:?}"))
                        .unwrap_or_else(|| "Off".into())
                ));
                ui.label(format!(
                    "RCS: {}",
                    if vessel.controls.rcs { "ON" } else { "off" }
                ));
                ui.label(format!(
                    "Stage: {}/{}",
                    vessel.next_stage,
                    vessel.stages.len()
                ));
                ui.add(
                    egui::ProgressBar::new(vessel.controls.throttle as f32)
                        .text(format!("Throttle {:.0}%", vessel.controls.throttle * 100.0)),
                );
            });

        egui::Panel::right("flight_mission")
            .default_size(260.0)
            .show(viewport_ui, |ui| {
                ui.heading("Flight plan");
                if ui.button("ACTIVATE NEXT STAGE [Space]").clicked()
                    && activate_next_stage(vessel, catalog)
                {
                    session.visual_dirty = true;
                }
                if let Some(stage) = vessel.stages.get(vessel.next_stage) {
                    ui.small(format!("Next: {}", stage.name));
                }
                ui.separator();
                ui.heading("SAS target");
                ui.horizontal_wrapped(|ui| {
                    for (label, mode) in [
                        ("Hold", SasMode::Stability),
                        ("Pro", SasMode::Prograde),
                        ("Retro", SasMode::Retrograde),
                        ("N+", SasMode::Normal),
                        ("Rad+", SasMode::RadialOut),
                        ("Node", SasMode::Maneuver),
                    ] {
                        if ui
                            .selectable_label(vessel.controls.sas == Some(mode), label)
                            .clicked()
                        {
                            vessel.controls.sas = Some(mode);
                        }
                    }
                    if ui
                        .selectable_label(vessel.controls.sas.is_none(), "Off")
                        .clicked()
                    {
                        vessel.controls.sas = None;
                    }
                });
                ui.separator();
                ui.heading("Guided mission");
                objective(ui, session.mission.launched, "Clear the launch tower");
                objective(ui, session.mission.staged, "Separate a spent stage");
                objective(
                    ui,
                    session.mission.achieved_orbit,
                    "Raise periapsis above 75 km",
                );
                objective(
                    ui,
                    session.mission.began_reentry,
                    "Re-enter Carapace's atmosphere",
                );
                objective(ui, session.mission.recovered, "Recover the command pod");
                ui.separator();
                ui.label(egui::RichText::new(&session.notice).color(egui::Color32::LIGHT_BLUE));
            });
    }

    if view.map {
        maneuver_window(ctx, session, clock);
    }
    if view.show_script_console {
        script_window(ctx, session, store, runtime, view);
    }
    if view.show_help {
        help_window(ctx);
    }
}

fn maneuver_window(ctx: &egui::Context, session: &mut Session, clock: &SimulationClock) {
    let Some(vessel) = session.vessel.as_mut() else {
        return;
    };
    egui::Window::new("Maneuver planning")
        .default_pos([290.0, 90.0])
        .show(ctx, |ui| {
            if vessel.maneuver.is_none() && ui.button("Add node +60 s").clicked() {
                vessel.maneuver = Some(ManeuverNode {
                    ut: clock.universal_time + 60.0,
                    prograde: 0.0,
                    normal: 0.0,
                    radial: 0.0,
                });
            }
            if let Some(node) = &mut vessel.maneuver {
                ui.horizontal(|ui| {
                    ui.label("Time UT");
                    ui.add(
                        egui::DragValue::new(&mut node.ut)
                            .speed(1.0)
                            .range(clock.universal_time..=clock.universal_time + 1.0e7),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Prograde");
                    ui.add(
                        egui::DragValue::new(&mut node.prograde)
                            .speed(1.0)
                            .suffix(" m/s"),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Normal");
                    ui.add(
                        egui::DragValue::new(&mut node.normal)
                            .speed(1.0)
                            .suffix(" m/s"),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Radial");
                    ui.add(
                        egui::DragValue::new(&mut node.radial)
                            .speed(1.0)
                            .suffix(" m/s"),
                    );
                });
                ui.label(format!("T− {:.1} s", node.ut - clock.universal_time));
                if ui.button("Remove node").clicked() {
                    vessel.maneuver = None;
                }
            }
        });
}

fn script_window(
    ctx: &egui::Context,
    session: &mut Session,
    store: &SaveStore,
    runtime: &mut ScriptRuntime,
    view: &mut ViewState,
) {
    egui::Window::new("Lua flight computer")
        .default_size([650.0, 620.0])
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                if !runtime.active {
                    if ui.button("Run").clicked() {
                        match runtime.load(runtime.source.clone(), None) {
                            Ok(()) => {
                                session.notice = "Automation running; F8 stops immediately.".into()
                            }
                            Err(error) => session.notice = format!("Lua error: {error}"),
                        }
                    }
                } else if ui.button("Stop [F8]").clicked() {
                    runtime.stop();
                }
                if ui.button("Save script").clicked() {
                    session.notice = match store.save_script("flight_computer", &runtime.source) {
                        Ok(path) => format!("Saved {}", path.display()),
                        Err(error) => format!("Save failed: {error}"),
                    };
                }
                ui.label(if runtime.active {
                    "● RUNNING"
                } else {
                    "○ stopped"
                });
            });
            ui.add(
                egui::TextEdit::multiline(&mut runtime.source)
                    .font(egui::TextStyle::Monospace)
                    .desired_rows(22)
                    .desired_width(f32::INFINITY),
            );
            ui.separator();
            ui.label("Log");
            egui::ScrollArea::vertical()
                .max_height(130.0)
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    for line in &view.script_log {
                        ui.monospace(line);
                    }
                    if let Some(error) = &runtime.last_error {
                        ui.colored_label(egui::Color32::LIGHT_RED, error);
                    }
                });
        });
}

fn help_window(ctx: &egui::Context) {
    egui::Window::new("Pilot controls").show(ctx, |ui| {
        ui.monospace("W/S pitch     A/D yaw      Q/E roll\nShift/Ctrl throttle        Z/X full/cut\nSpace stage    T SAS       R RCS\nM map          C camera    ,/. time warp\nF5/F9 save/load            F8 stop Lua");
        ui.separator();
        ui.label("High warp is available only while coasting safely outside the atmosphere. Map trajectories are analytic two-body conics; atmospheric paths become approximate at the entry interface.");
    });
}

fn telemetry_ui(ui: &mut egui::Ui, data: &FlightTelemetry) {
    value_row(ui, "Altitude", distance(data.altitude));
    value_row(
        ui,
        "Surface speed",
        format!("{:.1} m/s", data.surface_speed),
    );
    value_row(
        ui,
        "Vertical speed",
        format!("{:+.1} m/s", data.vertical_speed),
    );
    value_row(ui, "Apoapsis", distance(data.orbit.apoapsis));
    value_row(ui, "Periapsis", distance(data.orbit.periapsis));
    value_row(ui, "Mach", format!("{:.2}", data.mach));
    value_row(
        ui,
        "Dynamic pressure",
        format!("{:.1} kPa", data.dynamic_pressure / 1_000.0),
    );
    value_row(ui, "Heat flux", format!("{:.1}", data.heating));
    value_row(ui, "Mass", format!("{:.2} t", data.mass / 1_000.0));
    value_row(ui, "Liquid fuel", format!("{:.0} kg", data.liquid_fuel));
    value_row(ui, "Solid fuel", format!("{:.0} kg", data.solid_fuel));
}

fn value_row(ui: &mut egui::Ui, label: &str, value: String) {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.monospace(value);
        });
    });
}

fn distance(value: f64) -> String {
    if !value.is_finite() {
        "escape".into()
    } else if value.abs() >= 1_000_000.0 {
        format!("{:.2} Mm", value / 1_000_000.0)
    } else {
        format!("{:.1} km", value / 1_000.0)
    }
}

fn objective(ui: &mut egui::Ui, done: bool, label: &str) {
    ui.label(format!("{} {label}", if done { "✓" } else { "○" }));
}

fn stat(ui: &mut egui::Ui, label: &str, value: f64, suffix: &str) {
    ui.group(|ui| {
        ui.small(label);
        ui.monospace(format!("{value:.1}{suffix}"));
    });
}

fn stage_action_label(
    action: &StageAction,
    craft: &CraftBlueprint,
    catalog: &PartCatalog,
) -> String {
    let (verb, id) = match action {
        StageAction::ActivateEngine(id) => ("ignite", id),
        StageAction::Decouple(id) => ("release", id),
        StageAction::DeployParachute(id) => ("deploy", id),
    };
    let name = craft
        .parts
        .iter()
        .find(|part| part.instance_id == *id)
        .and_then(|part| catalog.get(&part.definition_id))
        .map(|def| def.name.as_str())
        .unwrap_or("missing part");
    format!("{verb} #{id} {name}")
}

fn part_depth(craft: &CraftBlueprint, id: u64) -> usize {
    let mut depth = 0;
    let mut current = craft
        .parts
        .iter()
        .find(|part| part.instance_id == id)
        .and_then(|part| part.parent);
    while let Some(parent) = current {
        depth += 1;
        current = craft
            .parts
            .iter()
            .find(|part| part.instance_id == parent)
            .and_then(|part| part.parent);
        if depth > craft.parts.len() {
            break;
        }
    }
    depth
}

fn push_history(editor: &mut EditorState, craft: &CraftBlueprint) {
    editor.history.push(craft.clone());
    if editor.history.len() > 64 {
        editor.history.remove(0);
    }
    editor.future.clear();
}

fn add_part(
    craft: &mut CraftBlueprint,
    catalog: &PartCatalog,
    selected: Option<u64>,
    definition: &str,
    symmetry: usize,
) {
    let Some(def) = catalog.get(definition) else {
        return;
    };
    if craft.parts.len() >= crate::model::MAX_PARTS {
        return;
    }
    let parent_id = selected.or_else(|| craft.root());
    let parent = parent_id
        .and_then(|id| craft.parts.iter().find(|part| part.instance_id == id))
        .cloned();
    let parent_def = parent
        .as_ref()
        .and_then(|part| catalog.get(&part.definition_id));
    let copies = if def.radial { symmetry.max(1) } else { 1 };
    let first_id = craft.next_id();
    for copy in 0..copies {
        if craft.parts.len() >= crate::model::MAX_PARTS {
            break;
        }
        let angle = TAU32 * copy as f32 / copies as f32;
        let position = if def.radial {
            let base = parent
                .as_ref()
                .map(|part| Vec3::from_array(part.local_position))
                .unwrap_or(Vec3::ZERO);
            let distance =
                parent_def.map(|parent| parent.radius).unwrap_or(1.0) + def.radius + 0.18;
            base + Vec3::new(angle.cos() * distance, 0.0, angle.sin() * distance)
        } else {
            let base = parent
                .as_ref()
                .map(|part| Vec3::from_array(part.local_position))
                .unwrap_or(Vec3::ZERO);
            let parent_height = parent_def.map(|parent| parent.height).unwrap_or(0.0);
            base - Vec3::Y * (parent_height + def.height) * 0.5
        };
        craft.parts.push(PartInstance {
            instance_id: first_id + copy as u64,
            definition_id: definition.into(),
            parent: parent_id,
            local_position: position.to_array(),
            local_rotation: Quat::from_rotation_y(-angle).to_array(),
        });
    }
}

fn auto_stages(craft: &mut CraftBlueprint, catalog: &PartCatalog) {
    let mut ignition = Stage {
        name: "Ignition".into(),
        actions: Vec::new(),
    };
    let mut radial = Stage {
        name: "Shed radial hardware".into(),
        actions: Vec::new(),
    };
    let mut inline = Stage {
        name: "Next stack stage".into(),
        actions: Vec::new(),
    };
    let mut recovery = Stage {
        name: "Recovery".into(),
        actions: Vec::new(),
    };
    for part in &craft.parts {
        let Some(def) = catalog.get(&part.definition_id) else {
            continue;
        };
        match def.module {
            PartModule::LiquidEngine { .. } | PartModule::SolidEngine { .. } => ignition
                .actions
                .push(StageAction::ActivateEngine(part.instance_id)),
            PartModule::RadialDecoupler { .. } => {
                radial.actions.push(StageAction::Decouple(part.instance_id))
            }
            PartModule::InlineDecoupler { .. } => {
                inline.actions.push(StageAction::Decouple(part.instance_id))
            }
            PartModule::Parachute { .. } => recovery
                .actions
                .push(StageAction::DeployParachute(part.instance_id)),
            _ => {}
        }
    }
    craft.stages = [ignition, radial, inline, recovery]
        .into_iter()
        .filter(|stage| !stage.actions.is_empty())
        .collect();
}

fn blueprint_descendants(craft: &CraftBlueprint, root: u64) -> std::collections::BTreeSet<u64> {
    let mut result = std::collections::BTreeSet::from([root]);
    loop {
        let before = result.len();
        for part in &craft.parts {
            if part.parent.is_some_and(|parent| result.contains(&parent)) {
                result.insert(part.instance_id);
            }
        }
        if result.len() == before {
            return result;
        }
    }
}

fn load_quicksave(
    session: &mut Session,
    store: &SaveStore,
    clock: &mut SimulationClock,
    runtime: &mut ScriptRuntime,
) {
    match store.load_quick() {
        Ok(save) => {
            session.vessel = Some(save.vessel);
            session.mission = save.mission;
            *clock = save.clock;
            session.notice = match runtime.load(save.script_source, save.script_state) {
                Ok(()) => "Quicksave restored".into(),
                Err(error) => format!("Flight restored; script paused: {error}"),
            };
            session.visual_dirty = true;
        }
        Err(error) => session.notice = format!("Load failed: {error}"),
    }
}

fn draw_crab(ui: &mut egui::Ui, size: f32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(size * 1.6, size), egui::Sense::hover());
    let painter = ui.painter();
    let center = rect.center() + egui::vec2(0.0, size * 0.08);
    let shell = egui::Color32::from_rgb(183, 68, 42);
    painter.add(egui::Shape::ellipse_filled(
        center,
        egui::vec2(size * 0.42, size * 0.28),
        shell,
    ));
    for side in [-1.0, 1.0] {
        let claw = center + egui::vec2(side * size * 0.56, -size * 0.05);
        painter.circle_filled(claw, size * 0.16, shell);
        painter.line_segment(
            [center + egui::vec2(side * size * 0.3, 0.0), claw],
            egui::Stroke::new(size * 0.08, shell),
        );
        for leg in 0..3 {
            let y = size * (0.09 + leg as f32 * 0.08);
            painter.line_segment(
                [
                    center + egui::vec2(side * size * 0.28, y - size * 0.1),
                    center + egui::vec2(side * size * (0.48 + leg as f32 * 0.06), y),
                ],
                egui::Stroke::new(size * 0.055, shell),
            );
        }
        let eye = center + egui::vec2(side * size * 0.17, -size * 0.26);
        painter.line_segment(
            [eye, eye + egui::vec2(0.0, -size * 0.12)],
            egui::Stroke::new(size * 0.045, shell),
        );
        painter.circle_filled(
            eye + egui::vec2(0.0, -size * 0.13),
            size * 0.055,
            egui::Color32::WHITE,
        );
        painter.circle_filled(
            eye + egui::vec2(side * size * 0.012, -size * 0.13),
            size * 0.025,
            egui::Color32::BLACK,
        );
    }
}
