use bevy::math::{DQuat, DVec3};
use bevy::prelude::Resource;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::f64::consts::{PI, TAU};

use crate::model::{
    CraftBlueprint, FlightSituation, HOME_ATMOSPHERE, PartCatalog, PartModule, SasMode,
    StageAction, Vessel,
};
use crate::orbit::{
    CelestialBodyDef, OrbitalElements, body_definition, celestial_system, circular_ephemeris,
    elements, propagate_universal, sphere_of_influence,
};

const G0: f64 = 9.80665;
const ON_RAILS_CLEARANCE: f64 = 5_000.0;

fn aerodynamic_lift_force(
    forward: DVec3,
    air_direction: DVec3,
    dynamic_pressure: f64,
    fin_lift: f64,
) -> DVec3 {
    let crossflow = forward.reject_from(air_direction);
    -crossflow * dynamic_pressure * fin_lift * forward.dot(air_direction).abs()
}

#[derive(Debug, Clone, Resource, Serialize, Deserialize)]
pub struct SimulationClock {
    pub universal_time: f64,
    pub warp_index: usize,
    pub paused: bool,
}

impl Default for SimulationClock {
    fn default() -> Self {
        Self {
            universal_time: 0.0,
            warp_index: 0,
            paused: false,
        }
    }
}

impl SimulationClock {
    pub const WARP_RATES: [f64; 8] = [1.0, 2.0, 4.0, 10.0, 100.0, 1_000.0, 10_000.0, 100_000.0];
    pub fn rate(&self) -> f64 {
        Self::WARP_RATES[self.warp_index.min(Self::WARP_RATES.len() - 1)]
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MissionProgress {
    pub launched: bool,
    pub staged: bool,
    pub achieved_orbit: bool,
    pub began_reentry: bool,
    pub recovered: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct FlightTelemetry {
    pub ut: f64,
    pub altitude: f64,
    pub radar_altitude: f64,
    pub speed: f64,
    pub surface_speed: f64,
    pub vertical_speed: f64,
    pub mach: f64,
    pub dynamic_pressure: f64,
    pub heating: f64,
    pub mass: f64,
    pub liquid_fuel: f64,
    pub solid_fuel: f64,
    pub monopropellant: f64,
    pub thrust: f64,
    pub twr: f64,
    pub orbit: OrbitalElements,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct CraftStats {
    pub dry_mass: f64,
    pub wet_mass: f64,
    pub liquid_fuel: f64,
    pub solid_fuel: f64,
    pub sea_level_thrust: f64,
    pub vacuum_thrust: f64,
    pub sea_level_twr: f64,
    pub vacuum_delta_v: f64,
    pub center_of_mass_y: f64,
    pub center_of_pressure_y: f64,
    pub center_of_thrust_y: f64,
}

pub fn craft_stats(craft: &CraftBlueprint, catalog: &PartCatalog) -> CraftStats {
    let mut stats = CraftStats::default();
    let mut weighted_mass_y = 0.0;
    let mut pressure_area = 0.0;
    let mut pressure_y = 0.0;
    let mut thrust_y = 0.0;
    let mut thrust_weight = 0.0;
    let mut effective_isp_weighted = 0.0;
    for part in &craft.parts {
        let Some(def) = catalog.get(&part.definition_id) else {
            continue;
        };
        stats.dry_mass += def.dry_mass;
        weighted_mass_y += def.dry_mass * part.local_position[1] as f64;
        let area = PI * def.radius as f64 * def.radius as f64 * def.drag_coefficient;
        pressure_area += area;
        pressure_y += area * part.local_position[1] as f64;
        match def.module {
            PartModule::LiquidTank { fuel } => stats.liquid_fuel += fuel,
            PartModule::SolidEngine { thrust, isp, fuel } => {
                stats.solid_fuel += fuel;
                stats.sea_level_thrust += thrust;
                stats.vacuum_thrust += thrust;
                effective_isp_weighted += thrust * isp;
                thrust_y += thrust * part.local_position[1] as f64;
                thrust_weight += thrust;
            }
            PartModule::LiquidEngine {
                thrust_vac,
                thrust_sl,
                isp_vac,
                ..
            } => {
                stats.sea_level_thrust += thrust_sl;
                stats.vacuum_thrust += thrust_vac;
                effective_isp_weighted += thrust_vac * isp_vac;
                thrust_y += thrust_vac * part.local_position[1] as f64;
                thrust_weight += thrust_vac;
            }
            _ => {}
        }
    }
    stats.wet_mass = stats.dry_mass + stats.liquid_fuel + stats.solid_fuel;
    stats.center_of_mass_y = if stats.dry_mass > 0.0 {
        weighted_mass_y / stats.dry_mass
    } else {
        0.0
    };
    stats.center_of_pressure_y = if pressure_area > 0.0 {
        pressure_y / pressure_area
    } else {
        0.0
    };
    stats.center_of_thrust_y = if thrust_weight > 0.0 {
        thrust_y / thrust_weight
    } else {
        0.0
    };
    stats.sea_level_twr = stats.sea_level_thrust / (stats.wet_mass * G0).max(1.0);
    let isp = if stats.vacuum_thrust > 0.0 {
        effective_isp_weighted / stats.vacuum_thrust
    } else {
        0.0
    };
    if stats.wet_mass > stats.dry_mass && isp > 0.0 {
        stats.vacuum_delta_v = isp * G0 * (stats.wet_mass / stats.dry_mass).ln();
    }
    stats
}

pub fn vessel_mass(vessel: &Vessel, catalog: &PartCatalog) -> f64 {
    vessel
        .parts
        .iter()
        .filter(|part| !part.destroyed)
        .map(|part| {
            catalog
                .get(&part.instance.definition_id)
                .map_or(0.0, |def| def.dry_mass + part.fuel + part.ablator)
        })
        .sum::<f64>()
        .max(1.0)
}

pub fn resource_totals(vessel: &Vessel, catalog: &PartCatalog) -> (f64, f64, f64) {
    let mut liquid = 0.0;
    let mut solid = 0.0;
    let mut mono = 0.0;
    for part in vessel.parts.iter().filter(|part| !part.destroyed) {
        if let Some(def) = catalog.get(&part.instance.definition_id) {
            match def.module {
                PartModule::LiquidTank { .. } => liquid += part.fuel,
                PartModule::SolidEngine { .. } => solid += part.fuel,
                PartModule::MonopropTank { .. } => mono += part.fuel,
                _ => {}
            }
        }
    }
    (liquid, solid, mono)
}

fn drain_resource(vessel: &mut Vessel, catalog: &PartCatalog, liquid: bool, amount: f64) -> f64 {
    let available: f64 = vessel
        .parts
        .iter()
        .filter(|part| !part.destroyed)
        .filter_map(|part| {
            let def = catalog.get(&part.instance.definition_id)?;
            let matches = if liquid {
                matches!(def.module, PartModule::LiquidTank { .. })
            } else {
                matches!(def.module, PartModule::MonopropTank { .. })
            };
            matches.then_some(part.fuel)
        })
        .sum();
    let taken = available.min(amount);
    if available <= 0.0 {
        return 0.0;
    }
    for part in vessel.parts.iter_mut().filter(|part| !part.destroyed) {
        let Some(def) = catalog.get(&part.instance.definition_id) else {
            continue;
        };
        let matches = if liquid {
            matches!(def.module, PartModule::LiquidTank { .. })
        } else {
            matches!(def.module, PartModule::MonopropTank { .. })
        };
        if matches {
            part.fuel = (part.fuel - taken * part.fuel / available).max(0.0);
        }
    }
    taken
}

fn descendants(vessel: &Vessel, root: u64) -> BTreeSet<u64> {
    let mut result = BTreeSet::from([root]);
    loop {
        let before = result.len();
        for part in &vessel.parts {
            if part
                .instance
                .parent
                .is_some_and(|parent| result.contains(&parent))
            {
                result.insert(part.instance.instance_id);
            }
        }
        if result.len() == before {
            break;
        }
    }
    result
}

pub fn activate_next_stage(vessel: &mut Vessel, catalog: &PartCatalog) -> bool {
    let Some(stage) = vessel.stages.get(vessel.next_stage).cloned() else {
        return false;
    };
    for action in stage.actions {
        match action {
            StageAction::ActivateEngine(id) => {
                if let Some(part) = vessel
                    .parts
                    .iter_mut()
                    .find(|part| part.instance.instance_id == id && !part.destroyed)
                {
                    part.active = true;
                }
            }
            StageAction::Decouple(id) => {
                let removed = descendants(vessel, id);
                for part in &mut vessel.parts {
                    if removed.contains(&part.instance.instance_id) {
                        part.destroyed = true;
                        part.active = false;
                    }
                }
            }
            StageAction::DeployParachute(id) => {
                if let Some(part) = vessel
                    .parts
                    .iter_mut()
                    .find(|part| part.instance.instance_id == id && !part.destroyed)
                    && catalog
                        .get(&part.instance.definition_id)
                        .is_some_and(|def| matches!(def.module, PartModule::Parachute { .. }))
                {
                    part.parachute_deployed = true;
                }
            }
        }
    }
    vessel.next_stage += 1;
    true
}

fn atmosphere(body: &CelestialBodyDef, altitude: f64) -> (f64, f64) {
    let Some(atmosphere) = &body.atmosphere else {
        return (0.0, 0.0);
    };
    if altitude >= atmosphere.height {
        return (0.0, 0.0);
    }
    let density =
        atmosphere.sea_level_density * (-altitude.max(0.0) / atmosphere.scale_height).exp();
    let pressure_fraction = density / atmosphere.sea_level_density.max(1e-9);
    (density, pressure_fraction)
}

fn ground_velocity(position: DVec3, rotation_period: f64) -> DVec3 {
    if rotation_period <= 0.0 {
        DVec3::ZERO
    } else {
        (DVec3::Z * (TAU / rotation_period)).cross(position)
    }
}

fn sas_torque(
    vessel: &Vessel,
    attitude: DQuat,
    velocity: DVec3,
    radial: DVec3,
    available: f64,
) -> DVec3 {
    let local_forward = attitude * DVec3::Y;
    let target = match vessel.controls.sas {
        Some(SasMode::Prograde) if velocity.length_squared() > 1.0 => velocity.normalize(),
        Some(SasMode::Retrograde) if velocity.length_squared() > 1.0 => -velocity.normalize(),
        Some(SasMode::RadialOut) => radial,
        Some(SasMode::RadialIn) => -radial,
        Some(SasMode::Normal) => radial.cross(velocity).normalize_or_zero(),
        Some(SasMode::AntiNormal) => -radial.cross(velocity).normalize_or_zero(),
        _ => local_forward,
    };
    local_forward.cross(target) * available * 1.8
}

pub fn step_vessel(
    vessel: &mut Vessel,
    catalog: &PartCatalog,
    dt: f64,
    ut: f64,
) -> FlightTelemetry {
    if matches!(vessel.situation, FlightSituation::Crashed) {
        return telemetry(vessel, catalog, ut, 0.0);
    }
    let mut position = vessel.position_vec();
    let mut velocity = vessel.velocity_vec();
    let mut attitude = vessel.attitude_quat().normalize();
    let mut angular_velocity = vessel.angular_velocity_vec();
    let body = body_definition(&vessel.primary_body);
    let altitude = position.length() - body.radius;
    let radial = position.normalize_or_zero();
    let (density, pressure_fraction) = atmosphere(&body, altitude);
    let atmosphere_velocity = ground_velocity(position, body.rotation_period);
    let air_velocity = velocity - atmosphere_velocity;
    let air_speed = air_velocity.length();
    let mass = vessel_mass(vessel, catalog);

    let mut force = -body.mu * mass / position.length_squared().max(1.0) * radial;
    let mut torque = DVec3::ZERO;
    let mut liquid_thrust = 0.0;
    let mut solid_thrust = 0.0;
    let mut liquid_request = 0.0;
    let throttle = vessel.controls.throttle.clamp(0.0, 1.0);

    for part in vessel
        .parts
        .iter_mut()
        .filter(|part| part.active && !part.destroyed)
    {
        let Some(def) = catalog.get(&part.instance.definition_id) else {
            continue;
        };
        match def.module {
            PartModule::LiquidEngine {
                thrust_vac,
                thrust_sl,
                isp_vac,
                isp_sl,
                ..
            } => {
                let thrust = (thrust_vac + (thrust_sl - thrust_vac) * pressure_fraction) * throttle;
                let isp = isp_vac + (isp_sl - isp_vac) * pressure_fraction;
                liquid_request += thrust / (isp * G0).max(1.0) * dt;
                liquid_thrust += thrust;
            }
            PartModule::SolidEngine { thrust, isp, .. } if part.fuel > 0.0 => {
                let burn = (thrust / (isp * G0) * dt).min(part.fuel);
                let fraction = if dt > 0.0 {
                    burn * isp * G0 / (thrust * dt)
                } else {
                    0.0
                };
                part.fuel -= burn;
                solid_thrust += thrust * fraction;
                if part.fuel <= 1e-6 {
                    part.active = false;
                }
            }
            _ => {}
        }
    }
    let drained = drain_resource(vessel, catalog, true, liquid_request);
    if liquid_request > 0.0 {
        liquid_thrust *= (drained / liquid_request).clamp(0.0, 1.0);
    }
    let total_thrust = liquid_thrust + solid_thrust;
    let forward = attitude * DVec3::Y;
    force += forward * total_thrust;

    let mut control_torque = 0.0;
    let mut fin_lift = 0.0;
    let mut drag_area = 0.0;
    let mut chute_area = 0.0;
    let (_, _, monopropellant) = resource_totals(vessel, catalog);
    let rcs_available = monopropellant > 1e-6;
    let mut unsafe_chutes = Vec::new();
    for part in vessel.parts.iter().filter(|part| !part.destroyed) {
        let Some(def) = catalog.get(&part.instance.definition_id) else {
            continue;
        };
        drag_area += PI * def.radius as f64 * def.radius as f64 * def.drag_coefficient;
        match def.module {
            PartModule::Command { torque, .. } | PartModule::ReactionWheel { torque } => {
                control_torque += torque
            }
            PartModule::Fin { lift, steerable } => {
                fin_lift += lift * if steerable { 1.35 } else { 1.0 };
                if steerable {
                    control_torque += density * air_speed * air_speed * 0.6;
                }
            }
            PartModule::Parachute {
                drag_area,
                safe_speed,
            } if part.parachute_deployed => {
                if air_speed <= safe_speed || density < 0.02 {
                    chute_area += drag_area;
                } else {
                    unsafe_chutes.push(part.instance.instance_id);
                }
            }
            PartModule::Rcs { thrust } if vessel.controls.rcs && rcs_available => {
                control_torque += thrust * 2.0
            }
            _ => {}
        }
    }
    control_torque += total_thrust * 0.045;
    if vessel.controls.rcs
        && rcs_available
        && (vessel.controls.pitch.abs() + vessel.controls.yaw.abs() + vessel.controls.roll.abs()
            > 0.01
            || vessel.controls.sas.is_some())
    {
        drain_resource(vessel, catalog, false, 0.18 * dt);
    }
    for id in unsafe_chutes {
        if let Some(part) = vessel
            .parts
            .iter_mut()
            .find(|part| part.instance.instance_id == id)
        {
            part.destroyed = true;
        }
    }

    if air_speed > 0.1 {
        let dynamic_pressure = 0.5 * density * air_speed * air_speed;
        let air_direction = air_velocity.normalize();
        force -= air_direction * dynamic_pressure * (drag_area + chute_area);
        force += aerodynamic_lift_force(forward, air_direction, dynamic_pressure, fin_lift);
        torque -= angular_velocity * dynamic_pressure * fin_lift * 0.05;
    }

    let local_input = DVec3::new(
        vessel.controls.pitch,
        vessel.controls.roll,
        -vessel.controls.yaw,
    );
    torque += attitude * local_input * control_torque;
    if vessel.controls.sas.is_some() && local_input.length_squared() < 1e-4 {
        torque += sas_torque(vessel, attitude, velocity, radial, control_torque);
        torque -= angular_velocity * control_torque * 0.7;
    }

    let inertia = (mass * 18.0).max(1.0);
    angular_velocity += torque / inertia * dt;
    angular_velocity *= (-0.02 * dt).exp();
    let angle = angular_velocity.length() * dt;
    if angle > 1e-12 {
        attitude =
            (DQuat::from_axis_angle(angular_velocity.normalize(), angle) * attitude).normalize();
    }

    let acceleration = force / mass;
    velocity += acceleration * dt;
    position += velocity * dt;

    let heat_rate = if density > 0.0 {
        1.1e-7 * density.sqrt() * air_speed.powi(3)
    } else {
        0.0
    };
    vessel.max_heating = heat_rate;
    let mut failed = Vec::new();
    let shield_available = vessel
        .parts
        .iter()
        .any(|part| !part.destroyed && part.ablator > 0.0);
    for part in vessel.parts.iter_mut().filter(|part| !part.destroyed) {
        let Some(def) = catalog.get(&part.instance.definition_id) else {
            continue;
        };
        let protection = if shield_available && !matches!(def.module, PartModule::HeatShield { .. })
        {
            0.22
        } else {
            1.0
        };
        part.temperature +=
            (heat_rate * protection - 0.018 * (part.temperature - 290.0).max(0.0)) * dt;
        if matches!(def.module, PartModule::HeatShield { .. })
            && heat_rate > 25.0
            && part.ablator > 0.0
        {
            let used = (heat_rate * 0.0025 * dt).min(part.ablator);
            part.ablator -= used;
            part.temperature = (part.temperature - used * 1.8).max(290.0);
        }
        if part.temperature > def.max_temperature {
            failed.push(part.instance.instance_id);
        }
    }
    for root in failed {
        let removed = descendants(vessel, root);
        for part in &mut vessel.parts {
            if removed.contains(&part.instance.instance_id) {
                part.destroyed = true;
                part.active = false;
            }
        }
    }
    let new_altitude = position.length() - body.radius;
    if new_altitude <= 5.0 {
        let surface_velocity = ground_velocity(position, body.rotation_period);
        let impact_speed = (velocity - surface_velocity).length();
        position = position.normalize_or_zero() * (body.radius + 5.0);
        if vessel.situation == FlightSituation::Prelaunch && total_thrust < mass * G0 * 1.02 {
            velocity = surface_velocity;
        } else if impact_speed > 18.0 {
            vessel.situation = FlightSituation::Crashed;
            velocity = surface_velocity;
        } else {
            vessel.situation = FlightSituation::Landed;
            velocity = surface_velocity;
        }
    } else if vessel.situation == FlightSituation::Prelaunch
        || vessel.situation == FlightSituation::Landed
    {
        vessel.situation = FlightSituation::Flying;
    }

    let el = elements(position, velocity, body.mu, body.radius);
    let atmosphere_height = body
        .atmosphere
        .as_ref()
        .map_or(0.0, |atmosphere| atmosphere.height);
    if new_altitude > atmosphere_height && el.periapsis > atmosphere_height {
        vessel.situation = FlightSituation::Orbiting;
    } else if !matches!(
        vessel.situation,
        FlightSituation::Crashed | FlightSituation::Landed
    ) {
        vessel.situation = FlightSituation::Flying;
    }
    let command_survives = vessel.parts.iter().any(|part| {
        !part.destroyed
            && catalog
                .get(&part.instance.definition_id)
                .is_some_and(|def| matches!(def.module, PartModule::Command { .. }))
    });
    if !command_survives {
        vessel.situation = FlightSituation::Crashed;
    }

    vessel.position = position.to_array();
    vessel.velocity = velocity.to_array();
    vessel.attitude = [attitude.x, attitude.y, attitude.z, attitude.w];
    vessel.angular_velocity = angular_velocity.to_array();
    apply_soi_transitions(vessel, ut);
    telemetry(vessel, catalog, ut, total_thrust)
}

pub fn step_on_rails(vessel: &mut Vessel, dt: f64) {
    if matches!(vessel.situation, FlightSituation::Crashed) {
        return;
    }
    let body = body_definition(&vessel.primary_body);
    let surface_encountered =
        trajectory_reaches_surface(vessel.position_vec(), vessel.velocity_vec(), &body, dt);
    let (position, velocity) =
        propagate_universal(vessel.position_vec(), vessel.velocity_vec(), body.mu, dt);
    vessel.position = position.to_array();
    vessel.velocity = velocity.to_array();
    update_on_rails_situation(vessel, surface_encountered);
}

pub fn step_on_rails_patched(vessel: &mut Vessel, ut: f64, dt: f64) {
    let previous_body = vessel.primary_body.clone();
    step_on_rails(vessel, dt);
    if matches!(vessel.situation, FlightSituation::Crashed) {
        return;
    }
    apply_soi_transitions(vessel, ut + dt);
    // A patched-conic transition changes both the body's surface/atmosphere and
    // the osculating orbit, so the old situation cannot be carried across it.
    let new_body = body_definition(&vessel.primary_body);
    let captured = new_body.parent == Some(previous_body.as_str());
    let orbit = elements(
        vessel.position_vec(),
        vessel.velocity_vec(),
        new_body.mu,
        new_body.radius,
    );
    // Entering a child's SOI inbound and emerging outbound in one rails frame
    // means periapsis also occurred in that frame. Do not let a body impact be
    // skipped merely because both endpoints happened to be above the terrain.
    let crossed_surface_during_capture = captured
        && vessel.position_vec().dot(vessel.velocity_vec()) > 0.0
        && orbit.periapsis <= 5.0;
    update_on_rails_situation(vessel, crossed_surface_during_capture);
}

/// Whether the vessel can safely take another analytic high-warp step.
///
/// Checking only the current altitude is insufficient: a single high-warp
/// frame can span an entire descent from space to below the surface. Bound
/// trajectories with a low periapsis, and inbound escape trajectories, must
/// return to the fixed-step simulation before that can happen.
pub fn on_rails_warp_is_safe(vessel: &Vessel) -> bool {
    if !matches!(vessel.situation, FlightSituation::Orbiting) {
        return false;
    }

    let body = body_definition(&vessel.primary_body);
    let position = vessel.position_vec();
    let velocity = vessel.velocity_vec();
    let atmosphere_height = body
        .atmosphere
        .as_ref()
        .map_or(0.0, |atmosphere| atmosphere.height);
    let minimum_altitude = atmosphere_height + ON_RAILS_CLEARANCE;
    let altitude = position.length() - body.radius;
    if altitude < minimum_altitude {
        return false;
    }

    let orbit = elements(position, velocity, body.mu, body.radius);
    let will_revisit_periapsis = orbit.period.is_some() || position.dot(velocity) < 0.0;
    !will_revisit_periapsis || orbit.periapsis >= minimum_altitude
}

fn time_until_periapsis(position: DVec3, velocity: DVec3, mu: f64) -> Option<f64> {
    let orbit = elements(position, velocity, mu, 0.0);
    if orbit.eccentricity <= 1e-12 {
        return None;
    }

    if orbit.specific_energy < 0.0 {
        let semi_major_axis = orbit.semi_major_axis;
        let cos_eccentric_anomaly =
            ((1.0 - position.length() / semi_major_axis) / orbit.eccentricity).clamp(-1.0, 1.0);
        let sin_eccentric_anomaly =
            position.dot(velocity) / (orbit.eccentricity * (mu * semi_major_axis).sqrt());
        let eccentric_anomaly = sin_eccentric_anomaly
            .atan2(cos_eccentric_anomaly)
            .rem_euclid(TAU);
        let mean_anomaly =
            (eccentric_anomaly - orbit.eccentricity * sin_eccentric_anomaly).rem_euclid(TAU);
        let mean_motion = (mu / semi_major_axis.powi(3)).sqrt();
        Some((TAU - mean_anomaly) / mean_motion)
    } else if position.dot(velocity) < 0.0 && orbit.semi_major_axis.is_finite() {
        let semi_major_axis = -orbit.semi_major_axis;
        let sinh_hyperbolic_anomaly =
            position.dot(velocity) / (orbit.eccentricity * (mu * semi_major_axis).sqrt());
        let hyperbolic_anomaly = sinh_hyperbolic_anomaly.asinh();
        let mean_anomaly = orbit.eccentricity * sinh_hyperbolic_anomaly - hyperbolic_anomaly;
        let mean_motion = (mu / semi_major_axis.powi(3)).sqrt();
        Some(-mean_anomaly / mean_motion)
    } else {
        None
    }
}

fn trajectory_reaches_surface(
    position: DVec3,
    velocity: DVec3,
    body: &CelestialBodyDef,
    dt: f64,
) -> bool {
    if position.length() <= body.radius + 5.0 {
        return true;
    }

    let orbit = elements(position, velocity, body.mu, body.radius);
    if orbit.periapsis > 5.0 {
        return false;
    }

    time_until_periapsis(position, velocity, body.mu).is_some_and(|time| time <= dt)
}

fn update_on_rails_situation(vessel: &mut Vessel, surface_encountered: bool) {
    let body = body_definition(&vessel.primary_body);
    let mut position = vessel.position_vec();
    let altitude = position.length() - body.radius;

    if surface_encountered || altitude <= 5.0 {
        let surface_direction = if position.length_squared() > f64::EPSILON {
            position.normalize()
        } else {
            DVec3::Y
        };
        position = surface_direction * (body.radius + 5.0);
        vessel.position = position.to_array();
        vessel.velocity = ground_velocity(position, body.rotation_period).to_array();
        vessel.situation = FlightSituation::Crashed;
        return;
    }

    let atmosphere_height = body
        .atmosphere
        .as_ref()
        .map_or(0.0, |atmosphere| atmosphere.height);
    let orbit = elements(position, vessel.velocity_vec(), body.mu, body.radius);
    vessel.situation = if altitude > atmosphere_height && orbit.periapsis > atmosphere_height {
        FlightSituation::Orbiting
    } else {
        FlightSituation::Flying
    };
}

pub fn apply_soi_transitions(vessel: &mut Vessel, ut: f64) {
    let current = body_definition(&vessel.primary_body);
    let position = vessel.position_vec();
    let velocity = vessel.velocity_vec();

    if let Some(parent_id) = current.parent {
        let parent = body_definition(parent_id);
        let soi = sphere_of_influence(current.semi_major_axis, current.mu, parent.mu);
        if position.length() > soi * 1.01 {
            let (body_position, body_velocity) = circular_ephemeris(&current, parent.mu, ut);
            vessel.position = (body_position + position).to_array();
            vessel.velocity = (body_velocity + velocity).to_array();
            vessel.primary_body = parent_id.into();
            return;
        }
    }

    for child in celestial_system()
        .into_iter()
        .filter(|body| body.parent == Some(current.id))
    {
        let soi = sphere_of_influence(child.semi_major_axis, child.mu, current.mu);
        let (child_position, child_velocity) = circular_ephemeris(&child, current.mu, ut);
        if (position - child_position).length() < soi * 0.98 {
            vessel.position = (position - child_position).to_array();
            vessel.velocity = (velocity - child_velocity).to_array();
            vessel.primary_body = child.id.into();
            return;
        }
    }
}

pub fn telemetry(vessel: &Vessel, catalog: &PartCatalog, ut: f64, thrust: f64) -> FlightTelemetry {
    let body = body_definition(&vessel.primary_body);
    let position = vessel.position_vec();
    let velocity = vessel.velocity_vec();
    let altitude = position.length() - body.radius;
    let surface_velocity = ground_velocity(position, body.rotation_period);
    let relative = velocity - surface_velocity;
    let (density, _) = atmosphere(&body, altitude);
    let mass = vessel_mass(vessel, catalog);
    let (liquid, solid, mono) = resource_totals(vessel, catalog);
    FlightTelemetry {
        ut,
        altitude,
        radar_altitude: altitude,
        speed: velocity.length(),
        surface_speed: relative.length(),
        vertical_speed: relative.dot(position.normalize_or_zero()),
        mach: relative.length() / 343.0,
        dynamic_pressure: 0.5 * density * relative.length_squared(),
        heating: vessel.max_heating,
        mass,
        liquid_fuel: liquid,
        solid_fuel: solid,
        monopropellant: mono,
        thrust,
        twr: thrust / (mass * G0).max(1.0),
        orbit: elements(position, velocity, body.mu, body.radius),
    }
}

pub fn update_mission(
    progress: &mut MissionProgress,
    vessel: &Vessel,
    telemetry: &FlightTelemetry,
) {
    progress.launched |= telemetry.altitude > 50.0;
    progress.staged |= vessel.next_stage >= 2;
    progress.achieved_orbit |=
        vessel.primary_body == "carapace" && telemetry.orbit.periapsis >= 75_000.0;
    progress.began_reentry |= vessel.primary_body == "carapace"
        && progress.achieved_orbit
        && telemetry.altitude < HOME_ATMOSPHERE;
    progress.recovered |= progress.began_reentry && vessel.situation == FlightSituation::Landed;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{PartCatalog, Vessel, stock_craft};
    use approx::assert_abs_diff_eq;

    #[test]
    fn aerodynamic_lift_scales_with_crossflow() {
        let lift_at = |angle_degrees: f64| {
            let angle = angle_degrees.to_radians();
            let forward = DVec3::new(angle.sin(), angle.cos(), 0.0);
            aerodynamic_lift_force(forward, DVec3::Y, 1.0, 1.0).length()
        };

        assert_eq!(lift_at(0.0), 0.0);
        assert!(lift_at(0.001) < 0.000_018);
        assert_abs_diff_eq!(lift_at(30.0), 3.0_f64.sqrt() / 4.0, epsilon = 1e-12);
        assert!(lift_at(90.0) < 1e-12);
    }

    #[test]
    fn staging_activates_then_sheds_boosters() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        assert!(activate_next_stage(&mut vessel, &catalog));
        assert!(
            vessel
                .parts
                .iter()
                .find(|p| p.instance.instance_id == 11)
                .unwrap()
                .active
        );
        assert!(activate_next_stage(&mut vessel, &catalog));
        assert!(
            vessel
                .parts
                .iter()
                .find(|p| p.instance.instance_id == 11)
                .unwrap()
                .destroyed
        );
    }

    #[test]
    fn engine_burn_consumes_fuel_and_lifts() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        activate_next_stage(&mut vessel, &catalog);
        vessel.controls.throttle = 1.0;
        let before = resource_totals(&vessel, &catalog);
        for i in 0..120 {
            step_vessel(&mut vessel, &catalog, 1.0 / 60.0, i as f64 / 60.0);
        }
        let after = resource_totals(&vessel, &catalog);
        assert!(after.0 < before.0);
        assert!(after.1 < before.1);
        assert!(vessel.position_vec().length() > crate::model::HOME_RADIUS + 5.0);
    }

    #[test]
    fn safe_stage_does_not_duplicate() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        for _ in 0..8 {
            activate_next_stage(&mut vessel, &catalog);
        }
        assert_eq!(vessel.next_stage, vessel.stages.len());
    }

    #[test]
    fn patched_conic_exit_preserves_parent_frame_state() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        let home = body_definition("carapace");
        let star = body_definition("pelagos");
        let soi = sphere_of_influence(home.semi_major_axis, home.mu, star.mu);
        vessel.position = [soi * 1.02, 0.0, 0.0];
        vessel.velocity = [0.0, 100.0, 0.0];
        let relative_position = vessel.position_vec();
        let (home_position, _) = circular_ephemeris(&home, star.mu, 0.0);
        apply_soi_transitions(&mut vessel, 0.0);
        assert_eq!(vessel.primary_body, "pelagos");
        assert!((vessel.position_vec() - (home_position + relative_position)).length() < 1e-4);
    }

    #[test]
    fn high_warp_rejects_an_orbit_that_will_enter_the_atmosphere() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        let body = body_definition("carapace");
        let apoapsis_radius = body.radius + 200_000.0;
        let periapsis_radius = body.radius + 40_000.0;
        let semi_major_axis = (apoapsis_radius + periapsis_radius) * 0.5;
        vessel.position = [apoapsis_radius, 0.0, 0.0];
        vessel.velocity = [
            0.0,
            (body.mu * (2.0 / apoapsis_radius - 1.0 / semi_major_axis)).sqrt(),
            0.0,
        ];
        vessel.situation = FlightSituation::Orbiting;

        assert!(!on_rails_warp_is_safe(&vessel));
    }

    #[test]
    fn on_rails_step_reclassifies_an_unsafe_orbit() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        let body = body_definition("carapace");
        let apoapsis_radius = body.radius + 200_000.0;
        let periapsis_radius = body.radius + 40_000.0;
        let semi_major_axis = (apoapsis_radius + periapsis_radius) * 0.5;
        vessel.position = [apoapsis_radius, 0.0, 0.0];
        vessel.velocity = [
            0.0,
            (body.mu * (2.0 / apoapsis_radius - 1.0 / semi_major_axis)).sqrt(),
            0.0,
        ];
        vessel.situation = FlightSituation::Orbiting;

        step_on_rails(&mut vessel, 1.0);

        assert_eq!(vessel.situation, FlightSituation::Flying);
    }

    #[test]
    fn on_rails_surface_encounter_crashes_even_when_the_endpoint_is_above_ground() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        let body = body_definition("carapace");
        let apoapsis_radius = body.radius + 200_000.0;
        let periapsis_radius = body.radius - 100_000.0;
        let semi_major_axis = (apoapsis_radius + periapsis_radius) * 0.5;
        let period = TAU * (semi_major_axis.powi(3) / body.mu).sqrt();
        vessel.position = [apoapsis_radius, 0.0, 0.0];
        vessel.velocity = [
            0.0,
            (body.mu * (2.0 / apoapsis_radius - 1.0 / semi_major_axis)).sqrt(),
            0.0,
        ];
        vessel.situation = FlightSituation::Orbiting;

        // One full period returns the analytic endpoint to the starting point,
        // but the intervening trajectory passes through the body.
        step_on_rails(&mut vessel, period);

        assert_eq!(vessel.situation, FlightSituation::Crashed);
        assert_abs_diff_eq!(
            vessel.position_vec().length(),
            body.radius + 5.0,
            epsilon = 1e-6
        );
    }

    #[test]
    fn soi_capture_re_evaluates_situation_for_the_new_body() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        let home = body_definition("carapace");
        let moon = body_definition("selene");
        let (moon_position, moon_velocity) = circular_ephemeris(&moon, home.mu, 0.0);
        vessel.position = (moon_position + DVec3::X * 300_000.0).to_array();
        vessel.velocity = moon_velocity.to_array();
        vessel.situation = FlightSituation::Orbiting;

        step_on_rails_patched(&mut vessel, 0.0, 0.0);

        assert_eq!(vessel.primary_body, "selene");
        assert_eq!(vessel.situation, FlightSituation::Flying);
        assert!(!on_rails_warp_is_safe(&vessel));
    }

    #[test]
    fn atmospheric_entry_generates_heat() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        vessel.position = [0.0, crate::model::HOME_RADIUS + 30_000.0, 0.0];
        vessel.velocity = [2_400.0, 0.0, 0.0];
        let before = vessel.parts[0].temperature;
        let telemetry = step_vessel(&mut vessel, &catalog, 0.1, 0.1);
        assert!(telemetry.heating > 0.0);
        assert!(vessel.parts[0].temperature > before);
    }
}
