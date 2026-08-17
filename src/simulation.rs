use bevy::math::{DMat3, DQuat, DVec3};
use bevy::prelude::Resource;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::{PI, TAU};

use crate::model::{
    CraftBlueprint, DetachedStage, FlightSituation, HOME_ATMOSPHERE, PartCatalog, PartDefinition,
    PartModule, SasMode, StageAction, Vessel,
};
use crate::orbit::{
    CelestialBodyDef, OrbitalElements, body_definition, celestial_system, circular_ephemeris,
    elements, propagate_universal, sphere_of_influence,
};

const G0: f64 = 9.80665;
const ON_RAILS_CLEARANCE: f64 = 5_000.0;
const SOI_CAPTURE_FACTOR: f64 = 0.98;
const SOI_RELEASE_FACTOR: f64 = 1.01;
const SOI_SWEEP_SEGMENT_FRACTION: f64 = 0.25;
const SOI_SWEEP_MAX_SEGMENTS: usize = 4_096;
const MAX_SOI_TRANSITIONS_PER_STEP: usize = 8;
const HEAT_CAPACITY_RATIO: f64 = 1.4;
const UPRIGHT_VERTICAL_IMPACT_LIMIT: f64 = 18.0;
const BROADSIDE_VERTICAL_IMPACT_LIMIT: f64 = 8.0;
const HORIZONTAL_IMPACT_LIMIT: f64 = 8.0;
const LANDING_LEG_VERTICAL_BONUS: f64 = 8.0;
const LANDING_LEG_HORIZONTAL_BONUS: f64 = 3.0;
const INLINE_COAXIAL_TOLERANCE: f64 = 0.05;
const INLINE_AXIS_ALIGNMENT: f64 = 0.98;

fn aerodynamic_lift_force(
    forward: DVec3,
    air_direction: DVec3,
    dynamic_pressure: f64,
    fin_lift: f64,
) -> DVec3 {
    let crossflow = forward.reject_from(air_direction);
    crossflow * dynamic_pressure * fin_lift * forward.dot(air_direction).abs()
}

fn projected_drag_area(
    definition: &PartDefinition,
    part: &crate::model::RuntimePart,
    vessel: &Vessel,
    catalog: &PartCatalog,
    world_rotation: DQuat,
    air_direction: DVec3,
) -> f64 {
    let local_direction = world_rotation.conjugate() * air_direction.normalize_or_zero();
    let radius = f64::from(definition.radius);
    let height = f64::from(definition.height);
    let area = if definition.radial
        || matches!(
            definition.module,
            PartModule::Fin { .. } | PartModule::LandingLeg | PartModule::Rcs { .. }
        ) {
        let width = radius * 1.2;
        let depth = radius * 0.65;
        local_direction.x.abs() * height * depth
            + local_direction.y.abs() * width * depth
            + local_direction.z.abs() * width * height
    } else {
        let axial = local_direction.y.abs();
        let position = DVec3::from_array(part.instance.local_position.map(f64::from));
        let local_rotation = DQuat::from_array(part.instance.local_rotation.map(f64::from));
        let axis = local_rotation * DVec3::Y;
        let leading_sign = local_direction.y.signum();
        let blocked_radius = if leading_sign == 0.0 {
            0.0
        } else {
            vessel
                .parts
                .iter()
                .filter(|neighbor| {
                    !neighbor.destroyed
                        && neighbor.instance.instance_id != part.instance.instance_id
                        && (neighbor.instance.parent == Some(part.instance.instance_id)
                            || part.instance.parent == Some(neighbor.instance.instance_id))
                })
                .filter_map(|neighbor| {
                    let neighbor_definition = catalog.get(&neighbor.instance.definition_id)?;
                    if neighbor_definition.radial
                        || matches!(
                            neighbor_definition.module,
                            PartModule::Fin { .. }
                                | PartModule::LandingLeg
                                | PartModule::Rcs { .. }
                        )
                    {
                        return None;
                    }
                    let neighbor_rotation =
                        DQuat::from_array(neighbor.instance.local_rotation.map(f64::from));
                    let neighbor_axis = neighbor_rotation * DVec3::Y;
                    if axis.dot(neighbor_axis).abs() < INLINE_AXIS_ALIGNMENT {
                        return None;
                    }
                    let neighbor_position =
                        DVec3::from_array(neighbor.instance.local_position.map(f64::from));
                    let offset = neighbor_position - position;
                    if offset.dot(axis) * leading_sign <= 0.0
                        || offset.reject_from(axis).length() > INLINE_COAXIAL_TOLERANCE
                    {
                        return None;
                    }
                    Some(f64::from(neighbor_definition.radius))
                })
                .fold(0.0, f64::max)
                .min(radius)
        };
        PI * (radius * radius - blocked_radius * blocked_radius) * axial
            + 2.0 * radius * height * (1.0 - axial * axial).max(0.0).sqrt()
    };
    area * definition.drag_coefficient
}

fn heat_rate(body: &CelestialBodyDef, position: DVec3, velocity: DVec3) -> f64 {
    let altitude = position.length() - body.radius;
    let (density, _) = atmosphere(body, altitude);
    if density <= 0.0 {
        return 0.0;
    }
    let relative = velocity - ground_velocity(position, body.rotation_period);
    1.1e-7 * density.sqrt() * relative.length().powi(3)
}

fn speed_of_sound(body: &CelestialBodyDef, altitude: f64) -> Option<f64> {
    let atmosphere = body.atmosphere.as_ref()?;
    if altitude >= atmosphere.height {
        return None;
    }
    let radius = body.radius + altitude.max(0.0);
    let gravity = body.mu / radius.powi(2);
    // For the isothermal atmospheres used here, H = R*T/g and
    // c = sqrt(gamma*R*T), so the existing scale height supplies a
    // body-specific sound speed without another independent tuning value.
    Some((HEAT_CAPACITY_RATIO * gravity * atmosphere.scale_height).sqrt())
}

#[derive(Debug, Clone, Copy)]
struct MassProperties {
    mass: f64,
    center: DVec3,
    inertia: DMat3,
}

fn part_resource_capacity(module: PartModule) -> f64 {
    match module {
        PartModule::LiquidTank { fuel }
        | PartModule::MonopropTank { fuel }
        | PartModule::SolidEngine { fuel, .. } => fuel,
        PartModule::HeatShield { ablator } => ablator,
        _ => 0.0,
    }
}

fn outer_product(vector: DVec3) -> DMat3 {
    DMat3::from_cols(vector * vector.x, vector * vector.y, vector * vector.z)
}

fn compound_mass_properties(vessel: &Vessel, catalog: &PartCatalog) -> MassProperties {
    let parts: Vec<_> = vessel
        .parts
        .iter()
        .filter(|part| !part.destroyed)
        .filter_map(|part| {
            let definition = catalog.get(&part.instance.definition_id)?;
            let mass = definition.dry_mass + part.fuel + part.ablator;
            let position = DVec3::from_array(part.instance.local_position.map(f64::from));
            Some((part, definition, mass, position))
        })
        .collect();
    let mass: f64 = parts.iter().map(|(_, _, mass, _)| mass).sum();
    if mass <= f64::EPSILON {
        return MassProperties {
            mass: 1.0,
            center: DVec3::ZERO,
            inertia: DMat3::IDENTITY,
        };
    }

    let center = parts
        .iter()
        .map(|(_, _, part_mass, position)| *position * *part_mass)
        .sum::<DVec3>()
        / mass;
    let mut inertia = DMat3::ZERO;
    for (part, definition, part_mass, position) in parts {
        // Match the simple cylinder/cuboid geometry rendered by the editor. Rotate
        // each part's intrinsic tensor, then use the parallel-axis theorem.
        let radius = f64::from(definition.radius);
        let height = f64::from(definition.height);
        let intrinsic = if definition.radial
            || matches!(
                definition.module,
                PartModule::Fin { .. } | PartModule::LandingLeg | PartModule::Rcs { .. }
            ) {
            let width = radius * 1.2;
            let depth = radius * 0.65;
            DMat3::from_diagonal(DVec3::new(
                part_mass * (height * height + depth * depth) / 12.0,
                part_mass * (width * width + depth * depth) / 12.0,
                part_mass * (width * width + height * height) / 12.0,
            ))
        } else {
            let transverse = part_mass * (3.0 * radius * radius + height * height) / 12.0;
            let axial = 0.5 * part_mass * radius * radius;
            DMat3::from_diagonal(DVec3::new(transverse, axial, transverse))
        };
        let rotation = DMat3::from_quat(DQuat::from_array(
            part.instance.local_rotation.map(f64::from),
        ));
        let offset = position - center;
        inertia += rotation * intrinsic * rotation.transpose()
            + part_mass * (DMat3::IDENTITY * offset.length_squared() - outer_product(offset));
    }

    MassProperties {
        mass,
        center,
        inertia,
    }
}

fn angular_acceleration(
    attitude: DQuat,
    angular_velocity: DVec3,
    torque: DVec3,
    inertia: DMat3,
) -> DVec3 {
    let inverse_attitude = attitude.conjugate();
    let local_velocity = inverse_attitude * angular_velocity;
    let local_torque = inverse_attitude * torque;
    let angular_momentum = inertia * local_velocity;
    let local_acceleration =
        inertia.inverse() * (local_torque - local_velocity.cross(angular_momentum));
    attitude * local_acceleration
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
    /// The guided mission reached safe recovery after orbit and atmospheric reentry.
    /// Physical touchdown readiness is represented by `FlightSituation::Landed`.
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
    let stats_vessel = Vessel::from_blueprint(craft, catalog);
    let first_propulsion_stage = craft.stages.iter().find_map(|stage| {
        let ids = stage
            .actions
            .iter()
            .filter_map(|action| match action {
                StageAction::ActivateEngine(id) => Some(*id),
                _ => None,
            })
            .collect::<BTreeSet<_>>();
        (!ids.is_empty()).then_some(ids)
    });
    let mut weighted_mass_y = 0.0;
    let mut pressure_area = 0.0;
    let mut pressure_y = 0.0;
    let mut thrust_y = 0.0;
    let mut thrust_weight = 0.0;
    for part in &craft.parts {
        let Some(def) = catalog.get(&part.definition_id) else {
            continue;
        };
        stats.dry_mass += def.dry_mass;
        let wet_part_mass = def.dry_mass + part_resource_capacity(def.module);
        stats.wet_mass += wet_part_mass;
        weighted_mass_y += wet_part_mass * part.local_position[1] as f64;
        let area = stats_vessel
            .parts
            .iter()
            .find(|runtime| runtime.instance.instance_id == part.instance_id)
            .map_or(0.0, |runtime| {
                projected_drag_area(
                    def,
                    runtime,
                    &stats_vessel,
                    catalog,
                    DQuat::from_array(part.local_rotation.map(f64::from)),
                    DVec3::Y,
                )
            });
        pressure_area += area;
        pressure_y += area * part.local_position[1] as f64;
        let contributes_initial_thrust = first_propulsion_stage
            .as_ref()
            .is_none_or(|ids| ids.contains(&part.instance_id));
        match def.module {
            PartModule::LiquidTank { fuel } => stats.liquid_fuel += fuel,
            PartModule::SolidEngine { thrust, fuel, .. } => {
                stats.solid_fuel += fuel;
                if contributes_initial_thrust {
                    stats.sea_level_thrust += thrust;
                    stats.vacuum_thrust += thrust;
                    thrust_y += thrust * part.local_position[1] as f64;
                    thrust_weight += thrust;
                }
            }
            PartModule::LiquidEngine {
                thrust_vac,
                thrust_sl,
                ..
            } if contributes_initial_thrust => {
                stats.sea_level_thrust += thrust_sl;
                stats.vacuum_thrust += thrust_vac;
                thrust_y += thrust_vac * part.local_position[1] as f64;
                thrust_weight += thrust_vac;
            }
            _ => {}
        }
    }
    stats.center_of_mass_y = if stats.wet_mass > 0.0 {
        weighted_mass_y / stats.wet_mass
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
    stats.vacuum_delta_v = staged_vacuum_delta_v(craft, catalog);
    stats
}

pub fn vessel_mass(vessel: &Vessel, catalog: &PartCatalog) -> f64 {
    compound_mass_properties(vessel, catalog).mass
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

fn drain_monopropellant(vessel: &mut Vessel, catalog: &PartCatalog, amount: f64) -> f64 {
    let available: f64 = vessel
        .parts
        .iter()
        .filter(|part| !part.destroyed)
        .filter_map(|part| {
            let def = catalog.get(&part.instance.definition_id)?;
            matches!(def.module, PartModule::MonopropTank { .. }).then_some(part.fuel)
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
        if matches!(def.module, PartModule::MonopropTank { .. }) {
            part.fuel = (part.fuel - taken * part.fuel / available).max(0.0);
        }
    }
    taken
}

fn blocks_liquid_crossfeed(part: &crate::model::RuntimePart, catalog: &PartCatalog) -> bool {
    catalog
        .get(&part.instance.definition_id)
        .is_some_and(|definition| {
            matches!(
                definition.module,
                PartModule::InlineDecoupler { .. } | PartModule::RadialDecoupler { .. }
            )
        })
}

fn liquid_fuel_network(vessel: &Vessel, catalog: &PartCatalog, start: u64) -> BTreeSet<u64> {
    let Some(start_part) = vessel.parts.iter().find(|part| {
        part.instance.instance_id == start
            && !part.destroyed
            && !blocks_liquid_crossfeed(part, catalog)
    }) else {
        return BTreeSet::new();
    };
    let mut network = BTreeSet::from([start_part.instance.instance_id]);
    loop {
        let before = network.len();
        for candidate in vessel
            .parts
            .iter()
            .filter(|part| !part.destroyed && !blocks_liquid_crossfeed(part, catalog))
        {
            let candidate_id = candidate.instance.instance_id;
            if network.contains(&candidate_id) {
                continue;
            }
            let connected = candidate
                .instance
                .parent
                .is_some_and(|parent| network.contains(&parent))
                || vessel.parts.iter().any(|member| {
                    network.contains(&member.instance.instance_id)
                        && member.instance.parent == Some(candidate_id)
                });
            if connected {
                network.insert(candidate_id);
            }
        }
        if network.len() == before {
            return network;
        }
    }
}

fn drain_liquid_fuel_network(
    vessel: &mut Vessel,
    catalog: &PartCatalog,
    network: &BTreeSet<u64>,
    amount: f64,
) -> f64 {
    let available: f64 = vessel
        .parts
        .iter()
        .filter(|part| !part.destroyed && network.contains(&part.instance.instance_id))
        .filter_map(|part| {
            let definition = catalog.get(&part.instance.definition_id)?;
            matches!(definition.module, PartModule::LiquidTank { .. }).then_some(part.fuel)
        })
        .sum();
    let taken = available.min(amount);
    if available <= 0.0 {
        return 0.0;
    }
    for part in vessel
        .parts
        .iter_mut()
        .filter(|part| !part.destroyed && network.contains(&part.instance.instance_id))
    {
        let Some(definition) = catalog.get(&part.instance.definition_id) else {
            continue;
        };
        if matches!(definition.module, PartModule::LiquidTank { .. }) {
            part.fuel = (part.fuel - taken * part.fuel / available).max(0.0);
        }
    }
    taken
}

fn liquid_fuel_available(vessel: &Vessel, catalog: &PartCatalog, network: &BTreeSet<u64>) -> f64 {
    vessel
        .parts
        .iter()
        .filter(|part| !part.destroyed && network.contains(&part.instance.instance_id))
        .filter_map(|part| {
            let definition = catalog.get(&part.instance.definition_id)?;
            matches!(definition.module, PartModule::LiquidTank { .. }).then_some(part.fuel)
        })
        .sum()
}

fn burn_vacuum_interval(vessel: &mut Vessel, catalog: &PartCatalog) -> Option<f64> {
    let mut liquid_networks: BTreeMap<u64, (BTreeSet<u64>, f64, f64)> = BTreeMap::new();
    let mut solid_burns = Vec::new();

    for part in vessel
        .parts
        .iter()
        .filter(|part| part.active && !part.destroyed)
    {
        let Some(definition) = catalog.get(&part.instance.definition_id) else {
            continue;
        };
        match definition.module {
            PartModule::LiquidEngine {
                thrust_vac,
                isp_vac,
                ..
            } if thrust_vac > 0.0 && isp_vac > 0.0 => {
                let network = liquid_fuel_network(vessel, catalog, part.instance.instance_id);
                let network_id = network
                    .first()
                    .copied()
                    .unwrap_or(part.instance.instance_id);
                let mass_flow = thrust_vac / (isp_vac * G0);
                liquid_networks
                    .entry(network_id)
                    .and_modify(|(_, thrust, flow)| {
                        *thrust += thrust_vac;
                        *flow += mass_flow;
                    })
                    .or_insert((network, thrust_vac, mass_flow));
            }
            PartModule::SolidEngine { thrust, isp, .. }
                if part.fuel > f64::EPSILON && thrust > 0.0 && isp > 0.0 =>
            {
                solid_burns.push((part.instance.instance_id, thrust, thrust / (isp * G0)));
            }
            _ => {}
        }
    }

    liquid_networks.retain(|_, (network, _, flow)| {
        *flow > 0.0 && liquid_fuel_available(vessel, catalog, network) > f64::EPSILON
    });
    let total_thrust = liquid_networks
        .values()
        .map(|(_, thrust, _)| thrust)
        .sum::<f64>()
        + solid_burns.iter().map(|(_, thrust, _)| thrust).sum::<f64>();
    let total_mass_flow = liquid_networks
        .values()
        .map(|(_, _, flow)| flow)
        .sum::<f64>()
        + solid_burns.iter().map(|(_, _, flow)| flow).sum::<f64>();
    if total_thrust <= 0.0 || total_mass_flow <= 0.0 {
        return None;
    }

    let mut duration = f64::INFINITY;
    for (network, _, flow) in liquid_networks.values() {
        duration = duration.min(liquid_fuel_available(vessel, catalog, network) / flow);
    }
    for (id, _, flow) in &solid_burns {
        if let Some(part) = vessel
            .parts
            .iter()
            .find(|part| part.instance.instance_id == *id)
        {
            duration = duration.min(part.fuel / flow);
        }
    }
    if !duration.is_finite() || duration <= 0.0 {
        return None;
    }

    let initial_mass = vessel_mass(vessel, catalog);
    for (network, _, flow) in liquid_networks.values() {
        drain_liquid_fuel_network(vessel, catalog, network, flow * duration);
    }
    for (id, _, flow) in solid_burns {
        if let Some(part) = vessel
            .parts
            .iter_mut()
            .find(|part| part.instance.instance_id == id)
        {
            part.fuel = (part.fuel - flow * duration).max(0.0);
            if part.fuel <= f64::EPSILON {
                part.active = false;
            }
        }
    }
    let final_mass = vessel_mass(vessel, catalog);
    (initial_mass > final_mass && final_mass > 0.0)
        .then_some(total_thrust / total_mass_flow * (initial_mass / final_mass).ln())
}

fn staged_vacuum_delta_v(craft: &CraftBlueprint, catalog: &PartCatalog) -> f64 {
    let mut vessel = Vessel::from_blueprint(craft, catalog);
    let has_engine_actions = craft.stages.iter().any(|stage| {
        stage
            .actions
            .iter()
            .any(|action| matches!(action, StageAction::ActivateEngine(_)))
    });
    if !has_engine_actions {
        for part in &mut vessel.parts {
            if catalog
                .get(&part.instance.definition_id)
                .is_some_and(|definition| {
                    matches!(
                        definition.module,
                        PartModule::LiquidEngine { .. } | PartModule::SolidEngine { .. }
                    )
                })
            {
                part.active = true;
            }
        }
    }

    let mut delta_v = 0.0;
    if has_engine_actions {
        while vessel.next_stage < vessel.stages.len() {
            activate_next_stage(&mut vessel, catalog);
            if let Some(interval_delta_v) = burn_vacuum_interval(&mut vessel, catalog) {
                delta_v += interval_delta_v;
            }
        }
    }
    while let Some(interval_delta_v) = burn_vacuum_interval(&mut vessel, catalog) {
        delta_v += interval_delta_v;
    }
    delta_v
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

fn decouple(vessel: &mut Vessel, catalog: &PartCatalog, root: u64) {
    let Some(root_part) = vessel
        .parts
        .iter()
        .find(|part| part.instance.instance_id == root && !part.destroyed)
    else {
        return;
    };
    let Some(definition) = catalog.get(&root_part.instance.definition_id) else {
        return;
    };
    let root_rotation = root_part.instance.local_rotation;
    let impulse = match definition.module {
        PartModule::InlineDecoupler { impulse } | PartModule::RadialDecoupler { impulse } => {
            impulse
        }
        _ => return,
    };

    let removed = descendants(vessel, root);
    let mut detached_mass = 0.0;
    let mut detached_center = DVec3::ZERO;
    for part in vessel
        .parts
        .iter()
        .filter(|part| !part.destroyed && removed.contains(&part.instance.instance_id))
    {
        let Some(definition) = catalog.get(&part.instance.definition_id) else {
            continue;
        };
        let mass = definition.dry_mass + part.fuel + part.ablator;
        let position = DVec3::from_array(part.instance.local_position.map(f64::from));
        detached_mass += mass;
        detached_center += position * mass;
    }
    if detached_mass <= f64::EPSILON {
        return;
    }
    detached_center /= detached_mass;

    let mut detached_parts = Vec::new();
    for part in &mut vessel.parts {
        if removed.contains(&part.instance.instance_id) && !part.destroyed {
            let mut detached = part.clone();
            let position = DVec3::from_array(detached.instance.local_position.map(f64::from))
                - detached_center;
            detached.instance.local_position = position.as_vec3().to_array();
            detached_parts.push(detached);
            part.destroyed = true;
            part.active = false;
        }
    }

    let remaining = compound_mass_properties(vessel, catalog);
    let local_separation = (detached_center - remaining.center).normalize_or_zero();
    let local_separation = if local_separation.length_squared() > 0.0 {
        local_separation
    } else {
        DQuat::from_array(root_rotation.map(f64::from)) * -DVec3::Y
    };
    let attitude = vessel.attitude_quat().normalize();
    let world_offset = attitude * detached_center;
    let separation = attitude * local_separation;
    let original_velocity = vessel.velocity_vec();
    let detached_velocity = original_velocity
        + vessel.angular_velocity_vec().cross(world_offset)
        + separation * (impulse / detached_mass);
    vessel.velocity = (original_velocity - separation * (impulse / remaining.mass)).to_array();
    vessel.debris.push(DetachedStage {
        primary_body: vessel.primary_body.clone(),
        parts: detached_parts,
        position: (vessel.position_vec() + world_offset).to_array(),
        velocity: detached_velocity.to_array(),
        attitude: vessel.attitude,
        angular_velocity: vessel.angular_velocity,
    });
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
                decouple(vessel, catalog, id);
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

fn touchdown_is_survivable(
    vessel: &Vessel,
    catalog: &PartCatalog,
    attitude: DQuat,
    radial: DVec3,
    surface_relative_velocity: DVec3,
) -> bool {
    let upright = (attitude * DVec3::Y).dot(radial).clamp(0.0, 1.0);
    let leg_count = vessel
        .parts
        .iter()
        .filter(|part| !part.destroyed)
        .filter(|part| {
            catalog
                .get(&part.instance.definition_id)
                .is_some_and(|definition| matches!(definition.module, PartModule::LandingLeg))
        })
        .count();
    let leg_support = (leg_count as f64 / 3.0).clamp(0.0, 1.0) * upright;
    let vertical_limit = BROADSIDE_VERTICAL_IMPACT_LIMIT
        + (UPRIGHT_VERTICAL_IMPACT_LIMIT - BROADSIDE_VERTICAL_IMPACT_LIMIT) * upright
        + LANDING_LEG_VERTICAL_BONUS * leg_support;
    let horizontal_limit = HORIZONTAL_IMPACT_LIMIT + LANDING_LEG_HORIZONTAL_BONUS * leg_support;
    let vertical_speed = surface_relative_velocity.dot(radial).min(0.0).abs();
    let horizontal_speed = surface_relative_velocity.reject_from(radial).length();
    // Combine the vertical and lateral envelopes rather than allowing both
    // components to independently reach their maximum survivable value.
    (vertical_speed / vertical_limit).powi(2) + (horizontal_speed / horizontal_limit).powi(2) <= 1.0
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
    let pre_burn_mass_properties = compound_mass_properties(vessel, catalog);

    let mut force = DVec3::ZERO;
    let mut torque = DVec3::ZERO;
    let mut applied_thrust = Vec::new();
    let mut liquid_networks: BTreeMap<u64, (BTreeSet<u64>, f64)> = BTreeMap::new();
    let throttle = vessel.controls.throttle.clamp(0.0, 1.0);

    for part in vessel
        .parts
        .iter()
        .filter(|part| part.active && !part.destroyed)
    {
        let Some(def) = catalog.get(&part.instance.definition_id) else {
            continue;
        };
        let PartModule::LiquidEngine {
            thrust_vac,
            thrust_sl,
            isp_vac,
            isp_sl,
            gimbal_deg,
            ..
        } = def.module
        else {
            continue;
        };
        let thrust = (thrust_vac + (thrust_sl - thrust_vac) * pressure_fraction) * throttle;
        let isp = isp_vac + (isp_sl - isp_vac) * pressure_fraction;
        let request = thrust / (isp * G0).max(1.0) * dt;
        let network = liquid_fuel_network(vessel, catalog, part.instance.instance_id);
        let network_id = network
            .first()
            .copied()
            .unwrap_or(part.instance.instance_id);
        liquid_networks
            .entry(network_id)
            .and_modify(|(_, total_request)| *total_request += request)
            .or_insert((network, request));
        applied_thrust.push((
            DVec3::from_array(part.instance.local_position.map(f64::from)),
            DQuat::from_array(part.instance.local_rotation.map(f64::from)) * DVec3::Y,
            thrust,
            gimbal_deg,
            Some(network_id),
        ));
    }

    for part in vessel
        .parts
        .iter_mut()
        .filter(|part| part.active && !part.destroyed)
    {
        let Some(def) = catalog.get(&part.instance.definition_id) else {
            continue;
        };
        match def.module {
            PartModule::SolidEngine { thrust, isp, .. } if part.fuel > 0.0 => {
                let burn = (thrust / (isp * G0) * dt).min(part.fuel);
                let fraction = if dt > 0.0 {
                    burn * isp * G0 / (thrust * dt)
                } else {
                    0.0
                };
                part.fuel -= burn;
                applied_thrust.push((
                    DVec3::from_array(part.instance.local_position.map(f64::from)),
                    DQuat::from_array(part.instance.local_rotation.map(f64::from)) * DVec3::Y,
                    thrust * fraction,
                    0.0,
                    None,
                ));
                if part.fuel <= 1e-6 {
                    part.active = false;
                }
            }
            _ => {}
        }
    }
    let liquid_fractions: BTreeMap<_, _> = liquid_networks
        .into_iter()
        .map(|(network_id, (network, request))| {
            let drained = drain_liquid_fuel_network(vessel, catalog, &network, request);
            let fraction = if request > 0.0 {
                (drained / request).clamp(0.0, 1.0)
            } else {
                1.0
            };
            (network_id, fraction)
        })
        .collect();
    let post_burn_mass_properties = compound_mass_properties(vessel, catalog);
    // Midpoint properties keep the burn first-order accurate while avoiding
    // the systematic pre-burn-mass bias of sampling only at frame start.
    let mass_properties = MassProperties {
        mass: (pre_burn_mass_properties.mass + post_burn_mass_properties.mass) * 0.5,
        center: (pre_burn_mass_properties.center + post_burn_mass_properties.center) * 0.5,
        inertia: (pre_burn_mass_properties.inertia + post_burn_mass_properties.inertia) * 0.5,
    };
    let mass = mass_properties.mass;
    let gimbal_input = DVec3::new(vessel.controls.pitch, 0.0, -vessel.controls.yaw);
    let gimbal_amount = gimbal_input.length().min(1.0);
    let mut total_thrust = 0.0;
    for (point, base_direction, mut thrust, gimbal_deg, liquid_network) in applied_thrust {
        if let Some(network_id) = liquid_network {
            thrust *= liquid_fractions.get(&network_id).copied().unwrap_or(0.0);
        }
        let offset = point - mass_properties.center;
        let mut direction = base_direction.normalize_or_zero();
        if gimbal_deg > 0.0 && gimbal_amount > 1e-6 {
            let lateral = gimbal_input
                .cross(offset)
                .reject_from(direction)
                .normalize_or_zero();
            if lateral.length_squared() > 0.0 {
                let angle = gimbal_deg.to_radians() * gimbal_amount;
                direction = direction * angle.cos() + lateral * angle.sin();
            }
        }
        let thrust_force = attitude * direction * thrust;
        force += thrust_force;
        torque += (attitude * offset).cross(thrust_force);
        total_thrust += thrust;
    }
    let forward = attitude * DVec3::Y;

    let mut control_torque = 0.0;
    let mut rcs_mass_flow = 0.0;
    let (_, _, monopropellant) = resource_totals(vessel, catalog);
    let rcs_available = monopropellant > 1e-6;
    let mut unsafe_chutes = Vec::new();
    for part in vessel.parts.iter().filter(|part| !part.destroyed) {
        let Some(def) = catalog.get(&part.instance.definition_id) else {
            continue;
        };
        let offset =
            DVec3::from_array(part.instance.local_position.map(f64::from)) - mass_properties.center;
        let world_offset = attitude * offset;
        let part_air_velocity = air_velocity + angular_velocity.cross(world_offset);
        let part_air_speed = part_air_velocity.length();
        let part_air_direction = part_air_velocity.normalize_or_zero();
        let part_dynamic_pressure = 0.5 * density * part_air_speed * part_air_speed;
        if part_air_speed > 0.1 {
            let part_rotation =
                attitude * DQuat::from_array(part.instance.local_rotation.map(f64::from));
            let drag_area = projected_drag_area(
                def,
                part,
                vessel,
                catalog,
                part_rotation,
                part_air_direction,
            );
            let drag_force = -part_air_direction * part_dynamic_pressure * drag_area;
            force += drag_force;
            torque += world_offset.cross(drag_force);
        }
        match def.module {
            PartModule::Command { torque, .. } | PartModule::ReactionWheel { torque } => {
                control_torque += torque
            }
            PartModule::Fin { lift, steerable } => {
                if part_air_speed > 0.1 {
                    let fin_lift = lift * if steerable { 1.35 } else { 1.0 };
                    let lift_force = aerodynamic_lift_force(
                        forward,
                        part_air_direction,
                        part_dynamic_pressure,
                        fin_lift,
                    );
                    force += lift_force;
                    torque += world_offset.cross(lift_force);
                }
                if steerable {
                    control_torque += density * air_speed * air_speed * 0.6;
                }
            }
            PartModule::Parachute {
                drag_area,
                safe_speed,
            } if part.parachute_deployed => {
                if part_air_speed <= safe_speed || density < 0.02 {
                    if part_air_speed > 0.1 {
                        let chute_force = -part_air_direction * part_dynamic_pressure * drag_area;
                        force += chute_force;
                        torque += world_offset.cross(chute_force);
                    }
                } else {
                    unsafe_chutes.push(part.instance.instance_id);
                }
            }
            PartModule::Rcs { thrust, isp } if vessel.controls.rcs && rcs_available => {
                control_torque += thrust * 2.0;
                rcs_mass_flow += thrust / (isp * G0).max(1.0);
            }
            _ => {}
        }
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

    let local_input = DVec3::new(
        vessel.controls.pitch,
        vessel.controls.roll,
        -vessel.controls.yaw,
    );
    let mut applied_control_torque = attitude * local_input * control_torque;
    if vessel.controls.sas.is_some() && local_input.length_squared() < 1e-4 {
        applied_control_torque += sas_torque(vessel, attitude, velocity, radial, control_torque);
        applied_control_torque -= angular_velocity * control_torque * 0.7;
    }
    torque += applied_control_torque;
    if vessel.controls.rcs && rcs_available && rcs_mass_flow > 0.0 {
        let effort = (applied_control_torque.length() / control_torque.max(1.0)).clamp(0.0, 1.0);
        drain_monopropellant(vessel, catalog, rcs_mass_flow * effort * dt);
    }

    angular_velocity +=
        angular_acceleration(attitude, angular_velocity, torque, mass_properties.inertia) * dt;
    let angle = angular_velocity.length() * dt;
    if angle > 1e-12 {
        attitude =
            (DQuat::from_axis_angle(angular_velocity.normalize(), angle) * attitude).normalize();
    }

    let acceleration = -body.mu / position.length_squared().max(1.0) * radial + force / mass;
    velocity += acceleration * dt;
    position += velocity * dt;

    let heat_rate = heat_rate(&body, position, velocity);
    vessel.max_heating = vessel.max_heating.max(heat_rate);
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
        let surface_relative_velocity = velocity - surface_velocity;
        position = position.normalize_or_zero() * (body.radius + 5.0);
        if vessel.situation == FlightSituation::Prelaunch && total_thrust < mass * G0 * 1.02 {
            velocity = surface_velocity;
        } else if !touchdown_is_survivable(
            vessel,
            catalog,
            attitude,
            position.normalize_or_zero(),
            surface_relative_velocity,
        ) {
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
    step_debris(vessel, dt);
    apply_soi_transitions(vessel, ut + dt);
    telemetry(vessel, catalog, ut + dt, total_thrust)
}

fn step_debris(vessel: &mut Vessel, dt: f64) {
    for debris in &mut vessel.debris {
        let body = body_definition(&debris.primary_body);
        let mut position = debris.position_vec();
        let mut velocity = debris.velocity_vec();
        let radial = position.normalize_or_zero();
        velocity += -body.mu / position.length_squared().max(1.0) * radial * dt;
        position += velocity * dt;
        if position.length() <= body.radius + 5.0 {
            position = position.normalize_or_zero() * (body.radius + 5.0);
            velocity = ground_velocity(position, body.rotation_period);
        }

        let angular_velocity = DVec3::from_array(debris.angular_velocity);
        let angle = angular_velocity.length() * dt;
        let mut attitude = debris.attitude_quat();
        if angle > 1e-12 {
            attitude = (DQuat::from_axis_angle(angular_velocity.normalize(), angle) * attitude)
                .normalize();
        }
        debris.position = position.to_array();
        debris.velocity = velocity.to_array();
        debris.attitude = attitude.to_array();
    }
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
    for debris in &mut vessel.debris {
        let debris_body = body_definition(&debris.primary_body);
        let (position, velocity) = propagate_universal(
            debris.position_vec(),
            debris.velocity_vec(),
            debris_body.mu,
            dt,
        );
        debris.position = position.to_array();
        debris.velocity = velocity.to_array();
    }
    update_on_rails_situation(vessel, surface_encountered);
}

pub fn step_on_rails_patched(vessel: &mut Vessel, ut: f64, dt: f64) {
    let previous_body = vessel.primary_body.clone();
    let mut elapsed = 0.0;
    for _ in 0..MAX_SOI_TRANSITIONS_PER_STEP {
        let remaining = dt - elapsed;
        if remaining <= f64::EPSILON {
            break;
        }

        let current = body_definition(&vessel.primary_body);
        let transition = next_soi_transition(
            vessel.position_vec(),
            vessel.velocity_vec(),
            &current,
            ut + elapsed,
            remaining,
        );
        let Some((transition_time, target, entering)) = transition else {
            step_on_rails(vessel, remaining);
            elapsed = dt;
            break;
        };

        step_on_rails(vessel, transition_time);
        if matches!(vessel.situation, FlightSituation::Crashed) {
            return;
        }

        let transition_ut = ut + elapsed + transition_time;
        let (body_position, body_velocity) = if entering {
            circular_ephemeris(&target, current.mu, transition_ut)
        } else {
            circular_ephemeris(&current, target.mu, transition_ut)
        };
        if entering {
            vessel.position = (vessel.position_vec() - body_position).to_array();
            vessel.velocity = (vessel.velocity_vec() - body_velocity).to_array();
        } else {
            vessel.position = (body_position + vessel.position_vec()).to_array();
            vessel.velocity = (body_velocity + vessel.velocity_vec()).to_array();
        }
        vessel.primary_body = target.id.into();
        elapsed += transition_time;
    }
    if elapsed < dt && !matches!(vessel.situation, FlightSituation::Crashed) {
        step_on_rails(vessel, dt - elapsed);
    }
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

fn next_soi_transition(
    position: DVec3,
    velocity: DVec3,
    current: &CelestialBodyDef,
    ut: f64,
    dt: f64,
) -> Option<(f64, CelestialBodyDef, bool)> {
    let entering = first_child_soi_encounter(position, velocity, current, ut, dt)
        .map(|(time, child)| (time, child, true));
    let exiting = current.parent.and_then(|parent_id| {
        let parent = body_definition(parent_id);
        current_soi_exit_time(position, velocity, current, &parent, dt)
            .map(|time| (time, parent, false))
    });

    match (entering, exiting) {
        (Some(entering), Some(exiting)) => Some(if entering.0 <= exiting.0 {
            entering
        } else {
            exiting
        }),
        (Some(entering), None) => Some(entering),
        (None, Some(exiting)) => Some(exiting),
        (None, None) => None,
    }
}

fn first_child_soi_encounter(
    position: DVec3,
    velocity: DVec3,
    current: &CelestialBodyDef,
    ut: f64,
    dt: f64,
) -> Option<(f64, CelestialBodyDef)> {
    if dt <= 0.0 {
        return None;
    }

    celestial_system()
        .into_iter()
        .filter(|body| body.parent == Some(current.id))
        .filter_map(|child| {
            child_soi_encounter_time(position, velocity, current, &child, ut, dt)
                .map(|time| (time, child))
        })
        .min_by(|(a, _), (b, _)| a.total_cmp(b))
}

fn child_soi_encounter_time(
    position: DVec3,
    velocity: DVec3,
    current: &CelestialBodyDef,
    child: &CelestialBodyDef,
    ut: f64,
    dt: f64,
) -> Option<f64> {
    let soi = sphere_of_influence(child.semi_major_axis, child.mu, current.mu) * SOI_CAPTURE_FACTOR;
    let relative_state_at = |elapsed: f64| {
        let (vessel_position, vessel_velocity) =
            propagate_universal(position, velocity, current.mu, elapsed);
        let (child_position, child_velocity) = circular_ephemeris(child, current.mu, ut + elapsed);
        (
            vessel_position - child_position,
            vessel_velocity - child_velocity,
        )
    };

    let (mut previous_position, start_velocity) = relative_state_at(0.0);
    if previous_position.length() < soi {
        return Some(0.0);
    }
    let (_, end_velocity) = relative_state_at(dt);
    let estimated_travel = start_velocity.length().max(end_velocity.length()) * dt;
    let segment_length = soi * SOI_SWEEP_SEGMENT_FRACTION;
    let segments =
        ((estimated_travel / segment_length).ceil() as usize).clamp(8, SOI_SWEEP_MAX_SEGMENTS);

    let mut previous_time = 0.0;
    for index in 1..=segments {
        let time = dt * index as f64 / segments as f64;
        let (relative_position, _) = relative_state_at(time);

        // The chord is a cheap broad phase. Its closest point also catches a
        // complete outside-to-outside crossing, unlike endpoint sampling.
        if segment_distance_to_origin(previous_position, relative_position) < soi * 1.1 {
            let closest_time = minimize_distance(previous_time, time, &relative_state_at);
            if relative_state_at(closest_time).0.length() < soi {
                let mut outside = previous_time;
                let mut inside = closest_time;
                if relative_state_at(outside).0.length() < soi {
                    return Some(outside);
                }
                for _ in 0..48 {
                    let midpoint = (outside + inside) * 0.5;
                    if relative_state_at(midpoint).0.length() < soi {
                        inside = midpoint;
                    } else {
                        outside = midpoint;
                    }
                }
                return Some(inside);
            }
        }

        previous_time = time;
        previous_position = relative_position;
    }
    None
}

fn current_soi_exit_time(
    position: DVec3,
    velocity: DVec3,
    current: &CelestialBodyDef,
    parent: &CelestialBodyDef,
    dt: f64,
) -> Option<f64> {
    let soi =
        sphere_of_influence(current.semi_major_axis, current.mu, parent.mu) * SOI_RELEASE_FACTOR;
    if position.length() > soi {
        return Some(0.0);
    }

    let orbit = elements(position, velocity, current.mu, 0.0);
    if orbit.apoapsis.is_finite() && orbit.apoapsis <= soi {
        return None;
    }

    let (_, end_velocity) = propagate_universal(position, velocity, current.mu, dt);
    let estimated_travel = velocity.length().max(end_velocity.length()) * dt;
    let segment_length = soi * SOI_SWEEP_SEGMENT_FRACTION;
    let segments =
        ((estimated_travel / segment_length).ceil() as usize).clamp(8, SOI_SWEEP_MAX_SEGMENTS);
    let mut previous_time = 0.0;

    for index in 1..=segments {
        let time = dt * index as f64 / segments as f64;
        let (next_position, _) = propagate_universal(position, velocity, current.mu, time);
        if next_position.length() > soi {
            let mut inside = previous_time;
            let mut outside = time;
            for _ in 0..48 {
                let midpoint = (inside + outside) * 0.5;
                if propagate_universal(position, velocity, current.mu, midpoint)
                    .0
                    .length()
                    > soi
                {
                    outside = midpoint;
                } else {
                    inside = midpoint;
                }
            }
            return Some(outside);
        }
        previous_time = time;
    }
    None
}

fn segment_distance_to_origin(start: DVec3, end: DVec3) -> f64 {
    let segment = end - start;
    if segment.length_squared() <= f64::EPSILON {
        return start.length();
    }
    let fraction = (-start.dot(segment) / segment.length_squared()).clamp(0.0, 1.0);
    (start + segment * fraction).length()
}

fn minimize_distance(
    mut start: f64,
    mut end: f64,
    relative_state_at: &impl Fn(f64) -> (DVec3, DVec3),
) -> f64 {
    for _ in 0..32 {
        let first = start + (end - start) / 3.0;
        let second = end - (end - start) / 3.0;
        if relative_state_at(first).0.length_squared()
            < relative_state_at(second).0.length_squared()
        {
            end = second;
        } else {
            start = first;
        }
    }
    (start + end) * 0.5
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
        if position.length() > soi * SOI_RELEASE_FACTOR {
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
        if (position - child_position).length() < soi * SOI_CAPTURE_FACTOR {
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
    let mach = speed_of_sound(&body, altitude)
        .map(|sound_speed| relative.length() / sound_speed)
        .unwrap_or(0.0);
    FlightTelemetry {
        ut,
        altitude,
        radar_altitude: altitude,
        speed: velocity.length(),
        surface_speed: relative.length(),
        vertical_speed: relative.dot(position.normalize_or_zero()),
        mach,
        dynamic_pressure: 0.5 * density * relative.length_squared(),
        heating: heat_rate(&body, position, velocity),
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
    use crate::model::{CraftBlueprint, PartCatalog, PartInstance, Stage, Vessel, stock_craft};
    use approx::assert_abs_diff_eq;

    fn part(id: u64, definition_id: &str, position: [f32; 3]) -> PartInstance {
        PartInstance {
            instance_id: id,
            definition_id: definition_id.into(),
            parent: (id != 1).then_some(1),
            local_position: position,
            local_rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    fn child_part(
        id: u64,
        definition_id: &str,
        parent: Option<u64>,
        position: [f32; 3],
    ) -> PartInstance {
        PartInstance {
            instance_id: id,
            definition_id: definition_id.into(),
            parent,
            local_position: position,
            local_rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    #[test]
    fn baseline_solids_have_physical_burn_times_and_stock_liftoff_twr() {
        let catalog = PartCatalog::default();
        let expected = [
            ("solid_stack", 74.0..75.0, 6.6..6.8),
            ("solid_radial", 69.0..70.0, 7.5..7.7),
        ];
        for (id, burn_range, mass_ratio_range) in expected {
            let definition = catalog.get(id).unwrap();
            let PartModule::SolidEngine {
                thrust, isp, fuel, ..
            } = definition.module
            else {
                panic!("{id} is not a solid engine")
            };
            let burn_time = fuel * isp * G0 / thrust;
            let wet_to_dry = (definition.dry_mass + fuel) / definition.dry_mass;
            assert!(burn_range.contains(&burn_time), "{id}: {burn_time}");
            assert!(mass_ratio_range.contains(&wet_to_dry), "{id}: {wet_to_dry}");
        }

        let stats = craft_stats(&stock_craft(), &catalog);
        assert!(
            (2.1..=2.2).contains(&stats.sea_level_twr),
            "stock liftoff TWR was {}",
            stats.sea_level_twr
        );
    }

    #[test]
    fn craft_stats_model_parallel_engines_and_connected_tank_crossfeed() {
        let catalog = PartCatalog::default();
        let parallel = CraftBlueprint {
            schema_version: 1,
            name: "Parallel engines".into(),
            parts: vec![
                child_part(1, "tank_long", None, [0.0, 1.0, 0.0]),
                child_part(2, "engine_sl_s", Some(1), [-0.5, -1.0, 0.0]),
                child_part(3, "engine_sl_s", Some(1), [0.5, -1.0, 0.0]),
            ],
            stages: vec![Stage {
                name: "Ignition".into(),
                actions: vec![
                    StageAction::ActivateEngine(2),
                    StageAction::ActivateEngine(3),
                ],
            }],
            crew: Vec::new(),
            script_name: None,
        };
        let parallel_stats = craft_stats(&parallel, &catalog);
        let PartModule::LiquidEngine {
            thrust_vac,
            isp_vac,
            ..
        } = catalog.get("engine_sl_s").unwrap().module
        else {
            unreachable!()
        };
        let parallel_dry = catalog.get("tank_long").unwrap().dry_mass
            + 2.0 * catalog.get("engine_sl_s").unwrap().dry_mass;
        let parallel_expected =
            isp_vac * G0 * ((parallel_dry + parallel_stats.liquid_fuel) / parallel_dry).ln();
        assert_abs_diff_eq!(
            parallel_stats.vacuum_thrust,
            2.0 * thrust_vac,
            epsilon = 1e-12
        );
        assert_abs_diff_eq!(
            parallel_stats.vacuum_delta_v,
            parallel_expected,
            epsilon = 1e-9
        );

        let crossfeed = CraftBlueprint {
            schema_version: 1,
            name: "Connected tanks".into(),
            parts: vec![
                child_part(1, "tank_long", None, [0.0, 2.0, 0.0]),
                child_part(2, "tank_short", Some(1), [0.0, -1.0, 0.0]),
                child_part(3, "engine_sl_s", Some(2), [0.0, -3.0, 0.0]),
            ],
            stages: Vec::new(),
            crew: Vec::new(),
            script_name: None,
        };
        let crossfeed_stats = craft_stats(&crossfeed, &catalog);
        let crossfeed_dry = catalog.get("tank_long").unwrap().dry_mass
            + catalog.get("tank_short").unwrap().dry_mass
            + catalog.get("engine_sl_s").unwrap().dry_mass;
        let crossfeed_expected =
            isp_vac * G0 * ((crossfeed_dry + crossfeed_stats.liquid_fuel) / crossfeed_dry).ln();
        assert_abs_diff_eq!(
            crossfeed_stats.vacuum_delta_v,
            crossfeed_expected,
            epsilon = 1e-9
        );
    }

    #[test]
    fn craft_stats_center_of_mass_includes_all_resources() {
        let catalog = PartCatalog::default();
        let craft = CraftBlueprint {
            schema_version: 1,
            name: "Mass test".into(),
            parts: vec![
                part(1, "pod_1", [0.0, 0.0, 0.0]),
                part(2, "tank_short", [0.0, 10.0, 0.0]),
                part(3, "mono_tank", [0.0, 20.0, 0.0]),
                part(4, "heatshield", [0.0, 30.0, 0.0]),
            ],
            stages: Vec::new(),
            crew: Vec::new(),
            script_name: None,
        };

        let expected_mass = 900.0 + (280.0 + 2_400.0) + (90.0 + 220.0) + (220.0 + 240.0);
        let expected_center =
            ((280.0 + 2_400.0) * 10.0 + (90.0 + 220.0) * 20.0 + (220.0 + 240.0) * 30.0)
                / expected_mass;
        let stats = craft_stats(&craft, &catalog);

        assert_abs_diff_eq!(stats.wet_mass, expected_mass, epsilon = 1e-12);
        assert_abs_diff_eq!(stats.center_of_mass_y, expected_center, epsilon = 1e-12);
    }

    #[test]
    fn off_axis_engine_applies_torque_about_live_center_of_mass() {
        let catalog = PartCatalog::default();
        let craft = CraftBlueprint {
            schema_version: 1,
            name: "Offset thrust".into(),
            parts: vec![
                part(1, "pod_1", [0.0, 0.0, 0.0]),
                part(2, "solid_stack", [2.0, -2.0, 0.0]),
            ],
            stages: Vec::new(),
            crew: Vec::new(),
            script_name: None,
        };
        let mut vessel = Vessel::from_blueprint(&craft, &catalog);
        vessel.parts[1].active = true;
        vessel.controls.sas = None;
        vessel.position = [
            crate::model::HOME_RADIUS + HOME_ATMOSPHERE + 1_000.0,
            0.0,
            0.0,
        ];
        vessel.velocity = [0.0, 0.0, 0.0];

        step_vessel(&mut vessel, &catalog, 0.001, 0.001);

        assert!(vessel.angular_velocity[2] > 0.0);
        assert_abs_diff_eq!(vessel.angular_velocity[0], 0.0, epsilon = 1e-12);
    }

    #[test]
    fn anisotropic_inertia_includes_euler_gyroscopic_term() {
        let acceleration = angular_acceleration(
            DQuat::IDENTITY,
            DVec3::new(1.0, 2.0, 3.0),
            DVec3::ZERO,
            DMat3::from_diagonal(DVec3::new(2.0, 3.0, 4.0)),
        );

        assert_abs_diff_eq!(acceleration.x, -3.0, epsilon = 1e-12);
        assert_abs_diff_eq!(acceleration.y, 2.0, epsilon = 1e-12);
        assert_abs_diff_eq!(acceleration.z, -0.5, epsilon = 1e-12);
    }

    #[test]
    fn fins_behind_center_of_mass_weathervane_into_airflow() {
        let catalog = PartCatalog::default();
        let craft = CraftBlueprint {
            schema_version: 1,
            name: "Stable aerodynamic test".into(),
            parts: vec![
                part(1, "pod_1", [0.0, 0.0, 0.0]),
                part(2, "fin", [0.0, -4.0, 0.0]),
            ],
            stages: Vec::new(),
            crew: Vec::new(),
            script_name: None,
        };
        let mut vessel = Vessel::from_blueprint(&craft, &catalog);
        vessel.controls.sas = None;
        vessel.situation = FlightSituation::Flying;
        let position = DVec3::new(crate::model::HOME_RADIUS + 10_000.0, 0.0, 0.0);
        vessel.position = position.to_array();
        vessel.velocity = (ground_velocity(position, body_definition("carapace").rotation_period)
            + DVec3::Y * 100.0)
            .to_array();
        vessel.attitude = DQuat::from_rotation_z(0.1).to_array();

        step_vessel(&mut vessel, &catalog, 0.001, 0.001);

        assert!(vessel.angular_velocity[2] < 0.0);
    }

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
        assert_eq!(vessel.debris.len(), 2);
        assert!(vessel.debris.iter().all(|stage| !stage.parts.is_empty()));
    }

    #[test]
    fn separator_impulse_is_equal_and_opposite() {
        let catalog = PartCatalog::default();
        let craft = CraftBlueprint {
            schema_version: 1,
            name: "Separation test".into(),
            parts: vec![
                part(1, "pod_1", [0.0, 0.0, 0.0]),
                part(2, "decoupler_radial", [2.0, 0.0, 0.0]),
                PartInstance {
                    instance_id: 3,
                    definition_id: "solid_radial".into(),
                    parent: Some(2),
                    local_position: [3.0, 0.0, 0.0],
                    local_rotation: [0.0, 0.0, 0.0, 1.0],
                },
            ],
            stages: vec![crate::model::Stage {
                name: "Separate".into(),
                actions: vec![StageAction::Decouple(2)],
            }],
            crew: Vec::new(),
            script_name: None,
        };
        let mut vessel = Vessel::from_blueprint(&craft, &catalog);
        vessel.angular_velocity = [0.0; 3];
        let initial_velocity = vessel.velocity_vec();

        activate_next_stage(&mut vessel, &catalog);

        let survivor_mass = vessel_mass(&vessel, &catalog);
        let debris = &vessel.debris[0];
        let debris_mass: f64 = debris
            .parts
            .iter()
            .map(|part| {
                let definition = catalog.get(&part.instance.definition_id).unwrap();
                definition.dry_mass + part.fuel + part.ablator
            })
            .sum();
        let survivor_momentum = (vessel.velocity_vec() - initial_velocity) * survivor_mass;
        let debris_momentum = (debris.velocity_vec() - initial_velocity) * debris_mass;
        assert_abs_diff_eq!(survivor_momentum.length(), 2_000.0, epsilon = 1e-9);
        assert_abs_diff_eq!(debris_momentum.length(), 2_000.0, epsilon = 1e-9);
        assert!((survivor_momentum + debris_momentum).length() < 1e-9);
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
    fn inline_decoupler_preserves_stock_upper_stage_fuel() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        let fuel = |vessel: &Vessel, id| {
            vessel
                .parts
                .iter()
                .find(|part| part.instance.instance_id == id)
                .unwrap()
                .fuel
        };
        let upper_before = fuel(&vessel, 4);
        let lower_before = fuel(&vessel, 8);

        activate_next_stage(&mut vessel, &catalog);
        vessel.controls.throttle = 1.0;
        step_vessel(&mut vessel, &catalog, 1.0 / 60.0, 0.0);

        assert_eq!(fuel(&vessel, 4), upper_before);
        assert!(fuel(&vessel, 8) < lower_before);

        activate_next_stage(&mut vessel, &catalog);
        activate_next_stage(&mut vessel, &catalog);
        let upper_stage_before = fuel(&vessel, 4);
        step_vessel(&mut vessel, &catalog, 1.0 / 60.0, 1.0 / 60.0);

        assert!(fuel(&vessel, 4) < upper_stage_before);
    }

    #[test]
    fn radial_decoupler_isolates_independent_liquid_fuel_networks() {
        let catalog = PartCatalog::default();
        let craft = CraftBlueprint {
            schema_version: 1,
            name: "Crossfeed test".into(),
            parts: vec![
                part(1, "pod_1", [0.0, 0.0, 0.0]),
                PartInstance {
                    instance_id: 2,
                    definition_id: "tank_short".into(),
                    parent: Some(1),
                    local_position: [0.0, -1.0, 0.0],
                    local_rotation: [0.0, 0.0, 0.0, 1.0],
                },
                PartInstance {
                    instance_id: 3,
                    definition_id: "engine_sl_s".into(),
                    parent: Some(2),
                    local_position: [0.0, -2.0, 0.0],
                    local_rotation: [0.0, 0.0, 0.0, 1.0],
                },
                PartInstance {
                    instance_id: 4,
                    definition_id: "decoupler_radial".into(),
                    parent: Some(2),
                    local_position: [2.0, -1.0, 0.0],
                    local_rotation: [0.0, 0.0, 0.0, 1.0],
                },
                PartInstance {
                    instance_id: 5,
                    definition_id: "tank_long".into(),
                    parent: Some(4),
                    local_position: [3.0, -1.0, 0.0],
                    local_rotation: [0.0, 0.0, 0.0, 1.0],
                },
                PartInstance {
                    instance_id: 6,
                    definition_id: "engine_sl_s".into(),
                    parent: Some(5),
                    local_position: [3.0, -3.0, 0.0],
                    local_rotation: [0.0, 0.0, 0.0, 1.0],
                },
            ],
            stages: Vec::new(),
            crew: Vec::new(),
            script_name: None,
        };
        let mut vessel = Vessel::from_blueprint(&craft, &catalog);
        vessel.parts[1].fuel = 0.0;
        vessel.parts[2].active = true;
        vessel.parts[5].active = true;
        vessel.controls.throttle = 1.0;
        let booster_fuel_before = vessel.parts[4].fuel;
        let body = body_definition("carapace");
        let (_, pressure_fraction) = atmosphere(&body, 5.0);
        let expected_thrust = match catalog.get("engine_sl_s").unwrap().module {
            PartModule::LiquidEngine {
                thrust_vac,
                thrust_sl,
                ..
            } => thrust_vac + (thrust_sl - thrust_vac) * pressure_fraction,
            _ => unreachable!(),
        };

        let data = step_vessel(&mut vessel, &catalog, 1.0 / 60.0, 0.0);

        assert_abs_diff_eq!(data.thrust, expected_thrust, epsilon = 1e-9);
        assert_eq!(vessel.parts[1].fuel, 0.0);
        assert!(vessel.parts[4].fuel < booster_fuel_before);
    }

    #[test]
    fn tanks_in_one_liquid_network_drain_proportionally() {
        let catalog = PartCatalog::default();
        let craft = CraftBlueprint {
            schema_version: 1,
            name: "Shared fuel network".into(),
            parts: vec![
                part(1, "pod_1", [0.0, 0.0, 0.0]),
                PartInstance {
                    instance_id: 2,
                    definition_id: "tank_short".into(),
                    parent: Some(1),
                    local_position: [0.0, -1.0, 0.0],
                    local_rotation: [0.0, 0.0, 0.0, 1.0],
                },
                PartInstance {
                    instance_id: 3,
                    definition_id: "tank_long".into(),
                    parent: Some(2),
                    local_position: [0.0, -3.0, 0.0],
                    local_rotation: [0.0, 0.0, 0.0, 1.0],
                },
                PartInstance {
                    instance_id: 4,
                    definition_id: "engine_sl_s".into(),
                    parent: Some(3),
                    local_position: [0.0, -5.0, 0.0],
                    local_rotation: [0.0, 0.0, 0.0, 1.0],
                },
            ],
            stages: Vec::new(),
            crew: Vec::new(),
            script_name: None,
        };
        let mut vessel = Vessel::from_blueprint(&craft, &catalog);
        let short_before = vessel.parts[1].fuel;
        let long_before = vessel.parts[2].fuel;
        let network = liquid_fuel_network(&vessel, &catalog, 4);

        let drained = drain_liquid_fuel_network(&mut vessel, &catalog, &network, 100.0);

        assert_eq!(drained, 100.0);
        assert_abs_diff_eq!(
            (short_before - vessel.parts[1].fuel) / short_before,
            (long_before - vessel.parts[2].fuel) / long_before,
            epsilon = 1e-12
        );
    }

    #[test]
    fn suborbital_landing_is_not_guided_mission_recovery() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        let mut progress = MissionProgress::default();
        let mut data = FlightTelemetry {
            altitude: 100.0,
            ..Default::default()
        };

        update_mission(&mut progress, &vessel, &data);
        vessel.situation = FlightSituation::Landed;
        data.altitude = 5.0;
        update_mission(&mut progress, &vessel, &data);

        assert!(progress.launched);
        assert!(!progress.achieved_orbit);
        assert!(!progress.began_reentry);
        assert!(!progress.recovered);
    }

    #[test]
    fn orbital_reentry_latches_guided_mission_recovery_on_landing() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        let mut progress = MissionProgress::default();
        activate_next_stage(&mut vessel, &catalog);
        activate_next_stage(&mut vessel, &catalog);
        let mut data = FlightTelemetry {
            altitude: 80_000.0,
            orbit: OrbitalElements {
                periapsis: 75_000.0,
                ..Default::default()
            },
            ..Default::default()
        };

        update_mission(&mut progress, &vessel, &data);
        vessel.situation = FlightSituation::Landed;
        data.altitude = 5.0;
        update_mission(&mut progress, &vessel, &data);

        assert!(progress.launched);
        assert!(progress.staged);
        assert!(progress.achieved_orbit);
        assert!(progress.began_reentry);
        assert!(progress.recovered);
    }

    #[test]
    fn crash_does_not_complete_guided_mission_recovery() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        vessel.situation = FlightSituation::Crashed;
        let mut progress = MissionProgress {
            launched: true,
            staged: true,
            achieved_orbit: true,
            began_reentry: true,
            recovered: false,
        };
        let data = FlightTelemetry {
            altitude: 5.0,
            ..Default::default()
        };

        update_mission(&mut progress, &vessel, &data);

        assert!(!progress.recovered);
    }

    #[test]
    fn burn_acceleration_uses_midpoint_mass() {
        let catalog = PartCatalog::default();
        let craft = CraftBlueprint {
            schema_version: 1,
            name: "Mass integration test".into(),
            parts: vec![
                part(1, "pod_1", [0.0, 0.0, 0.0]),
                part(2, "solid_stack", [0.0, 0.0, 0.0]),
            ],
            stages: Vec::new(),
            crew: Vec::new(),
            script_name: None,
        };
        let mut vessel = Vessel::from_blueprint(&craft, &catalog);
        vessel.parts[1].active = true;
        vessel.controls.sas = None;
        vessel.position = [
            crate::model::HOME_RADIUS + HOME_ATMOSPHERE + 1_000.0,
            0.0,
            0.0,
        ];
        vessel.velocity = [0.0; 3];
        let initial_mass = vessel_mass(&vessel, &catalog);
        let PartModule::SolidEngine { thrust, isp, .. } =
            catalog.get("solid_stack").unwrap().module
        else {
            unreachable!()
        };
        let burn = thrust / (isp * G0);
        let expected_delta_v = thrust / (initial_mass - burn * 0.5);

        step_vessel(&mut vessel, &catalog, 1.0, 0.0);

        assert_abs_diff_eq!(vessel.velocity[1], expected_delta_v, epsilon = 1e-10);
    }

    #[test]
    fn rcs_consumption_scales_with_clusters_and_control_effort() {
        let catalog = PartCatalog::default();
        let make_vessel = |clusters: u64, pitch: f64| {
            let mut parts = vec![child_part(1, "mono_tank", None, [0.0, 0.0, 0.0])];
            for id in 2..clusters + 2 {
                parts.push(child_part(id, "rcs", Some(1), [0.0, 0.0, 0.0]));
            }
            let craft = CraftBlueprint {
                schema_version: 1,
                name: "RCS test".into(),
                parts,
                stages: Vec::new(),
                crew: Vec::new(),
                script_name: None,
            };
            let mut vessel = Vessel::from_blueprint(&craft, &catalog);
            vessel.position = [
                crate::model::HOME_RADIUS + HOME_ATMOSPHERE + 1_000.0,
                0.0,
                0.0,
            ];
            vessel.velocity = [0.0; 3];
            vessel.controls.sas = None;
            vessel.controls.rcs = true;
            vessel.controls.pitch = pitch;
            vessel
        };
        let PartModule::Rcs { thrust, isp } = catalog.get("rcs").unwrap().module else {
            unreachable!()
        };
        let cluster_flow = thrust / (isp * G0);

        let mut one = make_vessel(1, 1.0);
        let one_before = one.parts[0].fuel;
        step_vessel(&mut one, &catalog, 1.0, 0.0);
        let one_used = one_before - one.parts[0].fuel;
        assert_abs_diff_eq!(one_used, cluster_flow, epsilon = 1e-12);

        let mut two = make_vessel(2, 1.0);
        let two_before = two.parts[0].fuel;
        step_vessel(&mut two, &catalog, 1.0, 0.0);
        let two_used = two_before - two.parts[0].fuel;
        assert_abs_diff_eq!(two_used, 2.0 * cluster_flow, epsilon = 1e-12);
        assert!(two.angular_velocity_vec().length() > one.angular_velocity_vec().length());

        let mut quarter = make_vessel(1, 0.25);
        let quarter_before = quarter.parts[0].fuel;
        step_vessel(&mut quarter, &catalog, 1.0, 0.0);
        assert_abs_diff_eq!(
            quarter_before - quarter.parts[0].fuel,
            cluster_flow * 0.25,
            epsilon = 1e-12
        );
    }

    #[test]
    fn rcs_uses_no_propellant_without_demand_and_stops_at_exhaustion() {
        let catalog = PartCatalog::default();
        let craft = CraftBlueprint {
            schema_version: 1,
            name: "RCS exhaustion".into(),
            parts: vec![
                child_part(1, "mono_tank", None, [0.0, 0.0, 0.0]),
                child_part(2, "rcs", Some(1), [0.0, 0.0, 0.0]),
            ],
            stages: Vec::new(),
            crew: Vec::new(),
            script_name: None,
        };
        let mut vessel = Vessel::from_blueprint(&craft, &catalog);
        vessel.position = [
            crate::model::HOME_RADIUS + HOME_ATMOSPHERE + 1_000.0,
            0.0,
            0.0,
        ];
        vessel.velocity = [0.0; 3];
        vessel.controls.sas = None;
        vessel.controls.rcs = true;
        let initial = vessel.parts[0].fuel;
        step_vessel(&mut vessel, &catalog, 1.0, 0.0);
        assert_abs_diff_eq!(vessel.parts[0].fuel, initial, epsilon = 1e-12);

        let mut exhausted = Vessel::from_blueprint(&craft, &catalog);
        exhausted.position = [
            crate::model::HOME_RADIUS + HOME_ATMOSPHERE + 1_000.0,
            0.0,
            0.0,
        ];
        exhausted.velocity = [0.0; 3];
        exhausted.controls.sas = None;
        exhausted.controls.rcs = true;
        exhausted.controls.pitch = 1.0;
        exhausted.parts[0].fuel = 0.01;
        step_vessel(&mut exhausted, &catalog, 1.0, 1.0);
        assert_eq!(exhausted.parts[0].fuel, 0.0);
        step_vessel(&mut exhausted, &catalog, 1.0, 2.0);
        assert_eq!(exhausted.parts[0].fuel, 0.0);
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
    fn swept_soi_detection_finds_a_grazing_encounter_between_warp_samples() {
        let home = body_definition("carapace");
        let moon = body_definition("selene");
        let dt = SimulationClock::WARP_RATES[7] / 60.0;
        let speed = 3_000.0;
        let (moon_position, moon_velocity) = circular_ephemeris(&moon, home.mu, 0.0);
        let capture_radius =
            sphere_of_influence(moon.semi_major_axis, moon.mu, home.mu) * SOI_CAPTURE_FACTOR;
        let impact_offset = moon_position.normalize() * capture_radius * 0.9;
        let relative_position = impact_offset - DVec3::Z * speed * dt * 0.5;
        let position = moon_position + relative_position;
        let velocity = moon_velocity + DVec3::Z * speed;

        let (end_position, uninterrupted_velocity) =
            propagate_universal(position, velocity, home.mu, dt);
        let (end_moon_position, _) = circular_ephemeris(&moon, home.mu, dt);
        assert!(relative_position.length() > capture_radius);
        assert!((end_position - end_moon_position).length() > capture_radius);

        let encounter =
            child_soi_encounter_time(position, velocity, &home, &moon, 0.0, dt).unwrap();
        assert!(encounter > 0.0);
        assert!(encounter < dt);

        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        vessel.position = position.to_array();
        vessel.velocity = velocity.to_array();
        vessel.situation = FlightSituation::Orbiting;
        step_on_rails_patched(&mut vessel, 0.0, dt);

        assert_eq!(vessel.primary_body, "carapace");
        assert!((vessel.velocity_vec() - uninterrupted_velocity).length() > 1.0);
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

    #[test]
    fn peak_heating_is_retained_while_telemetry_reports_current_flux() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        vessel.position = [0.0, crate::model::HOME_RADIUS + 30_000.0, 0.0];
        vessel.velocity = [2_400.0, 0.0, 0.0];
        let hot = step_vessel(&mut vessel, &catalog, 0.1, 0.0);
        let peak = vessel.max_heating;
        assert!(peak > 0.0);
        assert!(hot.heating > 0.0);

        vessel.position = [
            0.0,
            crate::model::HOME_RADIUS + HOME_ATMOSPHERE + 1_000.0,
            0.0,
        ];
        vessel.velocity = [0.0; 3];
        let cold = step_vessel(&mut vessel, &catalog, 0.1, 0.1);

        assert_eq!(cold.heating, 0.0);
        assert_eq!(vessel.max_heating, peak);
    }

    #[test]
    fn projected_drag_distinguishes_nose_first_and_broadside() {
        let catalog = PartCatalog::default();
        let craft = CraftBlueprint {
            schema_version: 1,
            name: "Drag test".into(),
            parts: vec![part(1, "tank_long", [0.0, 0.0, 0.0])],
            stages: Vec::new(),
            crew: Vec::new(),
            script_name: None,
        };
        let vessel = Vessel::from_blueprint(&craft, &catalog);
        let tank = catalog.get("tank_long").unwrap();
        let runtime = &vessel.parts[0];
        let nose_first =
            projected_drag_area(tank, runtime, &vessel, &catalog, DQuat::IDENTITY, DVec3::Y);
        let broadside =
            projected_drag_area(tank, runtime, &vessel, &catalog, DQuat::IDENTITY, DVec3::X);

        assert!(broadside > nose_first * 2.0);
    }

    #[test]
    fn inline_drag_shields_only_the_covered_axial_face() {
        let catalog = PartCatalog::default();
        let craft = CraftBlueprint {
            schema_version: 1,
            name: "Inline shielding".into(),
            parts: vec![
                child_part(1, "tank_long", None, [0.0, 0.0, 0.0]),
                child_part(2, "tank_short", Some(1), [0.0, 4.2, 0.0]),
                child_part(3, "rcs", Some(1), [1.5, 0.0, 0.0]),
            ],
            stages: Vec::new(),
            crew: Vec::new(),
            script_name: None,
        };
        let mut vessel = Vessel::from_blueprint(&craft, &catalog);
        let tank_definition = catalog.get("tank_long").unwrap();
        let rcs_definition = catalog.get("rcs").unwrap();
        let isolated = CraftBlueprint {
            parts: vec![child_part(1, "tank_long", None, [0.0, 0.0, 0.0])],
            ..craft.clone()
        };
        let isolated_vessel = Vessel::from_blueprint(&isolated, &catalog);

        let isolated_axial = projected_drag_area(
            tank_definition,
            &isolated_vessel.parts[0],
            &isolated_vessel,
            &catalog,
            DQuat::IDENTITY,
            DVec3::Y,
        );
        let shielded_axial = projected_drag_area(
            tank_definition,
            &vessel.parts[0],
            &vessel,
            &catalog,
            DQuat::IDENTITY,
            DVec3::Y,
        );
        assert!(shielded_axial < isolated_axial * 1e-9);

        let isolated_broadside = projected_drag_area(
            tank_definition,
            &isolated_vessel.parts[0],
            &isolated_vessel,
            &catalog,
            DQuat::IDENTITY,
            DVec3::X,
        );
        let shielded_broadside = projected_drag_area(
            tank_definition,
            &vessel.parts[0],
            &vessel,
            &catalog,
            DQuat::IDENTITY,
            DVec3::X,
        );
        assert_abs_diff_eq!(isolated_broadside, shielded_broadside, epsilon = 1e-12);

        let radial_before = projected_drag_area(
            rcs_definition,
            &vessel.parts[2],
            &vessel,
            &catalog,
            DQuat::IDENTITY,
            DVec3::Y,
        );
        vessel.parts[1].destroyed = true;
        let reexposed_axial = projected_drag_area(
            tank_definition,
            &vessel.parts[0],
            &vessel,
            &catalog,
            DQuat::IDENTITY,
            DVec3::Y,
        );
        let radial_after = projected_drag_area(
            rcs_definition,
            &vessel.parts[2],
            &vessel,
            &catalog,
            DQuat::IDENTITY,
            DVec3::Y,
        );
        assert_abs_diff_eq!(reexposed_axial, isolated_axial, epsilon = 1e-12);
        assert_abs_diff_eq!(radial_before, radial_after, epsilon = 1e-12);
    }

    #[test]
    fn mach_uses_the_local_atmosphere_and_is_zero_in_vacuum() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        let body = body_definition("carapace");
        let position = DVec3::Y * (body.radius + 5.0);
        vessel.position = position.to_array();
        vessel.velocity =
            (ground_velocity(position, body.rotation_period) + DVec3::X * 343.0).to_array();
        let sea_level = telemetry(&vessel, &catalog, 0.0, 0.0);
        assert_abs_diff_eq!(sea_level.mach, 1.0, epsilon = 0.01);

        vessel.position = (DVec3::Y * (body.radius + HOME_ATMOSPHERE + 1.0)).to_array();
        let vacuum = telemetry(&vessel, &catalog, 0.0, 0.0);
        assert_eq!(vacuum.mach, 0.0);
    }

    #[test]
    fn touchdown_depends_on_attitude_and_landing_legs() {
        let catalog = PartCatalog::default();
        let pod_craft = CraftBlueprint {
            schema_version: 1,
            name: "Bare pod".into(),
            parts: vec![part(1, "pod_1", [0.0, 0.0, 0.0])],
            stages: Vec::new(),
            crew: Vec::new(),
            script_name: None,
        };
        let bare = Vessel::from_blueprint(&pod_craft, &catalog);
        let radial = DVec3::Y;
        let impact = -radial * 12.0;
        assert!(touchdown_is_survivable(
            &bare,
            &catalog,
            DQuat::IDENTITY,
            radial,
            impact,
        ));
        assert!(!touchdown_is_survivable(
            &bare,
            &catalog,
            DQuat::from_rotation_z(PI * 0.5),
            radial,
            impact,
        ));

        let legged_craft = CraftBlueprint {
            parts: vec![
                part(1, "pod_1", [0.0, 0.0, 0.0]),
                part(2, "landing_leg", [1.0, -1.0, 0.0]),
                part(3, "landing_leg", [-1.0, -1.0, 0.0]),
                part(4, "landing_leg", [0.0, -1.0, 1.0]),
            ],
            name: "Legged pod".into(),
            ..pod_craft
        };
        let legged = Vessel::from_blueprint(&legged_craft, &catalog);
        let fast_impact = -radial * 22.0;
        assert!(!touchdown_is_survivable(
            &bare,
            &catalog,
            DQuat::IDENTITY,
            radial,
            fast_impact,
        ));
        assert!(touchdown_is_survivable(
            &legged,
            &catalog,
            DQuat::IDENTITY,
            radial,
            fast_impact,
        ));
    }
}
