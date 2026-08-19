use bevy::math::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;

pub const HOME_RADIUS: f64 = 600_000.0;
pub const HOME_MU: f64 = 3.532e12;
pub const HOME_ATMOSPHERE: f64 = 70_000.0;
pub const MAX_PARTS: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PartCategory {
    Command,
    Propulsion,
    Fuel,
    Coupling,
    Control,
    Aero,
    Utility,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum PartModule {
    Command {
        seats: u8,
        torque: f64,
    },
    LiquidEngine {
        thrust_vac: f64,
        thrust_sl: f64,
        isp_vac: f64,
        isp_sl: f64,
        gimbal_deg: f64,
    },
    SolidEngine {
        thrust: f64,
        isp: f64,
        fuel: f64,
    },
    LiquidTank {
        fuel: f64,
    },
    MonopropTank {
        fuel: f64,
    },
    InlineDecoupler {
        impulse: f64,
    },
    RadialDecoupler {
        impulse: f64,
    },
    ReactionWheel {
        torque: f64,
    },
    Rcs {
        thrust: f64,
        isp: f64,
    },
    Fin {
        lift: f64,
        steerable: bool,
    },
    Parachute {
        drag_area: f64,
        safe_speed: f64,
    },
    HeatShield {
        ablator: f64,
    },
    LandingLeg,
    Structural,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PartDefinition {
    pub id: String,
    pub name: String,
    pub category: PartCategory,
    pub module: PartModule,
    pub dry_mass: f64,
    pub radius: f32,
    pub height: f32,
    pub drag_coefficient: f64,
    pub max_temperature: f64,
    pub description: String,
    pub radial: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum StageAction {
    ActivateEngine(u64),
    Decouple(u64),
    DeployParachute(u64),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Stage {
    pub name: String,
    pub actions: Vec<StageAction>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PartInstance {
    pub instance_id: u64,
    pub definition_id: String,
    pub parent: Option<u64>,
    pub local_position: [f32; 3],
    pub local_rotation: [f32; 4],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CraftBlueprint {
    pub schema_version: u32,
    pub name: String,
    pub parts: Vec<PartInstance>,
    pub stages: Vec<Stage>,
    pub crew: Vec<String>,
    pub script_name: Option<String>,
}

impl CraftBlueprint {
    pub fn root(&self) -> Option<u64> {
        self.parts
            .iter()
            .find(|part| part.parent.is_none())
            .map(|part| part.instance_id)
    }

    pub fn next_id(&self) -> u64 {
        self.parts
            .iter()
            .map(|part| part.instance_id)
            .max()
            .unwrap_or(0)
            + 1
    }

    pub fn children_of(&self, id: u64) -> impl Iterator<Item = &PartInstance> {
        self.parts
            .iter()
            .filter(move |part| part.parent == Some(id))
    }

    pub fn validate(&self, catalog: &PartCatalog) -> Vec<ValidationIssue> {
        let mut issues = Vec::new();
        if self.parts.is_empty() {
            issues.push(ValidationIssue::Error("The vehicle has no parts".into()));
            return issues;
        }
        if self.parts.len() > MAX_PARTS {
            issues.push(ValidationIssue::Error(format!(
                "Vehicle exceeds the {MAX_PARTS}-part limit"
            )));
        }
        if self
            .parts
            .iter()
            .filter(|part| part.parent.is_none())
            .count()
            != 1
        {
            issues.push(ValidationIssue::Error(
                "Vehicle must have exactly one root part".into(),
            ));
        }
        let ids: BTreeSet<_> = self.parts.iter().map(|part| part.instance_id).collect();
        for part in &self.parts {
            if catalog.get(&part.definition_id).is_none() {
                issues.push(ValidationIssue::Error(format!(
                    "Unknown part {}",
                    part.definition_id
                )));
            }
            if let Some(parent) = part.parent
                && !ids.contains(&parent)
            {
                issues.push(ValidationIssue::Error(format!(
                    "Part {} has a missing parent",
                    part.instance_id
                )));
            }
        }
        let has_command = self.parts.iter().any(|part| {
            catalog
                .get(&part.definition_id)
                .is_some_and(|def| matches!(def.module, PartModule::Command { .. }))
        });
        if !has_command {
            issues.push(ValidationIssue::Error(
                "Add a command pod before launch".into(),
            ));
        }
        let has_engine = self.parts.iter().any(|part| {
            catalog.get(&part.definition_id).is_some_and(|def| {
                matches!(
                    def.module,
                    PartModule::LiquidEngine { .. } | PartModule::SolidEngine { .. }
                )
            })
        });
        if !has_engine {
            issues.push(ValidationIssue::Warning("This craft has no engine".into()));
        }
        let has_chute = self.parts.iter().any(|part| {
            catalog
                .get(&part.definition_id)
                .is_some_and(|def| matches!(def.module, PartModule::Parachute { .. }))
        });
        if !has_chute {
            issues.push(ValidationIssue::Warning("No parachute is installed".into()));
        }
        issues
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ValidationIssue {
    Error(String),
    Warning(String),
}

#[derive(Debug, Clone)]
pub struct PartCatalog {
    definitions: BTreeMap<String, PartDefinition>,
}

impl Default for PartCatalog {
    fn default() -> Self {
        let mut definitions = BTreeMap::new();
        let mut add = |id: &str,
                       name: &str,
                       category,
                       module,
                       dry_mass,
                       radius,
                       height,
                       cd,
                       max_temp,
                       radial,
                       description: &str| {
            definitions.insert(
                id.into(),
                PartDefinition {
                    id: id.into(),
                    name: name.into(),
                    category,
                    module,
                    dry_mass,
                    radius,
                    height,
                    drag_coefficient: cd,
                    max_temperature: max_temp,
                    description: description.into(),
                    radial,
                },
            );
        };

        add(
            "pod_1",
            "Carapace Mk I",
            PartCategory::Command,
            PartModule::Command {
                seats: 1,
                torque: 8_000.0,
            },
            900.0,
            1.15,
            1.8,
            0.28,
            1_600.0,
            false,
            "A compact command pod for one intrepid crab.",
        );
        add(
            "pod_2",
            "Carapace Mk II",
            PartCategory::Command,
            PartModule::Command {
                seats: 2,
                torque: 12_000.0,
            },
            1_500.0,
            1.45,
            2.1,
            0.3,
            1_600.0,
            false,
            "A roomier pod with two acceleration couches and four cup holders.",
        );
        add(
            "engine_sl_s",
            "Littoral L-20",
            PartCategory::Propulsion,
            PartModule::LiquidEngine {
                thrust_vac: 240_000.0,
                thrust_sl: 205_000.0,
                isp_vac: 320.0,
                isp_sl: 285.0,
                gimbal_deg: 5.0,
            },
            650.0,
            0.85,
            1.2,
            0.35,
            2_100.0,
            false,
            "A dependable gimballed launch engine.",
        );
        add(
            "engine_sl_l",
            "Trench L-60",
            PartCategory::Propulsion,
            PartModule::LiquidEngine {
                thrust_vac: 720_000.0,
                thrust_sl: 630_000.0,
                isp_vac: 315.0,
                isp_sl: 280.0,
                gimbal_deg: 3.0,
            },
            1_450.0,
            1.25,
            1.7,
            0.4,
            2_200.0,
            false,
            "High thrust for heavy first stages.",
        );
        add(
            "engine_vac",
            "Abyssal V-9",
            PartCategory::Propulsion,
            PartModule::LiquidEngine {
                thrust_vac: 110_000.0,
                thrust_sl: 55_000.0,
                isp_vac: 365.0,
                isp_sl: 190.0,
                gimbal_deg: 4.0,
            },
            360.0,
            0.75,
            1.15,
            0.32,
            2_050.0,
            false,
            "An efficient vacuum engine with a wide bell.",
        );
        add(
            "solid_stack",
            "Barnacle S-12",
            PartCategory::Propulsion,
            PartModule::SolidEngine {
                thrust: 55_000.0,
                isp: 245.0,
                fuel: 1_700.0,
            },
            300.0,
            0.75,
            3.8,
            0.42,
            2_000.0,
            false,
            "A compact stack-mounted solid motor.",
        );
        add(
            "solid_radial",
            "Breakwater S-30",
            PartCategory::Propulsion,
            PartModule::SolidEngine {
                thrust: 110_000.0,
                isp: 235.0,
                fuel: 3_300.0,
            },
            500.0,
            0.85,
            6.0,
            0.46,
            2_000.0,
            true,
            "A muscular radial booster. Once lit, it stays lit.",
        );
        add(
            "tank_short",
            "Tidepool Tank",
            PartCategory::Fuel,
            PartModule::LiquidTank { fuel: 2_400.0 },
            280.0,
            1.0,
            2.0,
            0.3,
            1_500.0,
            false,
            "A short liquid propellant tank.",
        );
        add(
            "tank_long",
            "Bluewater Tank",
            PartCategory::Fuel,
            PartModule::LiquidTank { fuel: 6_200.0 },
            620.0,
            1.0,
            4.2,
            0.3,
            1_500.0,
            false,
            "A long liquid propellant tank.",
        );
        add(
            "mono_tank",
            "Current Monoprop Tank",
            PartCategory::Fuel,
            PartModule::MonopropTank { fuel: 220.0 },
            90.0,
            0.7,
            0.8,
            0.3,
            1_400.0,
            false,
            "Feeds RCS thrusters anywhere on the vessel.",
        );
        add(
            "decoupler_s",
            "Seam 1.25",
            PartCategory::Coupling,
            PartModule::InlineDecoupler { impulse: 1_500.0 },
            65.0,
            1.0,
            0.25,
            0.35,
            1_600.0,
            false,
            "An in-line stage separator.",
        );
        add(
            "decoupler_l",
            "Seam 2.5",
            PartCategory::Coupling,
            PartModule::InlineDecoupler { impulse: 2_500.0 },
            120.0,
            1.3,
            0.3,
            0.35,
            1_600.0,
            false,
            "A larger in-line stage separator.",
        );
        add(
            "decoupler_radial",
            "Side-Shedder",
            PartCategory::Coupling,
            PartModule::RadialDecoupler { impulse: 2_000.0 },
            75.0,
            0.25,
            1.0,
            0.4,
            1_600.0,
            true,
            "Releases a radially attached booster with clearance impulse.",
        );
        add(
            "reaction_wheel",
            "Gyroclaw",
            PartCategory::Control,
            PartModule::ReactionWheel { torque: 16_000.0 },
            140.0,
            0.9,
            0.4,
            0.3,
            1_500.0,
            false,
            "Electric attitude control without propellant bookkeeping.",
        );
        add(
            "rcs",
            "Four-Pincer RCS",
            PartCategory::Control,
            PartModule::Rcs {
                thrust: 2_000.0,
                isp: 240.0,
            },
            35.0,
            0.18,
            0.4,
            0.5,
            1_700.0,
            true,
            "Four-way monopropellant attitude thruster.",
        );
        add(
            "fin",
            "Keel Fin",
            PartCategory::Aero,
            PartModule::Fin {
                lift: 7.0,
                steerable: false,
            },
            45.0,
            0.65,
            1.2,
            0.2,
            1_400.0,
            true,
            "A fixed aerodynamic stabilizer.",
        );
        add(
            "fin_control",
            "Rudderclaw",
            PartCategory::Aero,
            PartModule::Fin {
                lift: 8.0,
                steerable: true,
            },
            60.0,
            0.7,
            1.3,
            0.22,
            1_400.0,
            true,
            "A steerable fin for atmospheric control.",
        );
        add(
            "nose",
            "Low-Tide Nose Cone",
            PartCategory::Aero,
            PartModule::Structural,
            55.0,
            0.85,
            1.2,
            0.12,
            1_700.0,
            false,
            "Reduces drag on exposed stacks.",
        );
        add(
            "heatshield",
            "Ablative Shell",
            PartCategory::Utility,
            PartModule::HeatShield { ablator: 240.0 },
            220.0,
            1.2,
            0.25,
            0.55,
            3_400.0,
            false,
            "Sacrificial protection for atmospheric return.",
        );
        add(
            "drogue",
            "Reefed Drogue",
            PartCategory::Utility,
            PartModule::Parachute {
                drag_area: 45.0,
                safe_speed: 700.0,
            },
            55.0,
            0.35,
            0.35,
            0.4,
            1_300.0,
            false,
            "A high-speed stabilizing parachute.",
        );
        add(
            "parachute",
            "Main Canopy",
            PartCategory::Utility,
            PartModule::Parachute {
                drag_area: 260.0,
                safe_speed: 280.0,
            },
            90.0,
            0.45,
            0.4,
            0.4,
            1_300.0,
            false,
            "A large recovery canopy.",
        );
        add(
            "landing_leg",
            "Shoreline Leg",
            PartCategory::Utility,
            PartModule::LandingLeg,
            70.0,
            0.18,
            1.5,
            0.5,
            1_500.0,
            true,
            "A compact shock-absorbing landing leg.",
        );
        Self { definitions }
    }
}

impl PartCatalog {
    pub fn get(&self, id: &str) -> Option<&PartDefinition> {
        self.definitions.get(id)
    }

    pub fn iter(&self) -> impl Iterator<Item = &PartDefinition> {
        self.definitions.values()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SasMode {
    Stability,
    Prograde,
    Retrograde,
    Normal,
    AntiNormal,
    RadialIn,
    RadialOut,
    Maneuver,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct FlightControls {
    pub throttle: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
    pub sas: Option<SasMode>,
    pub rcs: bool,
}

impl Default for FlightControls {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
            sas: Some(SasMode::Stability),
            rcs: false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RuntimePart {
    pub instance: PartInstance,
    pub fuel: f64,
    pub ablator: f64,
    pub temperature: f64,
    pub active: bool,
    pub destroyed: bool,
    pub parachute_deployed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetachedStage {
    pub primary_body: String,
    pub parts: Vec<RuntimePart>,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub attitude: [f64; 4],
    pub angular_velocity: [f64; 3],
}

impl DetachedStage {
    pub fn position_vec(&self) -> DVec3 {
        DVec3::from_array(self.position)
    }

    pub fn velocity_vec(&self) -> DVec3 {
        DVec3::from_array(self.velocity)
    }

    pub fn attitude_quat(&self) -> DQuat {
        DQuat::from_xyzw(
            self.attitude[0],
            self.attitude[1],
            self.attitude[2],
            self.attitude[3],
        )
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManeuverNode {
    pub ut: f64,
    pub prograde: f64,
    pub normal: f64,
    pub radial: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FlightSituation {
    Prelaunch,
    Flying,
    Orbiting,
    Landed,
    Crashed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vessel {
    pub name: String,
    pub primary_body: String,
    pub parts: Vec<RuntimePart>,
    pub stages: Vec<Stage>,
    pub next_stage: usize,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub attitude: [f64; 4],
    pub angular_velocity: [f64; 3],
    pub controls: FlightControls,
    #[serde(default)]
    pub sas_target_attitude: Option<[f64; 4]>,
    pub situation: FlightSituation,
    pub max_heating: f64,
    pub maneuver: Option<ManeuverNode>,
    pub crew: Vec<String>,
    #[serde(default)]
    pub debris: Vec<DetachedStage>,
}

impl Vessel {
    pub fn from_blueprint(blueprint: &CraftBlueprint, catalog: &PartCatalog) -> Self {
        let parts = blueprint
            .parts
            .iter()
            .map(|instance| {
                let def = catalog
                    .get(&instance.definition_id)
                    .expect("validated part definition");
                let (fuel, ablator) = match def.module {
                    PartModule::LiquidTank { fuel } | PartModule::MonopropTank { fuel } => {
                        (fuel, 0.0)
                    }
                    PartModule::SolidEngine { fuel, .. } => (fuel, 0.0),
                    PartModule::HeatShield { ablator } => (0.0, ablator),
                    _ => (0.0, 0.0),
                };
                RuntimePart {
                    instance: instance.clone(),
                    fuel,
                    ablator,
                    temperature: 290.0,
                    active: false,
                    destroyed: false,
                    parachute_deployed: false,
                }
            })
            .collect();
        Self {
            name: blueprint.name.clone(),
            primary_body: "carapace".into(),
            parts,
            stages: blueprint.stages.clone(),
            next_stage: 0,
            position: [0.0, HOME_RADIUS + 5.0, 0.0],
            velocity: [-TAU / 21_600.0 * (HOME_RADIUS + 5.0), 0.0, 0.0],
            attitude: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0; 3],
            controls: FlightControls::default(),
            sas_target_attitude: None,
            situation: FlightSituation::Prelaunch,
            max_heating: 0.0,
            maneuver: None,
            crew: blueprint.crew.clone(),
            debris: Vec::new(),
        }
    }

    pub fn position_vec(&self) -> DVec3 {
        DVec3::from_array(self.position)
    }
    pub fn velocity_vec(&self) -> DVec3 {
        DVec3::from_array(self.velocity)
    }
    pub fn attitude_quat(&self) -> DQuat {
        DQuat::from_xyzw(
            self.attitude[0],
            self.attitude[1],
            self.attitude[2],
            self.attitude[3],
        )
    }
    pub fn angular_velocity_vec(&self) -> DVec3 {
        DVec3::from_array(self.angular_velocity)
    }

    pub fn active_ids(&self) -> BTreeSet<u64> {
        self.parts
            .iter()
            .filter(|part| !part.destroyed)
            .map(|part| part.instance.instance_id)
            .collect()
    }
}

pub fn stock_craft() -> CraftBlueprint {
    let part = |id, definition_id: &str, parent, x, y, z| PartInstance {
        instance_id: id,
        definition_id: definition_id.into(),
        parent,
        local_position: [x, y, z],
        local_rotation: [0.0, 0.0, 0.0, 1.0],
    };
    let parts = vec![
        part(1, "pod_1", None, 0.0, 9.4, 0.0),
        part(2, "parachute", Some(1), 0.0, 10.55, 0.0),
        part(3, "heatshield", Some(1), 0.0, 8.35, 0.0),
        part(4, "tank_short", Some(3), 0.0, 7.2, 0.0),
        part(5, "reaction_wheel", Some(4), 0.0, 5.95, 0.0),
        part(6, "engine_vac", Some(5), 0.0, 5.1, 0.0),
        part(7, "decoupler_s", Some(6), 0.0, 4.35, 0.0),
        part(8, "tank_long", Some(7), 0.0, 2.1, 0.0),
        part(9, "engine_sl_s", Some(8), 0.0, -0.60, 0.0),
        part(10, "decoupler_radial", Some(8), 1.35, 2.0, 0.0),
        part(11, "solid_radial", Some(10), 2.25, 1.0, 0.0),
        part(12, "decoupler_radial", Some(8), -1.35, 2.0, 0.0),
        part(13, "solid_radial", Some(12), -2.25, 1.0, 0.0),
        part(14, "fin_control", Some(8), 1.3, 0.2, 0.0),
        part(15, "fin_control", Some(8), -1.3, 0.2, 0.0),
    ];
    CraftBlueprint {
        schema_version: 1,
        name: "Crabitat Pathfinder".into(),
        parts,
        stages: vec![
            Stage {
                name: "Ignition".into(),
                actions: vec![
                    StageAction::ActivateEngine(9),
                    StageAction::ActivateEngine(11),
                    StageAction::ActivateEngine(13),
                ],
            },
            Stage {
                name: "Shed boosters".into(),
                actions: vec![StageAction::Decouple(10), StageAction::Decouple(12)],
            },
            Stage {
                name: "Upper stage".into(),
                actions: vec![StageAction::Decouple(7), StageAction::ActivateEngine(6)],
            },
            Stage {
                name: "Recovery".into(),
                actions: vec![StageAction::DeployParachute(2)],
            },
        ],
        crew: vec!["Dr. Clawdia Current".into()],
        script_name: Some("guided_ascent.lua".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stock_craft_is_valid() {
        let issues = stock_craft().validate(&PartCatalog::default());
        assert!(
            !issues
                .iter()
                .any(|issue| matches!(issue, ValidationIssue::Error(_))),
            "{issues:?}"
        );
    }
}
