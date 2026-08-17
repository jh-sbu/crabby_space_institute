use std::collections::VecDeque;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex, RwLock};
use std::time::Duration;

use axum::Router;
use bevy::prelude::*;
use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{CallToolResult, ContentBlock, Implementation, ServerCapabilities, ServerInfo};
use rmcp::transport::streamable_http_server::{
    StreamableHttpServerConfig, StreamableHttpService, session::local::LocalSessionManager,
};
use rmcp::{ServerHandler, schemars, tool, tool_handler, tool_router};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value, json};
use tokio::sync::oneshot;

use super::{
    AppMode, Catalog, EditorAction, EditorState, RemoteFlightInput, Session, Store, ViewState,
    activate_stage, apply_editor_action, continue_quicksave, enter_vehicle_assembly,
    load_quicksave, return_to_assembly, save_quicksave,
};
use crate::model::{CraftBlueprint, ManeuverNode, SasMode, ValidationIssue, Vessel};
use crate::orbit::celestial_system;
use crate::scripting::{COROUTINE_EXAMPLE, EXAMPLE_SCRIPT, ScriptMode, ScriptRuntime};
use crate::simulation::{MissionProgress, SimulationClock, telemetry};

const DEFAULT_MCP_ADDR: &str = "127.0.0.1:8765";
const REQUEST_TIMEOUT: Duration = Duration::from_secs(3);

pub(super) struct GameMcpPlugin;

impl Plugin for GameMcpPlugin {
    fn build(&self, app: &mut App) {
        let bridge = GameStateBridge::default();
        app.insert_resource(bridge.clone())
            .init_resource::<PendingReplies>()
            .add_systems(Startup, start_mcp_server)
            .add_systems(PreUpdate, process_mcp_requests)
            .add_systems(Last, publish_game_state);
    }
}

#[derive(Clone, Resource)]
struct GameStateBridge {
    game_snapshot: Arc<RwLock<Value>>,
    player_snapshot: Arc<RwLock<Value>>,
    requests: Arc<Mutex<VecDeque<McpRequest>>>,
}

impl Default for GameStateBridge {
    fn default() -> Self {
        let starting = json!({
            "status": "starting",
            "message": "The game has not completed its first update yet"
        });
        Self {
            game_snapshot: Arc::new(RwLock::new(starting.clone())),
            player_snapshot: Arc::new(RwLock::new(starting)),
            requests: Arc::default(),
        }
    }
}

enum McpRequest {
    Patch {
        patch: Map<String, Value>,
        reply: oneshot::Sender<Result<Value, String>>,
    },
    Action {
        name: String,
        action: GameAction,
        reply: oneshot::Sender<Result<Value, String>>,
    },
}

enum GameAction {
    Menu(MenuActionParams),
    Assembly(AssemblyActionParams),
    SetFlightControls(SetFlightControlsParams),
    Flight(FlightActionParams),
    Script(ScriptActionParams),
    View(ViewActionParams),
}

enum ReplyKind {
    Patch,
    Action(String),
}

struct PendingReply {
    kind: ReplyKind,
    result: Result<(), String>,
    reply: oneshot::Sender<Result<Value, String>>,
}

#[derive(Resource, Default)]
struct PendingReplies(VecDeque<PendingReply>);

#[derive(Debug, Clone, Serialize, Deserialize)]
struct DebugGameState {
    mode: AppMode,
    clock: SimulationClock,
    craft: CraftBlueprint,
    vessel: Option<Vessel>,
    mission: MissionProgress,
    notice: String,
}

impl DebugGameState {
    fn capture(mode: AppMode, session: &Session, clock: &SimulationClock) -> Self {
        Self {
            mode,
            clock: clock.clone(),
            craft: session.craft.clone(),
            vessel: session.vessel.clone(),
            mission: session.mission.clone(),
            notice: session.notice.clone(),
        }
    }

    fn validate(&self) -> Result<(), String> {
        if !self.clock.universal_time.is_finite() || self.clock.universal_time < 0.0 {
            return Err("clock.universal_time must be a finite, non-negative number".into());
        }
        if self.clock.warp_index >= SimulationClock::WARP_RATES.len() {
            return Err(format!(
                "clock.warp_index must be between 0 and {}",
                SimulationClock::WARP_RATES.len() - 1
            ));
        }

        let Some(vessel) = &self.vessel else {
            return Ok(());
        };
        if !celestial_system()
            .iter()
            .any(|body| body.id == vessel.primary_body)
        {
            return Err(format!(
                "vessel.primary_body is unknown: {}",
                vessel.primary_body
            ));
        }
        if vessel.next_stage > vessel.stages.len() {
            return Err("vessel.next_stage cannot exceed vessel.stages.length".into());
        }
        for (name, value) in [
            ("throttle", vessel.controls.throttle),
            ("pitch", vessel.controls.pitch),
            ("yaw", vessel.controls.yaw),
            ("roll", vessel.controls.roll),
        ] {
            let range = if name == "throttle" {
                0.0..=1.0
            } else {
                -1.0..=1.0
            };
            if !range.contains(&value) {
                return Err(format!("vessel.controls.{name} is outside its valid range"));
            }
        }
        let attitude_length_squared: f64 = vessel.attitude.iter().map(|value| value * value).sum();
        if attitude_length_squared <= f64::EPSILON {
            return Err("vessel.attitude must be a non-zero quaternion".into());
        }
        Ok(())
    }
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct PatchGameStateParams {
    /// RFC 7396 JSON Merge Patch applied to the state returned by inspect_game_state.
    patch: Map<String, Value>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
#[serde(tag = "action", rename_all = "snake_case")]
enum MenuActionParams {
    OpenVehicleAssembly,
    ContinueQuicksave,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct MenuActionRequest {
    #[serde(flatten)]
    action: MenuActionParams,
}

impl MenuActionParams {
    fn name(&self) -> &'static str {
        match self {
            Self::OpenVehicleAssembly => "menu.open_vehicle_assembly",
            Self::ContinueQuicksave => "menu.continue_quicksave",
        }
    }
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
#[serde(tag = "action", rename_all = "snake_case")]
enum AssemblyActionParams {
    RenameCraft { name: String },
    NewCraft,
    Undo,
    Redo,
    SelectPart { instance_id: u64 },
    SetSymmetry { count: usize },
    AddPart { definition_id: String },
    RemoveSelectedSubtree,
    SaveCraft,
    LoadCraft { name: String },
    Launch,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct AssemblyActionRequest {
    #[serde(flatten)]
    action: AssemblyActionParams,
}

impl AssemblyActionParams {
    fn name(&self) -> &'static str {
        match self {
            Self::RenameCraft { .. } => "assembly.rename_craft",
            Self::NewCraft => "assembly.new_craft",
            Self::Undo => "assembly.undo",
            Self::Redo => "assembly.redo",
            Self::SelectPart { .. } => "assembly.select_part",
            Self::SetSymmetry { .. } => "assembly.set_symmetry",
            Self::AddPart { .. } => "assembly.add_part",
            Self::RemoveSelectedSubtree => "assembly.remove_selected_subtree",
            Self::SaveCraft => "assembly.save_craft",
            Self::LoadCraft { .. } => "assembly.load_craft",
            Self::Launch => "assembly.launch",
        }
    }
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct SetFlightControlsParams {
    /// Set and latch throttle in the range 0..=1. Omit to leave it unchanged.
    throttle: Option<f64>,
    /// Latch pitch in the range -1..=1. Omit to leave the current MCP override unchanged.
    pitch: Option<f64>,
    /// Latch yaw in the range -1..=1. Omit to leave the current MCP override unchanged.
    yaw: Option<f64>,
    /// Latch roll in the range -1..=1. Omit to leave the current MCP override unchanged.
    roll: Option<f64>,
}

#[derive(Debug, Clone, Copy, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
enum SasSelection {
    Off,
    Stability,
    Prograde,
    Retrograde,
    Normal,
    AntiNormal,
    RadialIn,
    RadialOut,
    Maneuver,
}

impl SasSelection {
    fn value(self) -> Option<SasMode> {
        match self {
            Self::Off => None,
            Self::Stability => Some(SasMode::Stability),
            Self::Prograde => Some(SasMode::Prograde),
            Self::Retrograde => Some(SasMode::Retrograde),
            Self::Normal => Some(SasMode::Normal),
            Self::AntiNormal => Some(SasMode::AntiNormal),
            Self::RadialIn => Some(SasMode::RadialIn),
            Self::RadialOut => Some(SasMode::RadialOut),
            Self::Maneuver => Some(SasMode::Maneuver),
        }
    }
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
#[serde(tag = "action", rename_all = "snake_case")]
enum FlightActionParams {
    ReturnToAssembly,
    ActivateNextStage,
    ReleaseAttitudeControls,
    SetSas {
        mode: SasSelection,
    },
    SetRcs {
        enabled: bool,
    },
    IncreaseWarp,
    DecreaseWarp,
    SetPaused {
        paused: bool,
    },
    Quicksave,
    Quickload,
    AddManeuverNode,
    SetManeuverNode {
        ut: f64,
        prograde: f64,
        normal: f64,
        radial: f64,
    },
    RemoveManeuverNode,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct FlightActionRequest {
    #[serde(flatten)]
    action: FlightActionParams,
}

impl FlightActionParams {
    fn name(&self) -> &'static str {
        match self {
            Self::ReturnToAssembly => "flight.return_to_assembly",
            Self::ActivateNextStage => "flight.activate_next_stage",
            Self::ReleaseAttitudeControls => "flight.release_attitude_controls",
            Self::SetSas { .. } => "flight.set_sas",
            Self::SetRcs { .. } => "flight.set_rcs",
            Self::IncreaseWarp => "flight.increase_warp",
            Self::DecreaseWarp => "flight.decrease_warp",
            Self::SetPaused { .. } => "flight.set_paused",
            Self::Quicksave => "flight.quicksave",
            Self::Quickload => "flight.quickload",
            Self::AddManeuverNode => "flight.add_maneuver_node",
            Self::SetManeuverNode { .. } => "flight.set_maneuver_node",
            Self::RemoveManeuverNode => "flight.remove_maneuver_node",
        }
    }
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
#[serde(tag = "action", rename_all = "snake_case")]
enum ScriptActionParams {
    SetSource { source: String },
    SetFilename { name: String },
    Save,
    Load,
    LoadCallbackExample,
    LoadCoroutineExample,
    Run,
    Stop,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct ScriptActionRequest {
    #[serde(flatten)]
    action: ScriptActionParams,
}

impl ScriptActionParams {
    fn name(&self) -> &'static str {
        match self {
            Self::SetSource { .. } => "script.set_source",
            Self::SetFilename { .. } => "script.set_filename",
            Self::Save => "script.save",
            Self::Load => "script.load",
            Self::LoadCallbackExample => "script.load_callback_example",
            Self::LoadCoroutineExample => "script.load_coroutine_example",
            Self::Run => "script.run",
            Self::Stop => "script.stop",
        }
    }
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
#[serde(tag = "action", rename_all = "snake_case")]
enum ViewActionParams {
    SetAssemblyScriptEditor { open: bool },
    SetMap { open: bool },
    SetSystemMap { enabled: bool },
    CycleCamera,
    SetFlightScriptConsole { open: bool },
    SetHelp { open: bool },
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct ViewActionRequest {
    #[serde(flatten)]
    action: ViewActionParams,
}

impl ViewActionParams {
    fn name(&self) -> &'static str {
        match self {
            Self::SetAssemblyScriptEditor { .. } => "view.set_assembly_script_editor",
            Self::SetMap { .. } => "view.set_map",
            Self::SetSystemMap { .. } => "view.set_system_map",
            Self::CycleCamera => "view.cycle_camera",
            Self::SetFlightScriptConsole { .. } => "view.set_flight_script_console",
            Self::SetHelp { .. } => "view.set_help",
        }
    }
}

fn start_mcp_server(bridge: Res<GameStateBridge>) {
    let bridge = bridge.clone();
    std::thread::Builder::new()
        .name("crabby-mcp".into())
        .spawn(move || {
            if let Err(error) = run_mcp_server(bridge) {
                eprintln!("Crabby MCP server stopped: {error}");
            }
        })
        .expect("failed to start the MCP server thread");
}

fn run_mcp_server(bridge: GameStateBridge) -> anyhow::Result<()> {
    let address = std::env::var("CRABBY_MCP_ADDR").unwrap_or_else(|_| DEFAULT_MCP_ADDR.into());
    let address: SocketAddr = address
        .parse()
        .map_err(|error| anyhow::anyhow!("invalid CRABBY_MCP_ADDR: {error}"))?;
    anyhow::ensure!(
        address.ip().is_loopback(),
        "CRABBY_MCP_ADDR must be a loopback address"
    );

    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .thread_name("crabby-mcp-worker")
        .build()?;
    runtime.block_on(async move {
        let service_bridge = bridge.clone();
        let service = StreamableHttpService::new(
            move || Ok(GameStateMcp::new(service_bridge.clone())),
            LocalSessionManager::default().into(),
            StreamableHttpServerConfig::default().with_json_response(true),
        );
        let router = Router::new().nest_service("/mcp", service);
        let listener = tokio::net::TcpListener::bind(address).await?;
        eprintln!("Crabby MCP listening at http://{address}/mcp");
        axum::serve(listener, router).await?;
        Ok(())
    })
}

fn process_mcp_requests(
    bridge: Res<GameStateBridge>,
    state: Res<State<AppMode>>,
    mut next_state: ResMut<NextState<AppMode>>,
    mut session: ResMut<Session>,
    catalog: Res<Catalog>,
    store: Res<Store>,
    mut editor: ResMut<EditorState>,
    mut view: ResMut<ViewState>,
    mut clock: ResMut<SimulationClock>,
    mut runtime: ResMut<ScriptRuntime>,
    mut remote_input: ResMut<RemoteFlightInput>,
    mut pending: ResMut<PendingReplies>,
) {
    // Process one request per frame so a queued mode transition is authoritative
    // before the next FIFO action validates its required mode.
    let request = bridge
        .requests
        .lock()
        .expect("MCP request queue poisoned")
        .pop_front();
    let Some(request) = request else {
        return;
    };

    let (kind, result, reply) = match request {
        McpRequest::Patch { patch, reply } => (
            ReplyKind::Patch,
            apply_patch_request(
                patch,
                *state.get(),
                &mut next_state,
                &mut session,
                &catalog.0,
                &mut clock,
            ),
            reply,
        ),
        McpRequest::Action {
            name,
            action,
            reply,
        } => (
            ReplyKind::Action(name),
            apply_game_action(
                action,
                *state.get(),
                &mut next_state,
                &mut session,
                &catalog,
                &store,
                &mut editor,
                &mut view,
                &mut clock,
                &mut runtime,
                &mut remote_input,
            ),
            reply,
        ),
    };
    pending.0.push_back(PendingReply {
        kind,
        result,
        reply,
    });
}

fn apply_patch_request(
    patch: Map<String, Value>,
    mode: AppMode,
    next_state: &mut NextState<AppMode>,
    session: &mut Session,
    catalog: &crate::model::PartCatalog,
    clock: &mut SimulationClock,
) -> Result<(), String> {
    let mut value = serde_json::to_value(DebugGameState::capture(mode, session, clock))
        .expect("game state is serializable");
    merge_patch(&mut value, Value::Object(patch));
    let patched = serde_json::from_value::<DebugGameState>(value)
        .map_err(|error| format!("patch does not produce a valid game state: {error}"))?;
    patched.validate()?;
    next_state.set(patched.mode);
    *clock = patched.clock;
    session.craft = patched.craft;
    session.vessel = patched.vessel;
    session.mission = patched.mission;
    session.notice = patched.notice;
    session.telemetry = session
        .vessel
        .as_ref()
        .map(|vessel| telemetry(vessel, catalog, clock.universal_time, 0.0))
        .unwrap_or_default();
    session.visual_dirty = true;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn apply_game_action(
    action: GameAction,
    mode: AppMode,
    next_state: &mut NextState<AppMode>,
    session: &mut Session,
    catalog: &Catalog,
    store: &Store,
    editor: &mut EditorState,
    view: &mut ViewState,
    clock: &mut SimulationClock,
    runtime: &mut ScriptRuntime,
    remote_input: &mut RemoteFlightInput,
) -> Result<(), String> {
    match action {
        GameAction::Menu(action) => {
            require_mode(mode, AppMode::Menu)?;
            match action {
                MenuActionParams::OpenVehicleAssembly => {
                    enter_vehicle_assembly(session, runtime, next_state);
                    Ok(())
                }
                MenuActionParams::ContinueQuicksave => continue_quicksave(
                    session,
                    &store.0,
                    clock,
                    runtime,
                    view,
                    next_state,
                    remote_input,
                ),
            }
        }
        GameAction::Assembly(action) => {
            require_mode(mode, AppMode::Editor)?;
            apply_assembly_action(
                action,
                next_state,
                session,
                &catalog.0,
                &store.0,
                editor,
                runtime,
                remote_input,
            )
        }
        GameAction::SetFlightControls(params) => {
            require_flight(mode, session)?;
            set_flight_controls(params, session, remote_input)
        }
        GameAction::Flight(action) => {
            require_flight(mode, session)?;
            apply_flight_action(
                action,
                next_state,
                session,
                &catalog.0,
                &store.0,
                clock,
                runtime,
                remote_input,
            )
        }
        GameAction::Script(action) => {
            apply_script_action(action, mode, session, &store.0, editor, runtime)
        }
        GameAction::View(action) => apply_view_action(action, mode, editor, view),
    }
}

fn require_mode(actual: AppMode, required: AppMode) -> Result<(), String> {
    if actual == required {
        Ok(())
    } else {
        Err(format!(
            "action requires {required:?} mode; current mode is {actual:?}"
        ))
    }
}

fn require_flight(mode: AppMode, session: &Session) -> Result<(), String> {
    require_mode(mode, AppMode::Flight)?;
    if session.vessel.is_some() {
        Ok(())
    } else {
        Err("flight action requires an active vessel".into())
    }
}

fn apply_assembly_action(
    action: AssemblyActionParams,
    next_state: &mut NextState<AppMode>,
    session: &mut Session,
    catalog: &crate::model::PartCatalog,
    store: &crate::save::SaveStore,
    editor: &mut EditorState,
    runtime: &mut ScriptRuntime,
    remote_input: &mut RemoteFlightInput,
) -> Result<(), String> {
    let error_prefix = match &action {
        AssemblyActionParams::SaveCraft => Some("Craft save failed:"),
        AssemblyActionParams::LoadCraft { .. } => Some("Load failed:"),
        _ => None,
    };
    let editor_action = match action {
        AssemblyActionParams::RenameCraft { name } => {
            if name.trim().is_empty() {
                return Err("craft name cannot be empty".into());
            }
            session.craft.name = name;
            return Ok(());
        }
        AssemblyActionParams::NewCraft => EditorAction::New,
        AssemblyActionParams::Undo => {
            if editor.history.is_empty() {
                return Err("there is no assembly action to undo".into());
            }
            EditorAction::Undo
        }
        AssemblyActionParams::Redo => {
            if editor.future.is_empty() {
                return Err("there is no assembly action to redo".into());
            }
            EditorAction::Redo
        }
        AssemblyActionParams::SelectPart { instance_id } => {
            if !session
                .craft
                .parts
                .iter()
                .any(|part| part.instance_id == instance_id)
            {
                return Err(format!("craft has no part with instance_id {instance_id}"));
            }
            editor.selected = Some(instance_id);
            return Ok(());
        }
        AssemblyActionParams::SetSymmetry { count } => {
            if ![1, 2, 4].contains(&count) {
                return Err("symmetry count must be 1, 2, or 4".into());
            }
            editor.symmetry = count;
            return Ok(());
        }
        AssemblyActionParams::AddPart { definition_id } => {
            if catalog.get(&definition_id).is_none() {
                return Err(format!("part catalog has no definition {definition_id}"));
            }
            if session.craft.parts.len() >= crate::model::MAX_PARTS {
                return Err(format!(
                    "vehicle has reached the {}-part limit",
                    crate::model::MAX_PARTS
                ));
            }
            EditorAction::Add(definition_id)
        }
        AssemblyActionParams::RemoveSelectedSubtree => {
            let selected = editor
                .selected
                .ok_or_else(|| "no assembly part is selected".to_string())?;
            if session.craft.root() == Some(selected) {
                session.notice =
                    "The root command pod cannot be removed; start a new craft instead.".into();
                return Err(session.notice.clone());
            }
            EditorAction::Remove
        }
        AssemblyActionParams::SaveCraft => EditorAction::Save,
        AssemblyActionParams::LoadCraft { name } => EditorAction::Load(name),
        AssemblyActionParams::Launch => {
            if let Some(ValidationIssue::Error(error)) = session
                .craft
                .validate(catalog)
                .into_iter()
                .find(|issue| matches!(issue, ValidationIssue::Error(_)))
            {
                session.notice = format!("Cannot launch: {error}");
                return Err(session.notice.clone());
            }
            EditorAction::Launch
        }
    };

    apply_editor_action(
        editor_action,
        next_state,
        session,
        catalog,
        store,
        editor,
        runtime,
        remote_input,
    );
    if error_prefix.is_some_and(|prefix| session.notice.starts_with(prefix)) {
        Err(session.notice.clone())
    } else {
        Ok(())
    }
}

fn set_flight_controls(
    params: SetFlightControlsParams,
    session: &mut Session,
    remote_input: &mut RemoteFlightInput,
) -> Result<(), String> {
    if params.throttle.is_none()
        && params.pitch.is_none()
        && params.yaw.is_none()
        && params.roll.is_none()
    {
        return Err("provide at least one flight control value".into());
    }
    for (name, value, range) in [
        ("throttle", params.throttle, 0.0..=1.0),
        ("pitch", params.pitch, -1.0..=1.0),
        ("yaw", params.yaw, -1.0..=1.0),
        ("roll", params.roll, -1.0..=1.0),
    ] {
        if let Some(value) = value
            && (!value.is_finite() || !range.contains(&value))
        {
            return Err(format!("{name} is outside its valid range"));
        }
    }
    let vessel = session.vessel.as_mut().expect("flight mode has a vessel");
    if let Some(value) = params.throttle {
        vessel.controls.throttle = value;
    }
    if let Some(value) = params.pitch {
        remote_input.pitch = Some(value);
        vessel.controls.pitch = value;
    }
    if let Some(value) = params.yaw {
        remote_input.yaw = Some(value);
        vessel.controls.yaw = value;
    }
    if let Some(value) = params.roll {
        remote_input.roll = Some(value);
        vessel.controls.roll = value;
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn apply_flight_action(
    action: FlightActionParams,
    next_state: &mut NextState<AppMode>,
    session: &mut Session,
    catalog: &crate::model::PartCatalog,
    store: &crate::save::SaveStore,
    clock: &mut SimulationClock,
    runtime: &mut ScriptRuntime,
    remote_input: &mut RemoteFlightInput,
) -> Result<(), String> {
    match action {
        FlightActionParams::ReturnToAssembly => {
            return_to_assembly(session, runtime, next_state, remote_input);
        }
        FlightActionParams::ActivateNextStage => {
            let Session {
                vessel,
                visual_dirty,
                ..
            } = session;
            if !activate_stage(
                vessel.as_mut().expect("flight mode has a vessel"),
                catalog,
                visual_dirty,
                runtime,
            ) {
                return Err("the vessel has no remaining stages".into());
            }
        }
        FlightActionParams::ReleaseAttitudeControls => {
            remote_input.release_attitude_controls();
            let vessel = session.vessel.as_mut().expect("flight mode has a vessel");
            vessel.controls.pitch = 0.0;
            vessel.controls.yaw = 0.0;
            vessel.controls.roll = 0.0;
        }
        FlightActionParams::SetSas { mode } => {
            session
                .vessel
                .as_mut()
                .expect("flight mode has a vessel")
                .controls
                .sas = mode.value();
        }
        FlightActionParams::SetRcs { enabled } => {
            session
                .vessel
                .as_mut()
                .expect("flight mode has a vessel")
                .controls
                .rcs = enabled;
        }
        FlightActionParams::IncreaseWarp => {
            clock.warp_index = (clock.warp_index + 1).min(SimulationClock::WARP_RATES.len() - 1);
        }
        FlightActionParams::DecreaseWarp => {
            clock.warp_index = clock.warp_index.saturating_sub(1);
        }
        FlightActionParams::SetPaused { paused } => clock.paused = paused,
        FlightActionParams::Quicksave => {
            let result = save_quicksave(
                session.vessel.as_ref().expect("flight mode has a vessel"),
                &session.mission,
                clock,
                runtime,
                store,
            );
            session.notice = match result {
                Ok(message) => message,
                Err(error) => {
                    session.notice.clone_from(&error);
                    return Err(error);
                }
            };
        }
        FlightActionParams::Quickload => {
            load_quicksave(session, store, clock, runtime, remote_input);
            if session.notice.starts_with("Load failed:") {
                return Err(session.notice.clone());
            }
        }
        FlightActionParams::AddManeuverNode => {
            let vessel = session.vessel.as_mut().expect("flight mode has a vessel");
            if vessel.maneuver.is_some() {
                return Err("the vessel already has a maneuver node".into());
            }
            vessel.maneuver = Some(ManeuverNode {
                ut: clock.universal_time + 60.0,
                prograde: 0.0,
                normal: 0.0,
                radial: 0.0,
            });
        }
        FlightActionParams::SetManeuverNode {
            ut,
            prograde,
            normal,
            radial,
        } => {
            if ![ut, prograde, normal, radial]
                .into_iter()
                .all(f64::is_finite)
            {
                return Err("maneuver values must be finite".into());
            }
            if !(clock.universal_time..=clock.universal_time + 1.0e7).contains(&ut) {
                return Err("maneuver UT is outside the player-editable range".into());
            }
            let vessel = session.vessel.as_mut().expect("flight mode has a vessel");
            if vessel.maneuver.is_none() {
                return Err("the vessel has no maneuver node to edit".into());
            }
            vessel.maneuver = Some(ManeuverNode {
                ut,
                prograde,
                normal,
                radial,
            });
        }
        FlightActionParams::RemoveManeuverNode => {
            let vessel = session.vessel.as_mut().expect("flight mode has a vessel");
            if vessel.maneuver.take().is_none() {
                return Err("the vessel has no maneuver node to remove".into());
            }
        }
    }
    Ok(())
}

fn apply_script_action(
    action: ScriptActionParams,
    mode: AppMode,
    session: &mut Session,
    store: &crate::save::SaveStore,
    editor: &mut EditorState,
    runtime: &mut ScriptRuntime,
) -> Result<(), String> {
    match action {
        ScriptActionParams::SetSource { source } => {
            if !matches!(mode, AppMode::Editor | AppMode::Flight) {
                return Err("script source can only be edited in assembly or flight".into());
            }
            runtime.source = source;
        }
        ScriptActionParams::SetFilename { name } => {
            require_mode(mode, AppMode::Editor)?;
            if name.trim().is_empty() {
                return Err("script filename cannot be empty".into());
            }
            editor.script_name = name;
        }
        ScriptActionParams::Save => {
            let name = match mode {
                AppMode::Editor => &editor.script_name,
                AppMode::Flight => "flight_computer",
                AppMode::Menu => return Err("scripts cannot be saved from the menu".into()),
            };
            session.notice = store
                .save_script(name, &runtime.source)
                .map(|path| format!("Saved {}", path.display()))
                .map_err(|error| format!("Save failed: {error}"))?;
        }
        ScriptActionParams::Load => {
            require_mode(mode, AppMode::Editor)?;
            runtime.source = store.load_script(&editor.script_name).map_err(|error| {
                let message = format!("Script load failed: {error}");
                session.notice.clone_from(&message);
                message
            })?;
        }
        ScriptActionParams::LoadCallbackExample => {
            require_mode(mode, AppMode::Editor)?;
            runtime.source = EXAMPLE_SCRIPT.into();
        }
        ScriptActionParams::LoadCoroutineExample => {
            require_mode(mode, AppMode::Editor)?;
            runtime.source = COROUTINE_EXAMPLE.into();
        }
        ScriptActionParams::Run => {
            require_mode(mode, AppMode::Flight)?;
            runtime
                .load(runtime.source.clone(), None)
                .map_err(|error| {
                    let message = format!("Lua error: {error}");
                    session.notice.clone_from(&message);
                    message
                })?;
            session.notice = "Automation running; F8 stops immediately.".into();
        }
        ScriptActionParams::Stop => {
            require_mode(mode, AppMode::Flight)?;
            runtime.stop();
        }
    }
    Ok(())
}

fn apply_view_action(
    action: ViewActionParams,
    mode: AppMode,
    editor: &mut EditorState,
    view: &mut ViewState,
) -> Result<(), String> {
    match action {
        ViewActionParams::SetAssemblyScriptEditor { open } => {
            require_mode(mode, AppMode::Editor)?;
            editor.show_script = open;
        }
        ViewActionParams::SetMap { open } => {
            require_mode(mode, AppMode::Flight)?;
            view.map = open;
        }
        ViewActionParams::SetSystemMap { enabled } => {
            require_mode(mode, AppMode::Flight)?;
            if !view.map {
                return Err("system map can only be changed while the map is open".into());
            }
            view.system_map = enabled;
        }
        ViewActionParams::CycleCamera => {
            require_mode(mode, AppMode::Flight)?;
            view.camera_mode = (view.camera_mode + 1) % 3;
        }
        ViewActionParams::SetFlightScriptConsole { open } => {
            require_mode(mode, AppMode::Flight)?;
            view.show_script_console = open;
        }
        ViewActionParams::SetHelp { open } => {
            require_mode(mode, AppMode::Flight)?;
            view.show_help = open;
        }
    }
    Ok(())
}

fn capture_player_state(
    mode: AppMode,
    catalog: &Catalog,
    store: &Store,
    editor: &EditorState,
    view: &ViewState,
    runtime: &ScriptRuntime,
    remote_input: &RemoteFlightInput,
) -> Value {
    let script_mode = runtime.mode.map(|mode| match mode {
        ScriptMode::Callback => "callback",
        ScriptMode::Coroutine => "coroutine",
    });
    let part_catalog: Vec<_> = catalog
        .0
        .iter()
        .map(|part| {
            json!({
                "id": part.id,
                "name": part.name,
                "category": part.category,
                "description": part.description,
                "radial": part.radial,
            })
        })
        .collect();
    json!({
        "mode": mode,
        "menu": {
            "quicksave_available": store.0.quicksave_exists(),
        },
        "assembly": {
            "selected_part": editor.selected,
            "symmetry": editor.symmetry,
            "can_undo": !editor.history.is_empty(),
            "can_redo": !editor.future.is_empty(),
            "script_filename": editor.script_name,
            "script_editor_open": editor.show_script,
            "saved_crafts": store.0.list_crafts(),
            "part_catalog": part_catalog,
        },
        "flight_view": {
            "map_open": view.map,
            "system_map": view.system_map,
            "camera_mode": view.camera_mode,
            "script_console_open": view.show_script_console,
            "help_open": view.show_help,
        },
        "script": {
            "source": runtime.source,
            "active": runtime.active,
            "mode": script_mode,
            "last_error": runtime.last_error,
            "log": view.script_log,
        },
        "mcp_attitude_overrides": {
            "pitch": remote_input.pitch,
            "yaw": remote_input.yaw,
            "roll": remote_input.roll,
        },
    })
}

fn publish_game_state(
    bridge: Res<GameStateBridge>,
    state: Res<State<AppMode>>,
    session: Res<Session>,
    catalog: Res<Catalog>,
    store: Res<Store>,
    editor: Res<EditorState>,
    view: Res<ViewState>,
    clock: Res<SimulationClock>,
    runtime: Res<ScriptRuntime>,
    remote_input: Res<RemoteFlightInput>,
    mut pending: ResMut<PendingReplies>,
) {
    let game = serde_json::to_value(DebugGameState::capture(*state.get(), &session, &clock))
        .expect("game state is serializable");
    let player = capture_player_state(
        *state.get(),
        &catalog,
        &store,
        &editor,
        &view,
        &runtime,
        &remote_input,
    );
    *bridge
        .game_snapshot
        .write()
        .expect("MCP game snapshot poisoned") = game.clone();
    *bridge
        .player_snapshot
        .write()
        .expect("MCP player snapshot poisoned") = player.clone();

    while let Some(pending_reply) = pending.0.pop_front() {
        let value = match (pending_reply.kind, pending_reply.result) {
            (ReplyKind::Patch, Ok(())) => Ok(game.clone()),
            (ReplyKind::Patch, Err(error)) => Err(error),
            (ReplyKind::Action(action), Ok(())) => Ok(json!({
                "action": action,
                "status": "completed",
                "game_state": game,
                "player_state": player,
            })),
            (ReplyKind::Action(action), Err(error)) => Ok(json!({
                "action": action,
                "status": "rejected",
                "error": error,
                "game_state": game,
                "player_state": player,
            })),
        };
        let _ = pending_reply.reply.send(value);
    }
}

fn merge_patch(target: &mut Value, patch: Value) {
    let Value::Object(patch) = patch else {
        *target = patch;
        return;
    };
    if !target.is_object() {
        *target = Value::Object(Map::new());
    }
    let target = target.as_object_mut().expect("target was made an object");
    for (key, patch_value) in patch {
        if patch_value.is_null() {
            target.remove(&key);
        } else {
            merge_patch(target.entry(key).or_insert(Value::Null), patch_value);
        }
    }
}

#[derive(Clone)]
struct GameStateMcp {
    bridge: GameStateBridge,
}

impl GameStateMcp {
    fn new(bridge: GameStateBridge) -> Self {
        Self { bridge }
    }

    fn result(value: Value) -> CallToolResult {
        let text = serde_json::to_string_pretty(&value).unwrap_or_else(|_| value.to_string());
        let mut result = CallToolResult::success(vec![ContentBlock::text(text)]);
        result.structured_content = Some(value);
        result
    }

    fn error_result(value: Value) -> CallToolResult {
        let text = serde_json::to_string_pretty(&value).unwrap_or_else(|_| value.to_string());
        let mut result = CallToolResult::error(vec![ContentBlock::text(text)]);
        result.structured_content = Some(value);
        result
    }

    async fn await_response(
        &self,
        response: oneshot::Receiver<Result<Value, String>>,
    ) -> CallToolResult {
        match tokio::time::timeout(REQUEST_TIMEOUT, response).await {
            Ok(Ok(Ok(value))) if value["status"] == "rejected" => Self::error_result(value),
            Ok(Ok(Ok(value))) => Self::result(value),
            Ok(Ok(Err(error))) => CallToolResult::error(vec![ContentBlock::text(format!(
                "Action rejected: {error}"
            ))]),
            Ok(Err(_)) => CallToolResult::error(vec![ContentBlock::text(
                "The game closed before applying the request",
            )]),
            Err(_) => CallToolResult::error(vec![ContentBlock::text(
                "Timed out waiting for Bevy's update loop to apply the request",
            )]),
        }
    }

    async fn enqueue_patch(&self, patch: Map<String, Value>) -> CallToolResult {
        let (reply, response) = oneshot::channel();
        self.bridge
            .requests
            .lock()
            .expect("MCP request queue poisoned")
            .push_back(McpRequest::Patch { patch, reply });
        self.await_response(response).await
    }

    async fn enqueue_action(&self, name: String, action: GameAction) -> CallToolResult {
        let (reply, response) = oneshot::channel();
        self.bridge
            .requests
            .lock()
            .expect("MCP request queue poisoned")
            .push_back(McpRequest::Action {
                name,
                action,
                reply,
            });
        self.await_response(response).await
    }
}

#[tool_router]
impl GameStateMcp {
    #[tool(
        description = "Return an atomic snapshot of the live game state, including mode, simulation clock, craft blueprint, active vessel, mission progress, and UI notice"
    )]
    fn inspect_game_state(&self) -> CallToolResult {
        Self::result(
            self.bridge
                .game_snapshot
                .read()
                .expect("MCP game snapshot poisoned")
                .clone(),
        )
    }

    #[tool(
        description = "Return player-facing interaction state: assembly selection and catalog, saves, view state, Lua state, and latched MCP attitude controls"
    )]
    fn inspect_player_state(&self) -> CallToolResult {
        Self::result(
            self.bridge
                .player_snapshot
                .read()
                .expect("MCP player snapshot poisoned")
                .clone(),
        )
    }

    #[tool(
        description = "Atomically mutate live game state with an RFC 7396 JSON Merge Patch. Read inspect_game_state first, patch only intended fields, and pause the clock when editing coupled flight values"
    )]
    async fn patch_game_state(
        &self,
        Parameters(PatchGameStateParams { patch }): Parameters<PatchGameStateParams>,
    ) -> CallToolResult {
        self.enqueue_patch(patch).await
    }

    #[tool(
        description = "Perform a player-equivalent menu action. Actions require menu mode and intentionally exclude Quit"
    )]
    async fn menu_action(
        &self,
        Parameters(MenuActionRequest { action: params }): Parameters<MenuActionRequest>,
    ) -> CallToolResult {
        let name = params.name().into();
        self.enqueue_action(name, GameAction::Menu(params)).await
    }

    #[tool(
        description = "Perform a player-equivalent vehicle assembly action using the normal editor history, validation, staging, save/load, and launch paths"
    )]
    async fn assembly_action(
        &self,
        Parameters(AssemblyActionRequest { action: params }): Parameters<AssemblyActionRequest>,
    ) -> CallToolResult {
        let name = params.name().into();
        self.enqueue_action(name, GameAction::Assembly(params))
            .await
    }

    #[tool(
        description = "Set flight controls. Throttle latches like player throttle; supplied pitch/yaw/roll axes remain overridden until released explicitly or by a flight lifecycle transition"
    )]
    async fn set_flight_controls(
        &self,
        Parameters(params): Parameters<SetFlightControlsParams>,
    ) -> CallToolResult {
        self.enqueue_action(
            "flight.set_flight_controls".into(),
            GameAction::SetFlightControls(params),
        )
        .await
    }

    #[tool(
        description = "Perform a player-equivalent flight action for staging, SAS/RCS, warp/pause, quicksave/load, maneuver nodes, or returning to assembly"
    )]
    async fn flight_action(
        &self,
        Parameters(FlightActionRequest { action: params }): Parameters<FlightActionRequest>,
    ) -> CallToolResult {
        let name = params.name().into();
        self.enqueue_action(name, GameAction::Flight(params)).await
    }

    #[tool(
        description = "Perform a player-equivalent Lua editor or flight-computer action; availability depends on the current game mode"
    )]
    async fn script_action(
        &self,
        Parameters(ScriptActionRequest { action: params }): Parameters<ScriptActionRequest>,
    ) -> CallToolResult {
        let name = params.name().into();
        self.enqueue_action(name, GameAction::Script(params)).await
    }

    #[tool(
        description = "Perform a player-equivalent view action for assembly/flight windows, map modes, help, and camera cycling"
    )]
    async fn view_action(
        &self,
        Parameters(ViewActionRequest { action: params }): Parameters<ViewActionRequest>,
    ) -> CallToolResult {
        let name = params.name().into();
        self.enqueue_action(name, GameAction::View(params)).await
    }
}

#[tool_handler]
impl ServerHandler for GameStateMcp {
    fn get_info(&self) -> ServerInfo {
        ServerInfo::new(ServerCapabilities::builder().enable_tools().build())
            .with_server_info(Implementation::from_build_env())
            .with_instructions(
                "Inspect, play, and mutate the live Crabby Space Institute game for debugging and development. Prefer semantic action tools for player-equivalent interaction; use patch_game_state only for low-level debugging. Mutations affect the running game immediately.",
            )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_app(bridge: GameStateBridge) -> App {
        let mut app = App::new();
        app.add_plugins(bevy::state::app::StatesPlugin)
            .init_state::<AppMode>()
            .insert_resource(Session::default())
            .insert_resource(Catalog(crate::model::PartCatalog::default()))
            .insert_resource(Store(crate::save::SaveStore::default()))
            .insert_resource(EditorState::default())
            .insert_resource(ViewState::default())
            .insert_resource(SimulationClock::default())
            .insert_resource(ScriptRuntime::default())
            .insert_resource(RemoteFlightInput::default())
            .insert_resource(bridge)
            .init_resource::<PendingReplies>()
            .add_systems(PreUpdate, process_mcp_requests)
            .add_systems(
                Update,
                super::super::simulate_flight.run_if(in_state(AppMode::Flight)),
            )
            .add_systems(Last, publish_game_state);
        app
    }

    fn run_request(
        app: &mut App,
        bridge: &GameStateBridge,
        action: GameAction,
        name: &str,
    ) -> Result<Value, String> {
        let (reply, response) = oneshot::channel();
        bridge
            .requests
            .lock()
            .unwrap()
            .push_back(McpRequest::Action {
                name: name.into(),
                action,
                reply,
            });
        app.update();
        response.blocking_recv().unwrap()
    }

    #[test]
    fn merge_patch_updates_nested_values_and_removes_nulls() {
        let mut target = json!({
            "clock": { "paused": false, "warp_index": 1 },
            "notice": "old"
        });
        merge_patch(
            &mut target,
            json!({
                "clock": { "paused": true },
                "notice": null
            }),
        );
        assert_eq!(
            target,
            json!({
                "clock": { "paused": true, "warp_index": 1 }
            })
        );
    }

    #[test]
    fn validation_rejects_an_out_of_range_warp_index() {
        let session = Session::default();
        let mut state =
            DebugGameState::capture(AppMode::Menu, &session, &SimulationClock::default());
        state.clock.warp_index = SimulationClock::WARP_RATES.len();
        assert!(state.validate().unwrap_err().contains("warp_index"));
    }

    #[test]
    fn mcp_advertises_inspection_mutation_and_action_tools() {
        let router = GameStateMcp::tool_router();
        for route in [
            "inspect_game_state",
            "inspect_player_state",
            "patch_game_state",
            "menu_action",
            "assembly_action",
            "set_flight_controls",
            "flight_action",
            "script_action",
            "view_action",
        ] {
            assert!(router.has_route(route), "missing MCP route {route}");
        }
    }

    #[test]
    fn patch_tool_round_trips_through_bevys_update_loop() {
        let bridge = GameStateBridge::default();
        let mut app = test_app(bridge.clone());
        app.update();
        let (reply, response) = oneshot::channel();
        bridge
            .requests
            .lock()
            .unwrap()
            .push_back(McpRequest::Patch {
                patch: serde_json::from_value(json!({
                    "clock": { "paused": true },
                    "notice": "patched through MCP"
                }))
                .unwrap(),
                reply,
            });
        app.update();
        let value = response.blocking_recv().unwrap().unwrap();
        assert_eq!(value["notice"], "patched through MCP");
        assert!(app.world().resource::<SimulationClock>().paused);
    }

    #[test]
    fn semantic_actions_reach_a_real_ignited_flight() {
        let bridge = GameStateBridge::default();
        let mut app = test_app(bridge.clone());
        app.update();

        run_request(
            &mut app,
            &bridge,
            GameAction::Menu(MenuActionParams::OpenVehicleAssembly),
            "menu.open_vehicle_assembly",
        )
        .unwrap();
        assert_eq!(
            *app.world().resource::<State<AppMode>>().get(),
            AppMode::Editor
        );

        run_request(
            &mut app,
            &bridge,
            GameAction::Assembly(AssemblyActionParams::Launch),
            "assembly.launch",
        )
        .unwrap();
        assert_eq!(
            *app.world().resource::<State<AppMode>>().get(),
            AppMode::Flight
        );

        run_request(
            &mut app,
            &bridge,
            GameAction::SetFlightControls(SetFlightControlsParams {
                throttle: Some(1.0),
                pitch: None,
                yaw: None,
                roll: None,
            }),
            "flight.set_flight_controls",
        )
        .unwrap();
        run_request(
            &mut app,
            &bridge,
            GameAction::Flight(FlightActionParams::ActivateNextStage),
            "flight.activate_next_stage",
        )
        .unwrap();

        {
            let session = app.world().resource::<Session>();
            let vessel = session.vessel.as_ref().unwrap();
            assert_eq!(vessel.controls.throttle, 1.0);
            assert_eq!(vessel.next_stage, 1);
            assert!(vessel.parts.iter().any(|part| part.active));
        }

        for _ in 0..600 {
            app.update();
            if app.world().resource::<Session>().mission.launched {
                break;
            }
        }
        let session = app.world().resource::<Session>();
        let vessel = session.vessel.as_ref().unwrap();
        assert!(session.mission.launched);
        assert!(vessel.position[1] > crate::model::HOME_RADIUS + 50.0);
    }

    #[test]
    fn attitude_controls_latch_and_release() {
        let bridge = GameStateBridge::default();
        let mut app = test_app(bridge.clone());
        app.update();
        run_request(
            &mut app,
            &bridge,
            GameAction::Menu(MenuActionParams::OpenVehicleAssembly),
            "menu.open_vehicle_assembly",
        )
        .unwrap();
        run_request(
            &mut app,
            &bridge,
            GameAction::Assembly(AssemblyActionParams::Launch),
            "assembly.launch",
        )
        .unwrap();
        run_request(
            &mut app,
            &bridge,
            GameAction::SetFlightControls(SetFlightControlsParams {
                throttle: None,
                pitch: Some(0.5),
                yaw: Some(-0.25),
                roll: None,
            }),
            "flight.set_flight_controls",
        )
        .unwrap();
        let remote = app.world().resource::<RemoteFlightInput>();
        assert_eq!(remote.pitch, Some(0.5));
        assert_eq!(remote.yaw, Some(-0.25));

        run_request(
            &mut app,
            &bridge,
            GameAction::Flight(FlightActionParams::ReleaseAttitudeControls),
            "flight.release_attitude_controls",
        )
        .unwrap();
        let remote = app.world().resource::<RemoteFlightInput>();
        assert_eq!(remote.pitch, None);
        assert_eq!(remote.yaw, None);
    }

    #[test]
    fn wrong_mode_actions_return_structured_rejections_without_mutating_state() {
        let bridge = GameStateBridge::default();
        let mut app = test_app(bridge.clone());
        app.update();

        let result = run_request(
            &mut app,
            &bridge,
            GameAction::Assembly(AssemblyActionParams::Launch),
            "assembly.launch",
        )
        .unwrap();
        assert_eq!(result["status"], "rejected");
        assert!(result["error"].as_str().unwrap().contains("Editor mode"));
        assert_eq!(
            *app.world().resource::<State<AppMode>>().get(),
            AppMode::Menu
        );
        assert!(app.world().resource::<Session>().vessel.is_none());
    }
}
