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
use serde_json::{Map, Value};
use tokio::sync::oneshot;

use super::{AppMode, Catalog, Session};
use crate::model::{CraftBlueprint, Vessel};
use crate::orbit::celestial_system;
use crate::simulation::{MissionProgress, SimulationClock, telemetry};

const DEFAULT_MCP_ADDR: &str = "127.0.0.1:8765";
const MUTATION_TIMEOUT: Duration = Duration::from_secs(3);

pub(super) struct GameMcpPlugin;

impl Plugin for GameMcpPlugin {
    fn build(&self, app: &mut App) {
        let bridge = GameStateBridge::default();
        app.insert_resource(bridge.clone())
            .add_systems(Startup, start_mcp_server)
            .add_systems(Last, synchronize_game_state);
    }
}

#[derive(Clone, Resource)]
struct GameStateBridge {
    snapshot: Arc<RwLock<Value>>,
    mutations: Arc<Mutex<VecDeque<MutationRequest>>>,
}

impl Default for GameStateBridge {
    fn default() -> Self {
        Self {
            snapshot: Arc::new(RwLock::new(serde_json::json!({
                "status": "starting",
                "message": "The game has not completed its first update yet"
            }))),
            mutations: Arc::default(),
        }
    }
}

struct MutationRequest {
    patch: Map<String, Value>,
    reply: oneshot::Sender<Result<Value, String>>,
}

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

fn synchronize_game_state(
    bridge: Res<GameStateBridge>,
    state: Res<State<AppMode>>,
    mut next_state: ResMut<NextState<AppMode>>,
    mut session: ResMut<Session>,
    catalog: Res<Catalog>,
    mut clock: ResMut<SimulationClock>,
) {
    loop {
        let request = bridge
            .mutations
            .lock()
            .expect("MCP mutation queue poisoned")
            .pop_front();
        let Some(request) = request else {
            break;
        };

        let mut value =
            serde_json::to_value(DebugGameState::capture(*state.get(), &session, &clock))
                .expect("game state is serializable");
        merge_patch(&mut value, Value::Object(request.patch));
        let result = serde_json::from_value::<DebugGameState>(value)
            .map_err(|error| format!("patch does not produce a valid game state: {error}"))
            .and_then(|patched| {
                patched.validate()?;
                next_state.set(patched.mode);
                *clock = patched.clock.clone();
                session.craft = patched.craft.clone();
                session.vessel = patched.vessel.clone();
                session.mission = patched.mission.clone();
                session.notice = patched.notice.clone();
                session.telemetry = session
                    .vessel
                    .as_ref()
                    .map(|vessel| telemetry(vessel, &catalog.0, clock.universal_time, 0.0))
                    .unwrap_or_default();
                session.visual_dirty = true;
                serde_json::to_value(patched).map_err(|error| error.to_string())
            });
        let _ = request.reply.send(result);
    }

    let snapshot = DebugGameState::capture(*state.get(), &session, &clock);
    if let Ok(value) = serde_json::to_value(snapshot) {
        *bridge.snapshot.write().expect("MCP snapshot poisoned") = value;
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

#[derive(Debug, Deserialize, schemars::JsonSchema)]
struct PatchGameStateParams {
    /// RFC 7396 JSON Merge Patch applied to the state returned by inspect_game_state.
    patch: Map<String, Value>,
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
}

#[tool_router]
impl GameStateMcp {
    #[tool(
        description = "Return an atomic snapshot of the live game state, including mode, simulation clock, craft blueprint, active vessel, mission progress, and UI notice"
    )]
    fn inspect_game_state(&self) -> CallToolResult {
        Self::result(
            self.bridge
                .snapshot
                .read()
                .expect("MCP snapshot poisoned")
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
        let (reply, response) = oneshot::channel();
        self.bridge
            .mutations
            .lock()
            .expect("MCP mutation queue poisoned")
            .push_back(MutationRequest { patch, reply });

        match tokio::time::timeout(MUTATION_TIMEOUT, response).await {
            Ok(Ok(Ok(value))) => Self::result(value),
            Ok(Ok(Err(error))) => {
                CallToolResult::error(vec![ContentBlock::text(format!("Patch rejected: {error}"))])
            }
            Ok(Err(_)) => CallToolResult::error(vec![ContentBlock::text(
                "The game closed before applying the patch",
            )]),
            Err(_) => CallToolResult::error(vec![ContentBlock::text(
                "Timed out waiting for Bevy's update loop to apply the patch",
            )]),
        }
    }
}

#[tool_handler]
impl ServerHandler for GameStateMcp {
    fn get_info(&self) -> ServerInfo {
        ServerInfo::new(ServerCapabilities::builder().enable_tools().build())
            .with_server_info(Implementation::from_build_env())
            .with_instructions(
                "Inspect and mutate the live Crabby Space Institute game for debugging and \
                 development. Mutations affect the running game immediately.",
            )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn merge_patch_updates_nested_values_and_removes_nulls() {
        let mut target = serde_json::json!({
            "clock": { "paused": false, "warp_index": 1 },
            "notice": "old"
        });
        merge_patch(
            &mut target,
            serde_json::json!({
                "clock": { "paused": true },
                "notice": null
            }),
        );
        assert_eq!(
            target,
            serde_json::json!({
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
    fn mcp_advertises_inspection_and_mutation_tools() {
        let router = GameStateMcp::tool_router();
        assert!(router.has_route("inspect_game_state"));
        assert!(router.has_route("patch_game_state"));
    }

    #[test]
    fn patch_tool_round_trips_through_bevys_update_loop() {
        let bridge = GameStateBridge::default();
        let mut app = App::new();
        app.add_plugins(bevy::state::app::StatesPlugin)
            .init_state::<AppMode>()
            .insert_resource(Session::default())
            .insert_resource(Catalog(crate::model::PartCatalog::default()))
            .insert_resource(SimulationClock::default())
            .insert_resource(bridge.clone())
            .add_systems(Update, synchronize_game_state);
        app.update();

        let server = GameStateMcp::new(bridge.clone());
        let patch_thread = std::thread::spawn(move || {
            tokio::runtime::Builder::new_current_thread()
                .enable_time()
                .build()
                .unwrap()
                .block_on(
                    server.patch_game_state(Parameters(PatchGameStateParams {
                        patch: serde_json::from_value(serde_json::json!({
                            "clock": { "paused": true },
                            "notice": "patched through MCP"
                        }))
                        .unwrap(),
                    })),
                )
        });

        while bridge
            .mutations
            .lock()
            .expect("MCP mutation queue poisoned")
            .is_empty()
        {
            std::thread::yield_now();
        }
        app.update();

        let result = patch_thread.join().unwrap();
        assert_ne!(result.is_error, Some(true));
        assert_eq!(
            result.structured_content.unwrap()["notice"],
            "patched through MCP"
        );
        assert!(app.world().resource::<SimulationClock>().paused);
        assert_eq!(
            app.world().resource::<Session>().notice,
            "patched through MCP"
        );
    }
}
