use bevy::prelude::Resource;
use mlua::thread::ThreadStatus;
use mlua::{Function, HookTriggers, Lua, LuaSerdeExt, Table, Thread, Value, VmState};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use crate::model::SasMode;
use crate::simulation::FlightTelemetry;

pub const EXAMPLE_SCRIPT: &str = r#"-- Crabby Space Institute: orbit insertion (callback style)
state = state or { phase = 0, boosters_dropped = false, throttle = 1.0 }

local function regulate_twr(target)
  state.throttle = state.throttle or 1.0
  local current = resources.twr()
  if current > 0.1 then
    state.throttle = math.max(0.2, math.min(1.0, state.throttle * target / current))
    control.set_throttle(state.throttle)
  end
end

function on_start()
  control.set_throttle(1.0)
  control.set_sas("stability")
  control.stage()
  log.info("Ignition. Beginning automated gravity turn.")
end

function on_restore(restored)
  state.throttle = state.throttle or 1.0
end

function on_fixed_update(dt)
  state.throttle = state.throttle or 1.0
  if state.phase < 3 then
    regulate_twr(3.2)
  elseif state.phase == 4 then
    regulate_twr(3.0)
  end

  if not state.boosters_dropped and (resources.solid_fuel() < 5 or flight.apoapsis() > 90000) then
    control.stage()
    state.boosters_dropped = true
    log.info("Radial boosters clear")
  end

  if state.phase == 0 and flight.altitude() > 1200 then
    control.set_sas("off")
    control.set_rotation(0.15, 0.0, 0.0)
    state.phase = 1
  elseif state.phase == 1 and flight.altitude() > 7000 then
    control.set_rotation(0.0, 0.0, 0.0)
    control.set_sas("prograde")
    state.phase = 2
  elseif state.phase == 2 and state.boosters_dropped and flight.apoapsis() > 90000 then
    control.set_throttle(0.0)
    state.phase = 3
    log.info("Coasting to apoapsis")
  elseif state.phase == 3 and flight.altitude() > 70000 and flight.vertical_speed() < 120 then
    control.stage()
    control.set_sas("prograde")
    state.throttle = 1.0
    control.set_throttle(1.0)
    state.phase = 4
    log.info("Upper-stage circularization burn")
  elseif state.phase == 4 and flight.periapsis() > 76000 then
    control.set_throttle(0.0)
    state.phase = 5
    log.info("Orbit established")
  end
end
"#;

pub const COROUTINE_EXAMPLE: &str = r#"-- Coroutine style: yields while conditions are false.
state = state or { launches = 0 }

function main()
  state.launches = state.launches + 1
  control.set_throttle(1.0)
  control.set_sas("stability")
  control.stage()
  wait.until_condition(function() return flight.altitude() > 1200 end)
  control.set_sas("off")
  control.set_rotation(0.15, 0.0, 0.0)
  wait.until_condition(function() return flight.altitude() > 7000 end)
  control.set_rotation(0.0, 0.0, 0.0)
  control.set_sas("prograde")
  wait.until_condition(function() return resources.solid_fuel() < 5 or flight.apoapsis() > 90000 end)
  control.stage()
  wait.until_condition(function() return flight.apoapsis() > 90000 end)
  control.set_throttle(0.0)
  log.info("Target apoapsis reached")
end
"#;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ScriptCommands {
    pub throttle: Option<f64>,
    pub pitch: Option<f64>,
    pub yaw: Option<f64>,
    pub roll: Option<f64>,
    pub sas: Option<Option<SasMode>>,
    pub rcs: Option<bool>,
    pub stage: bool,
    pub deploy_parachutes: bool,
    pub warp_rate: Option<f64>,
    pub maneuver: Option<(f64, f64, f64, f64)>,
}

impl ScriptCommands {
    fn merge(self, newer: Self) -> Self {
        Self {
            throttle: newer.throttle.or(self.throttle),
            pitch: newer.pitch.or(self.pitch),
            yaw: newer.yaw.or(self.yaw),
            roll: newer.roll.or(self.roll),
            sas: newer.sas.or(self.sas),
            rcs: newer.rcs.or(self.rcs),
            stage: self.stage || newer.stage,
            deploy_parachutes: self.deploy_parachutes || newer.deploy_parachutes,
            warp_rate: newer.warp_rate.or(self.warp_rate),
            maneuver: newer.maneuver.or(self.maneuver),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScriptMode {
    Callback,
    Coroutine,
}

#[derive(Resource)]
pub struct ScriptRuntime {
    lua: Option<Lua>,
    commands: Arc<Mutex<ScriptCommands>>,
    logs: Arc<Mutex<Vec<String>>>,
    instruction_counter: Arc<AtomicUsize>,
    pub source: String,
    pub active: bool,
    pub mode: Option<ScriptMode>,
    pub last_error: Option<String>,
}

impl Default for ScriptRuntime {
    fn default() -> Self {
        Self {
            lua: None,
            commands: Arc::new(Mutex::new(ScriptCommands::default())),
            logs: Arc::new(Mutex::new(Vec::new())),
            instruction_counter: Arc::new(AtomicUsize::new(0)),
            source: EXAMPLE_SCRIPT.into(),
            active: false,
            mode: None,
            last_error: None,
        }
    }
}

impl ScriptRuntime {
    pub fn load(
        &mut self,
        source: String,
        restored_state: Option<serde_json::Value>,
    ) -> mlua::Result<()> {
        let lua = Lua::new();
        lua.set_memory_limit(8 * 1024 * 1024)?;
        let counter = self.instruction_counter.clone();
        lua.set_hook(
            HookTriggers::new().every_nth_instruction(1_000),
            move |_, _| {
                if counter.fetch_add(1_000, Ordering::Relaxed) >= 100_000 {
                    Err(mlua::Error::RuntimeError(
                        "script exceeded 100,000 instructions this tick".into(),
                    ))
                } else {
                    Ok(VmState::Continue)
                }
            },
        )?;
        self.install_api(&lua)?;
        lua.load(&source).set_name("vessel_script").exec()?;

        let globals = lua.globals();
        let has_main = globals.get::<Option<Function>>("main")?.is_some();
        let has_update = globals
            .get::<Option<Function>>("on_fixed_update")?
            .is_some();
        if has_main && has_update {
            return Err(mlua::Error::RuntimeError(
                "define either main() or on_fixed_update(), not both".into(),
            ));
        }
        let mode = if has_main {
            ScriptMode::Coroutine
        } else {
            ScriptMode::Callback
        };
        if let Some(state) = restored_state {
            let value = lua.to_value(&state)?;
            globals.set("state", value)?;
            if let Some(restore) = globals.get::<Option<Function>>("on_restore")? {
                restore.call::<()>(globals.get::<Value>("state")?)?;
            }
        } else if globals.get::<Option<Table>>("state")?.is_none() {
            globals.set("state", lua.create_table()?)?;
        }
        if mode == ScriptMode::Coroutine {
            let main = globals.get::<Function>("main")?;
            let thread = lua.create_thread(main)?;
            globals.set("__mission_thread", thread)?;
        } else if let Some(start) = globals.get::<Option<Function>>("on_start")? {
            start.call::<()>(())?;
        }
        drop(globals);
        self.lua = Some(lua);
        self.source = source;
        self.mode = Some(mode);
        self.active = true;
        self.last_error = None;
        self.push_log(format!(
            "Script started in {} mode",
            if mode == ScriptMode::Callback {
                "callback"
            } else {
                "coroutine"
            }
        ));
        Ok(())
    }

    fn install_api(&self, lua: &Lua) -> mlua::Result<()> {
        let globals = lua.globals();
        for name in [
            "os", "io", "package", "debug", "dofile", "loadfile", "require", "load",
        ] {
            globals.set(name, Value::Nil)?;
        }

        let flight = lua.create_table()?;
        for key in [
            "time",
            "altitude",
            "radar_altitude",
            "speed",
            "surface_speed",
            "vertical_speed",
            "mach",
            "dynamic_pressure",
            "heating",
            "apoapsis",
            "periapsis",
        ] {
            let key_owned = key.to_string();
            flight.set(
                key,
                lua.create_function(move |lua, ()| {
                    let telemetry = lua.globals().get::<Table>("__telemetry")?;
                    telemetry.get::<f64>(key_owned.as_str())
                })?,
            )?;
        }
        globals.set("flight", flight)?;

        let resources = lua.create_table()?;
        for key in [
            "mass",
            "liquid_fuel",
            "solid_fuel",
            "monopropellant",
            "thrust",
            "twr",
        ] {
            let key_owned = key.to_string();
            resources.set(
                key,
                lua.create_function(move |lua, ()| {
                    let telemetry = lua.globals().get::<Table>("__telemetry")?;
                    telemetry.get::<f64>(key_owned.as_str())
                })?,
            )?;
        }
        globals.set("resources", resources)?;

        let control = lua.create_table()?;
        let commands = self.commands.clone();
        control.set(
            "set_throttle",
            lua.create_function(move |_, value: f64| {
                commands.lock().unwrap().throttle = Some(value.clamp(0.0, 1.0));
                Ok(())
            })?,
        )?;
        let commands = self.commands.clone();
        control.set(
            "set_rotation",
            lua.create_function(move |_, (pitch, yaw, roll): (f64, f64, f64)| {
                let mut cmd = commands.lock().unwrap();
                cmd.pitch = Some(pitch.clamp(-1.0, 1.0));
                cmd.yaw = Some(yaw.clamp(-1.0, 1.0));
                cmd.roll = Some(roll.clamp(-1.0, 1.0));
                Ok(())
            })?,
        )?;
        let commands = self.commands.clone();
        control.set(
            "set_sas",
            lua.create_function(move |_, mode: String| {
                let mode = match mode.to_ascii_lowercase().as_str() {
                    "off" => None,
                    "prograde" => Some(SasMode::Prograde),
                    "retrograde" => Some(SasMode::Retrograde),
                    "normal" => Some(SasMode::Normal),
                    "antinormal" | "anti_normal" => Some(SasMode::AntiNormal),
                    "radial_in" => Some(SasMode::RadialIn),
                    "radial_out" => Some(SasMode::RadialOut),
                    "maneuver" => Some(SasMode::Maneuver),
                    _ => Some(SasMode::Stability),
                };
                commands.lock().unwrap().sas = Some(mode);
                Ok(())
            })?,
        )?;
        let commands = self.commands.clone();
        control.set(
            "set_rcs",
            lua.create_function(move |_, enabled: bool| {
                commands.lock().unwrap().rcs = Some(enabled);
                Ok(())
            })?,
        )?;
        let commands = self.commands.clone();
        control.set(
            "stage",
            lua.create_function(move |_, ()| {
                commands.lock().unwrap().stage = true;
                Ok(())
            })?,
        )?;
        let commands = self.commands.clone();
        control.set(
            "deploy_parachutes",
            lua.create_function(move |_, ()| {
                commands.lock().unwrap().deploy_parachutes = true;
                Ok(())
            })?,
        )?;
        globals.set("control", control)?;

        let nav = lua.create_table()?;
        let commands = self.commands.clone();
        nav.set(
            "set_warp",
            lua.create_function(move |_, rate: f64| {
                commands.lock().unwrap().warp_rate = Some(rate);
                Ok(())
            })?,
        )?;
        let commands = self.commands.clone();
        nav.set(
            "set_maneuver",
            lua.create_function(
                move |_, (ut, prograde, normal, radial): (f64, f64, f64, f64)| {
                    commands.lock().unwrap().maneuver = Some((ut, prograde, normal, radial));
                    Ok(())
                },
            )?,
        )?;
        globals.set("nav", nav)?;

        let logs = self.logs.clone();
        let log = lua.create_table()?;
        log.set(
            "info",
            lua.create_function(move |_, message: String| {
                logs.lock().unwrap().push(message);
                Ok(())
            })?,
        )?;
        globals.set("log", log)?;

        lua.load(
            r#"
            wait = {}
            function wait.seconds(seconds)
              local target = flight.time() + seconds
              while flight.time() < target do coroutine.yield() end
            end
            function wait.until_condition(predicate)
              while not predicate() do coroutine.yield() end
            end
            function wait.event(name)
              while __last_event ~= name do coroutine.yield() end
              __last_event = nil
            end
        "#,
        )
        .exec()?;
        Ok(())
    }

    pub fn tick(&mut self, telemetry: &FlightTelemetry) -> ScriptCommands {
        if !self.active {
            return ScriptCommands::default();
        }
        self.instruction_counter.store(0, Ordering::Relaxed);
        let pending = std::mem::take(&mut *self.commands.lock().unwrap());
        let result = self.tick_inner(telemetry);
        if let Err(error) = result {
            self.active = false;
            self.last_error = Some(error.to_string());
            self.push_log(format!("ERROR: {error}"));
        }
        let generated = std::mem::take(&mut *self.commands.lock().unwrap());
        pending.merge(generated)
    }

    fn tick_inner(&mut self, telemetry: &FlightTelemetry) -> mlua::Result<()> {
        let Some(lua) = &self.lua else { return Ok(()) };
        let globals = lua.globals();
        let table = lua.create_table()?;
        for (key, value) in [
            ("time", telemetry.ut),
            ("altitude", telemetry.altitude),
            ("radar_altitude", telemetry.radar_altitude),
            ("speed", telemetry.speed),
            ("surface_speed", telemetry.surface_speed),
            ("vertical_speed", telemetry.vertical_speed),
            ("mach", telemetry.mach),
            ("dynamic_pressure", telemetry.dynamic_pressure),
            ("heating", telemetry.heating),
            ("apoapsis", telemetry.orbit.apoapsis),
            ("periapsis", telemetry.orbit.periapsis),
            ("mass", telemetry.mass),
            ("liquid_fuel", telemetry.liquid_fuel),
            ("solid_fuel", telemetry.solid_fuel),
            ("monopropellant", telemetry.monopropellant),
            ("thrust", telemetry.thrust),
            ("twr", telemetry.twr),
        ] {
            table.set(key, value)?;
        }
        globals.set("__telemetry", table)?;
        match self.mode {
            Some(ScriptMode::Callback) => {
                if let Some(update) = globals.get::<Option<Function>>("on_fixed_update")? {
                    update.call::<()>(1.0 / 60.0)?;
                }
            }
            Some(ScriptMode::Coroutine) => {
                let thread = globals.get::<Thread>("__mission_thread")?;
                if thread.status() == ThreadStatus::Resumable {
                    thread.resume::<()>(())?;
                } else {
                    self.active = false;
                }
            }
            None => {}
        }
        Ok(())
    }

    pub fn snapshot_state(&self) -> Option<serde_json::Value> {
        let lua = self.lua.as_ref()?;
        let value = lua.globals().get::<Value>("state").ok()?;
        lua.from_value(value).ok()
    }

    pub fn emit_event(&mut self, event: &str) {
        let Some(lua) = &self.lua else { return };
        let globals = lua.globals();
        if globals.set("__last_event", event).is_err() {
            return;
        }
        if self.mode == Some(ScriptMode::Callback)
            && let Ok(Some(callback)) = globals.get::<Option<Function>>("on_event")
            && let Err(error) = callback.call::<()>(event)
        {
            self.last_error = Some(error.to_string());
            self.active = false;
        }
    }

    pub fn stop(&mut self) {
        self.active = false;
        self.push_log("Automation stopped".into());
    }

    pub fn drain_logs(&self) -> Vec<String> {
        let mut logs = self.logs.lock().unwrap();
        std::mem::take(&mut *logs)
    }

    fn push_log(&self, message: String) {
        let mut logs = self.logs.lock().unwrap();
        logs.push(message);
        if logs.len() > 200 {
            logs.remove(0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{HOME_ATMOSPHERE, PartCatalog, Vessel, stock_craft};
    use crate::orbit::OrbitalElements;
    use crate::simulation::{activate_next_stage, step_vessel, telemetry as flight_telemetry};

    fn telemetry(altitude: f64) -> FlightTelemetry {
        FlightTelemetry {
            altitude,
            orbit: OrbitalElements {
                apoapsis: altitude,
                ..Default::default()
            },
            ..Default::default()
        }
    }

    #[test]
    fn callback_can_command_throttle() {
        let mut runtime = ScriptRuntime::default();
        runtime
            .load(
                "function on_fixed_update(dt) control.set_throttle(0.75) end".into(),
                None,
            )
            .unwrap();
        let commands = runtime.tick(&telemetry(10.0));
        assert_eq!(commands.throttle, Some(0.75));
    }

    #[test]
    fn coroutine_yields_and_resumes() {
        let mut runtime = ScriptRuntime::default();
        runtime.load("function main() wait.until_condition(function() return flight.altitude() > 100 end); control.stage() end".into(), None).unwrap();
        assert!(!runtime.tick(&telemetry(10.0)).stage);
        assert!(runtime.tick(&telemetry(110.0)).stage);
    }

    #[test]
    fn unsafe_libraries_are_absent() {
        let mut runtime = ScriptRuntime::default();
        runtime.load("function on_fixed_update(dt) assert(os == nil and io == nil and require == nil) end".into(), None).unwrap();
        runtime.tick(&telemetry(0.0));
        assert!(runtime.active);
    }

    #[test]
    fn on_start_command_runs_once() {
        let mut runtime = ScriptRuntime::default();
        runtime
            .load("function on_start() control.stage() end".into(), None)
            .unwrap();
        assert!(runtime.tick(&telemetry(0.0)).stage);
        assert!(!runtime.tick(&telemetry(0.0)).stage);
    }

    #[test]
    fn serializable_state_restores_after_reload() {
        let source = "state = state or { phase = 0 }; function on_fixed_update(dt) state.phase = state.phase + 1 end";
        let mut runtime = ScriptRuntime::default();
        runtime.load(source.into(), None).unwrap();
        runtime.tick(&telemetry(0.0));
        let state = runtime.snapshot_state();
        runtime.load(source.into(), state).unwrap();
        runtime.tick(&telemetry(0.0));
        assert_eq!(runtime.snapshot_state().unwrap()["phase"], 2);
    }

    #[test]
    fn default_guidance_reaches_a_balanced_orbit_at_sixty_hertz() {
        let catalog = PartCatalog::default();
        let mut vessel = Vessel::from_blueprint(&stock_craft(), &catalog);
        let mut runtime = ScriptRuntime::default();
        runtime.load(EXAMPLE_SCRIPT.into(), None).unwrap();

        let dt = 1.0 / 60.0;
        let mut ut = 0.0;
        let mut current = flight_telemetry(&vessel, &catalog, ut, 0.0);
        let mut booster_separation = None;
        let mut core_fuel_at_separation = None;
        let mut orbit_time = None;
        let mut max_twr = 0.0_f64;
        let mut max_dynamic_pressure = 0.0_f64;

        for _ in 0..(300 * 60) {
            let before = flight_telemetry(&vessel, &catalog, ut, current.thrust);
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
            if commands.stage {
                let prior_stage = vessel.next_stage;
                activate_next_stage(&mut vessel, &catalog);
                if prior_stage == 1 && vessel.next_stage == 2 {
                    booster_separation = Some(ut);
                    core_fuel_at_separation = vessel
                        .parts
                        .iter()
                        .find(|part| part.instance.instance_id == 8)
                        .map(|part| part.fuel);
                }
            }

            current = step_vessel(&mut vessel, &catalog, dt, ut);
            ut += dt;
            max_twr = max_twr.max(current.twr);
            max_dynamic_pressure = max_dynamic_pressure.max(current.dynamic_pressure);
            if current.orbit.periapsis >= 75_000.0 {
                orbit_time.get_or_insert(ut);
            }
            if current.orbit.periapsis > 76_000.0 && vessel.controls.throttle == 0.0 {
                break;
            }
        }

        assert!(runtime.active, "guidance failed: {:?}", runtime.last_error);
        let separation = booster_separation.expect("boosters never separated");
        assert!(
            (65.0..=72.0).contains(&separation),
            "separated at {separation}"
        );
        assert!(
            core_fuel_at_separation.is_some_and(|fuel| fuel > 1_000.0),
            "core tank was empty at booster separation: {core_fuel_at_separation:?}"
        );
        assert!(max_twr <= 3.3, "maximum TWR was {max_twr}");
        assert!(
            (35_000.0..=55_000.0).contains(&max_dynamic_pressure),
            "maximum dynamic pressure was {max_dynamic_pressure} Pa"
        );
        let orbit_time = orbit_time.expect("guidance did not reach orbit");
        assert!(orbit_time < 300.0, "orbit took {orbit_time} seconds");
        assert!(current.orbit.periapsis >= 75_000.0);
        assert!(
            current.orbit.apoapsis <= 120_000.0,
            "apoapsis was {}",
            current.orbit.apoapsis
        );
        let upper_fuel = vessel
            .parts
            .iter()
            .find(|part| part.instance.instance_id == 4)
            .map_or(0.0, |part| part.fuel);
        assert!(upper_fuel > 0.0, "upper stage exhausted its tank");
        assert!(current.altitude > HOME_ATMOSPHERE);
    }
}
