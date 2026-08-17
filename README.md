# Crabby Space Institute

A playable 3D spaceflight sandbox built in Rust with Bevy. Design staged rockets, fly through a simulated atmosphere, plan orbital maneuvers, cross patched-conic spheres of influence, return through reentry heating, and automate the flight computer with sandboxed Lua.

## Run

```bash
cargo run --release
```

The first build is large because Bevy and the vendored Lua 5.4 runtime compile locally. Native keyboard and mouse desktop builds are supported on Linux, Windows, and macOS.

## Flight controls

| Input | Action |
|---|---|
| W / S | Pitch |
| A / D | Yaw |
| Q / E | Roll |
| Shift / Ctrl | Increase / decrease throttle |
| Z / X | Full / zero throttle |
| Space | Activate next stage |
| T / R | Toggle SAS / RCS |
| M / C | Map / camera mode |
| , / . | Decrease / increase time warp |
| F5 / F9 | Quicksave / quickload |
| F8 | Stop Lua automation immediately |

The stock `Crabitat Pathfinder` is ready to launch. The optional guided mission asks you to clear the tower, stage, establish a 75 km orbit around Carapace, reenter, and recover the command pod.

## Lua flight computer

Open the Lua editor in assembly or flight. A script uses either callbacks:

```lua
state = state or { phase = 0 }

function on_fixed_update(dt)
  control.set_throttle(1.0)
  if flight.altitude() > 10000 then
    control.set_sas("prograde")
  end
end
```

or a yielding mission coroutine:

```lua
function main()
  control.set_throttle(1.0)
  control.stage()
  wait.until_condition(function() return flight.apoapsis() > 80000 end)
  control.set_throttle(0.0)
end
```

Available modules are:

- `flight`: time, altitude, speed, vertical speed, Mach, dynamic pressure, heat flux, apoapsis, and periapsis.
- `resources`: mass, liquid/solid/monopropellant levels, thrust, and TWR.
- `control`: throttle, rotation, SAS, RCS, staging, and parachutes.
- `nav`: time warp and maneuver-node creation.
- `wait` and `log`: coroutine scheduling and the in-game console.

Lua cannot access files, networking, processes, dynamic modules, or debug facilities. Each VM is limited to 8 MiB and 100,000 instructions per simulation tick. The serializable global `state` table survives quickload; coroutine stacks restart by design.

## Development

```bash
cargo fmt --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-targets
```

The simulation library uses `f64` state vectors independently of rendering. Active craft run fixed-step compound-rigid-body physics; safe high warp uses universal-variable Kepler propagation and transforms state continuously when entering or leaving a body's sphere of influence.

### Development MCP

An optional, local-only MCP server can inspect and mutate live game state for debugging:

```bash
cargo run --features mcp
```

While that build is running, connect an MCP client to `http://127.0.0.1:8765/mcp`. Set `CRABBY_MCP_ADDR` to choose another loopback address. The server refuses non-loopback binds.

The MCP exposes two complementary debugging surfaces:

- `inspect_game_state` and `patch_game_state` inspect or atomically merge-patch simulation state. Read before patching, patch only intended fields, and pause the clock before editing coupled flight values.
- `inspect_player_state` plus `menu_action`, `assembly_action`, `set_flight_controls`, `flight_action`, `script_action`, and `view_action` drive the same validation and gameplay paths as player input. Use these semantic actions for normal live testing.

For example, this sequence enters Vehicle Assembly, launches the stock craft through normal validation, and produces a physics-driven liftoff:

```json
{"tool":"menu_action","arguments":{"action":"open_vehicle_assembly"}}
{"tool":"assembly_action","arguments":{"action":"launch"}}
{"tool":"set_flight_controls","arguments":{"throttle":1.0}}
{"tool":"flight_action","arguments":{"action":"activate_next_stage"}}
```

Supplied MCP pitch, yaw, and roll axes remain latched across frames so a client can emulate held flight keys. Call `flight_action` with `{"action":"release_attitude_controls"}` to release all three axes back to the physical keyboard; they are also released when leaving flight, launching a vessel, or loading a quicksave. Throttle remains set, matching the player's latched throttle control. Action tools reject invalid modes, identifiers, and ranges without bypassing gameplay validation. `Quit` is intentionally not exposed because it would sever the debugging connection.

The MCP code and its networking dependencies are excluded unless the `mcp` feature is enabled.
