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
