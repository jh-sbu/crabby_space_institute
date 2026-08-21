-- Test/example fixture. It may intentionally drift from assets/scripts/guided_ascent.lua.
-- Crabby Space Institute: orbit insertion (callback style)
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
  if state.phase == 1 then
    control.set_sas("off")
    control.set_rotation(0.30, 0.0, 0.0)
  else
    control.set_rotation(0.0, 0.0, 0.0)
  end

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
    control.set_rotation(0.30, 0.0, 0.0)
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
