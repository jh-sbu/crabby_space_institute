-- Coroutine style: yields while conditions are false.
state = state or { launches = 0 }

function main()
  state.launches = state.launches + 1
  control.set_throttle(1.0)
  control.set_sas("stability")
  control.stage()
  wait.until_condition(function() return flight.altitude() > 1200 end)
  control.set_sas("off")
  control.set_rotation(0.30, 0.0, 0.0)
  wait.until_condition(function() return flight.altitude() > 7000 end)
  control.set_rotation(0.0, 0.0, 0.0)
  control.set_sas("prograde")
  wait.until_condition(function() return resources.solid_fuel() < 5 or flight.apoapsis() > 90000 end)
  control.stage()
  wait.until_condition(function() return flight.apoapsis() > 90000 end)
  control.set_throttle(0.0)
  log.info("Target apoapsis reached")
end
