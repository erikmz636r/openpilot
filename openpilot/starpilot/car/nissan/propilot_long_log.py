"""Copy a Rogue ProPILOT observation into starpilotCarState.

Logging only. The caller does not send CAN.
"""


def fill_propilot_long(state, obs) -> None:
  msg = state.propilotLong
  msg.valid = bool(obs.valid)
  msg.driveTorque = int(obs.drive_torque)
  msg.driveTorqueInverseOk = bool(obs.drive_torque_inverse_ok)
  msg.driveTorqueSmooth = int(obs.drive_torque_smooth) & 0xFF
  msg.byte4 = int(obs.byte4) & 0xFF
  msg.engaged = bool(obs.engaged)
  msg.brakePressure = int(obs.brake_pressure)
  msg.brakeActive = bool(obs.brake_active)
  msg.brakeDecel = float(obs.brake_decel)
  msg.statusByte0 = int(obs.status_byte0) & 0xFF
  msg.statusByte4 = int(obs.status_byte4) & 0xFF
  msg.statusEngaged = bool(obs.status_engaged)
  msg.armed = bool(obs.armed)
  msg.resPressed = bool(obs.res_pressed)
  msg.pedalRequest = int(obs.pedal_request)
  msg.pedalState = int(obs.pedal_state) & 0xFF
  msg.pedalEngaged = bool(obs.pedal_engaged)
  msg.lead0 = int(obs.lead[0]) & 0xFF
  msg.lead1 = int(obs.lead[1]) & 0xFF
  msg.lead2 = int(obs.lead[2]) & 0xFF
  msg.lead3 = int(obs.lead[3]) & 0xFF
  msg.seenTorque = bool(obs.seen_torque)
  msg.seenBrake = bool(obs.seen_brake)
  msg.seenStatus = bool(obs.seen_status)
  msg.seenPedal = bool(obs.seen_pedal)
  msg.seenLead = bool(obs.seen_lead)
