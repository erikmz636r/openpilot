import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.carlog import carlog
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.nissan import nissancan
from opendbc.car.nissan.values import CAR, CarControllerParams
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.nissan.icbm import MODE_ACTIVE, CarInputs, IcbmController, make_throttle_values

VisualAlert = structs.CarControl.HUDControl.VisualAlert


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.car_fingerprint = CP.carFingerprint

    self.angle_filter = FirstOrderFilter(0.0, 0.1, DT_CTRL)

    self.apply_angle_last = 0

    self.packer = CANPacker(dbc_names[Bus.pt])

    # Rogue cruise-button management. Off unless the StarPilot setting turns it on at startup
    # (starpilot/car/nissan/icbm.py); Active also needs the panda flag set at the same time.
    self.icbm = IcbmController() if CP.carFingerprint == CAR.NISSAN_ROGUE else None

  def _icbm_update(self, CC, CS, now_nanos, pcm_cancel_cmd):
    propilot = getattr(CS, "propilot_long", None)
    button = self.icbm.update(CarInputs(
      now_ns=int(now_nanos),
      op_enabled=bool(CC.enabled),
      cruise_enabled=bool(CS.out.cruiseState.enabled),
      propilot_engaged=bool(propilot is not None and propilot.valid and propilot.engaged),
      v_ego=float(CS.out.vEgo),
      standstill=bool(CS.out.standstill),
      gas_pressed=bool(CS.out.gasPressed),
      brake_pressed=bool(CS.out.brakePressed),
      throttle=CS.cruise_throttle_msg,
      throttle_ns=int(CS.cruise_throttle_nanos),
      set_raw=int(CS.set_speed_raw),
      set_ns=int(CS.set_speed_nanos),
      speed_mph_units=bool(CS.speed_units_mph),
      pcm_cancel=bool(pcm_cancel_cmd),
    ))
    if button is None or self.icbm.mode != MODE_ACTIVE or pcm_cancel_cmd:
      return None
    return self.packer.make_can_msg("CRUISE_THROTTLE", 2, make_throttle_values(CS.cruise_throttle_msg, button))

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    ### STEER ###
    steer_hud_alert = 1 if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw) else 0

    # At low speeds and at high steering angles, EPS is sensitive to jitter in angle request. Smooth to fix uncomfortable response.
    if CC.latActive:
      self.angle_filter.update_alpha(float(np.interp(CS.out.vEgo, [5, 10, 20], [0.2, 0.1, 0.0])))
      self.angle_filter.update(actuators.steeringAngleDeg)
    else:
      self.angle_filter.x = actuators.steeringAngleDeg

    # windup slower
    self.apply_angle_last = apply_std_steer_angle_limits(self.angle_filter.x, self.apply_angle_last, CS.out.vEgoRaw,
                                                         CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)

    lkas_max_torque = 0
    if CC.latActive:
      # Max torque from driver before EPS will give up and not apply torque
      if not bool(CS.out.steeringPressed):
        lkas_max_torque = CarControllerParams.LKAS_MAX_TORQUE
      else:
        # Scale max torque based on how much torque the driver is applying to the wheel.
        # Start scaling torque at STEER_THRESHOLD down to 0.2. If we don't scale this low, EPS will temp fault from high driver and LKAS torque
        lkas_max_torque = max(
          CarControllerParams.LKAS_MIN_TORQUE,
          CarControllerParams.LKAS_MAX_TORQUE - 0.6 * max(0, abs(CS.out.steeringTorque) - CarControllerParams.STEER_THRESHOLD),
        )

    if self.CP.carFingerprint in (CAR.NISSAN_ROGUE, CAR.NISSAN_XTRAIL, CAR.NISSAN_ALTIMA) and pcm_cancel_cmd:
      can_sends.append(nissancan.create_acc_cancel_cmd(self.packer, self.car_fingerprint, CS.cruise_throttle_msg))

    if self.icbm is not None and self.icbm.mode != 0 and now_nanos is not None:
      try:
        icbm_send = self._icbm_update(CC, CS, now_nanos, pcm_cancel_cmd)
        if icbm_send is not None:
          can_sends.append(icbm_send)
      except Exception:
        # Never let cruise-button control stall steering: turn it off for this drive.
        carlog.exception("rogue icbm disabled after error in carcontroller")
        self.icbm.disable()

    # TODO: Find better way to cancel!
    # For some reason spamming the cancel button is unreliable on the Leaf
    # We now cancel by making propilot think the seatbelt is unlatched,
    # this generates a beep and a warning message every time you disengage
    if self.CP.carFingerprint in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC) and self.frame % 2 == 0:
      can_sends.append(nissancan.create_cancel_msg(self.packer, CS.cancel_msg, pcm_cancel_cmd))

    can_sends.append(nissancan.create_steering_control(
      self.packer, self.apply_angle_last, self.frame, CC.latActive, lkas_max_torque))

    # Below are the HUD messages. We copy the stock message and modify
    if self.CP.carFingerprint != CAR.NISSAN_ALTIMA:
      if self.frame % 2 == 0:
        can_sends.append(nissancan.create_lkas_hud_msg(self.packer, CS.lkas_hud_msg, CC.enabled, hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                       hud_control.leftLaneDepart, hud_control.rightLaneDepart))

      if self.frame % 50 == 0:
        can_sends.append(nissancan.create_lkas_hud_info_msg(
          self.packer, CS.lkas_hud_info_msg, steer_hud_alert
        ))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
