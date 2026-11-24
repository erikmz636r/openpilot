import copy
from collections import deque
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.nissan.propilot_long import observe
from opendbc.car.nissan.values import CAR, DBC, CarControllerParams

ButtonType = structs.CarState.ButtonEvent.Type

TORQUE_SAMPLES = 12


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    self.lkas_hud_msg = {}
    self.lkas_hud_info_msg = {}

    self.steeringTorqueSamples = deque(TORQUE_SAMPLES*[0], TORQUE_SAMPLES)
    self.shifter_values = can_define.dv["GEARBOX"]["GEAR_SHIFTER"]

    self.distance_button = 0
    self.lkas_setting = None
    # Read-only ProPILOT longitudinal sample. None except on NISSAN_ROGUE.
    self.propilot_long = None

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_adas = can_parsers[Bus.adas]

    ret = structs.CarState()

    prev_distance_button = self.distance_button
    self.distance_button = cp.vl["CRUISE_THROTTLE"]["FOLLOW_DISTANCE_BUTTON"]

    if self.CP.carFingerprint in (CAR.NISSAN_ROGUE, CAR.NISSAN_XTRAIL, CAR.NISSAN_ALTIMA):
      ret.gasPressed = bool(cp.vl["GAS_PEDAL"]["GAS_PEDAL"] > 3)
    elif self.CP.carFingerprint in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC):
      ret.gasPressed = bool(cp.vl["CRUISE_THROTTLE"]["GAS_PEDAL"] > 3)

    if self.CP.carFingerprint in (CAR.NISSAN_ROGUE, CAR.NISSAN_XTRAIL, CAR.NISSAN_ALTIMA):
      ret.brakePressed = bool(cp.vl["DOORS_LIGHTS"]["USER_BRAKE_PRESSED"])
    elif self.CP.carFingerprint in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC):
      ret.brakePressed = bool(cp.vl["CRUISE_THROTTLE"]["USER_BRAKE_PRESSED"])

    fl = cp.vl["WHEEL_SPEEDS_FRONT"]["WHEEL_SPEED_FL"] * CV.KPH_TO_MS
    fr = cp.vl["WHEEL_SPEEDS_FRONT"]["WHEEL_SPEED_FR"] * CV.KPH_TO_MS
    rl = cp.vl["WHEEL_SPEEDS_REAR"]["WHEEL_SPEED_RL"] * CV.KPH_TO_MS
    rr = cp.vl["WHEEL_SPEEDS_REAR"]["WHEEL_SPEED_RR"] * CV.KPH_TO_MS
    # safety uses the rear wheel speeds for the speed measurement and angle limiting
    ret.vEgoRaw = (rl + rr) / 2.0

    v_ego_raw_full = (fl + fr + rl + rr) / 4.0
    ret.vEgo, ret.aEgo = self.update_speed_kf(v_ego_raw_full)
    ret.standstill = cp.vl["WHEEL_SPEEDS_REAR"]["WHEEL_SPEED_RL"] == 0.0 and cp.vl["WHEEL_SPEEDS_REAR"]["WHEEL_SPEED_RR"] == 0.0

    if self.CP.carFingerprint == CAR.NISSAN_ALTIMA:
      ret.cruiseState.enabled = bool(cp.vl["CRUISE_STATE"]["CRUISE_ENABLED"])
    else:
      ret.cruiseState.enabled = bool(cp_adas.vl["CRUISE_STATE"]["CRUISE_ENABLED"])

    if self.CP.carFingerprint in (CAR.NISSAN_ROGUE, CAR.NISSAN_XTRAIL):
      ret.seatbeltUnlatched = cp.vl["HUD"]["SEATBELT_DRIVER_LATCHED"] == 0
      ret.cruiseState.available = bool(cp_cam.vl["PRO_PILOT"]["CRUISE_ON"])
    elif self.CP.carFingerprint in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC):
      if self.CP.carFingerprint == CAR.NISSAN_LEAF:
        ret.seatbeltUnlatched = cp.vl["SEATBELT"]["SEATBELT_DRIVER_LATCHED"] == 0
      elif self.CP.carFingerprint == CAR.NISSAN_LEAF_IC:
        ret.seatbeltUnlatched = cp.vl["CANCEL_MSG"]["CANCEL_SEATBELT"] == 1
      ret.cruiseState.available = bool(cp.vl["CRUISE_THROTTLE"]["CRUISE_AVAILABLE"])
    elif self.CP.carFingerprint == CAR.NISSAN_ALTIMA:
      ret.seatbeltUnlatched = cp.vl["HUD"]["SEATBELT_DRIVER_LATCHED"] == 0
      ret.cruiseState.available = bool(cp_adas.vl["PRO_PILOT"]["CRUISE_ON"])

    if self.CP.carFingerprint == CAR.NISSAN_ALTIMA:
      speed = cp.vl["PROPILOT_HUD"]["SET_SPEED"]
    else:
      speed = cp_adas.vl["PROPILOT_HUD"]["SET_SPEED"]

    if speed != 255:
      if self.CP.carFingerprint in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC):
        conversion = CV.MPH_TO_MS if cp.vl["HUD_SETTINGS"]["SPEED_MPH"] else CV.KPH_TO_MS
      else:
        conversion = CV.MPH_TO_MS if cp.vl["HUD"]["SPEED_MPH"] else CV.KPH_TO_MS
      ret.cruiseState.speed = speed * conversion
      ret.cruiseState.speedCluster = (speed - 1) * conversion  # Speed on HUD is always 1 lower than actually sent on can bus

    # Altima EPS faults when user overrides above a certain torque.
    # It also faults on sharp curves, although the threshold is unknown at this time
    if self.CP.carFingerprint == CAR.NISSAN_ALTIMA:
      ret.steeringTorque = cp_cam.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
      ret.steerFaultTemporary = cp_cam.vl["STEER_TORQUE_SENSOR"]["LKAS_STATUS"] == 9
      ret.steeringAngleDeg = cp_cam.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    else:
      ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
      ret.steerFaultTemporary = cp.vl["STEER_TORQUE_SENSOR"]["LKAS_STATUS"] == 9
      ret.steeringAngleDeg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]

    self.steeringTorqueSamples.append(ret.steeringTorque)
    # Filtering driver torque to prevent steeringPressed false positives
    ret.steeringPressed = bool(abs(sum(self.steeringTorqueSamples) / TORQUE_SAMPLES) > CarControllerParams.STEER_THRESHOLD)

    ret.leftBlinker = bool(cp.vl["LIGHTS"]["LEFT_BLINKER"])
    ret.rightBlinker = bool(cp.vl["LIGHTS"]["RIGHT_BLINKER"])

    ret.doorOpen = any([cp.vl["DOORS_LIGHTS"]["DOOR_OPEN_RR"],
                        cp.vl["DOORS_LIGHTS"]["DOOR_OPEN_RL"],
                        cp.vl["DOORS_LIGHTS"]["DOOR_OPEN_FR"],
                        cp.vl["DOORS_LIGHTS"]["DOOR_OPEN_FL"]])

    ret.espDisabled = bool(cp.vl["ESP"]["ESP_DISABLED"])

    can_gear = int(cp.vl["GEARBOX"]["GEAR_SHIFTER"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # stock lkas should be off
    # TODO: is this needed?
    if self.CP.carFingerprint == CAR.NISSAN_ALTIMA:
      ret.invalidLkasSetting = bool(cp.vl["LKAS_SETTINGS"]["LKAS_ENABLED"])
    else:
      ret.invalidLkasSetting = bool(cp_adas.vl["LKAS_SETTINGS"]["LKAS_ENABLED"])

    self.cruise_throttle_msg = copy.copy(cp.vl["CRUISE_THROTTLE"])

    if self.CP.carFingerprint in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC):
      self.cancel_msg = copy.copy(cp.vl["CANCEL_MSG"])

    if self.CP.carFingerprint != CAR.NISSAN_ALTIMA:
      self.lkas_hud_msg = copy.copy(cp_adas.vl["PROPILOT_HUD"])
      self.lkas_hud_info_msg = copy.copy(cp_adas.vl["PROPILOT_HUD_INFO_MSG"])

    button_events = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    # LKAS setting transitions are Dom's wheel input, not a separate physical switch.
    # Seed only a received value; startup/absent defaults cannot synthesize a press.
    source = cp if self.CP.carFingerprint == CAR.NISSAN_ALTIMA else cp_adas
    if source.ts_nanos["LKAS_SETTINGS"]["LKAS_ENABLED"] > 0:
      if self.lkas_setting is not None:
        button_events += create_button_events(ret.invalidLkasSetting, self.lkas_setting, {1: ButtonType.lkas})
      self.lkas_setting = ret.invalidLkasSetting

    ret.buttonEvents = button_events

    # Decode only. This does not change cruise, torque, or the returned CarState
    # fields that controlsd already consumes.
    if self.CP.carFingerprint == CAR.NISSAN_ROGUE:
      self.propilot_long = self._propilot_long(cp, cp_cam, cp_adas)
    else:
      self.propilot_long = None
    return ret

  @staticmethod
  def _seen(parser, message: str, signal: str) -> bool:
    return parser.ts_nanos[message][signal] > 0

  def _propilot_long(self, cp, cp_cam, cp_adas):
    # Panda buses on Rogue (not Altima): pt=0, cam=1, adas=2.
    # 0x1B6 / 0x1C3 / 0x2D4 are on bus 1, 0x15A on bus 0, 0x4DF on bus 2.
    torque = cp_cam.vl["PRO_PILOT"]
    brake = cp_cam.vl["PROPILOT_BRAKE"]
    status = cp_cam.vl["PROPILOT_LONG_STATUS"]
    pedal = cp.vl["PROPILOT_PEDAL_REQUEST"]
    lead = cp_adas.vl["PROPILOT_CAMERA_LEAD"]
    return observe(
      torque=int(round(torque["PROPILOT_DRIVE_TORQUE"])),
      torque_inverse=int(round(torque["PROPILOT_DRIVE_TORQUE_INV"])),
      smooth=int(round(torque["PROPILOT_DRIVE_TORQUE_SMOOTH"])),
      byte4=int(round(torque["PROPILOT_BYTE4"])),
      brake_pressure=int(round(brake["BRAKE_PRESSURE"])),
      brake_active=bool(brake["BRAKE_ACTIVE"]),
      status_byte0=int(round(status["STATUS_BYTE0"])),
      status_byte4=int(round(status["STATUS_BYTE4"])),
      # Same CRUISE_THROTTLE frame as the distance button. Do not turn this into a button event.
      res_pressed=bool(cp.vl["CRUISE_THROTTLE"]["RES_BUTTON"]),
      pedal_request=int(round(pedal["PEDAL_REQUEST"])),
      pedal_state=int(round(pedal["ENGAGED_STATE"])),
      lead=(
        int(round(lead["LEAD_BYTE0"])),
        int(round(lead["LEAD_BYTE1"])),
        int(round(lead["LEAD_BYTE2"])),
        int(round(lead["LEAD_BYTE3"])),
      ),
      seen_torque=self._seen(cp_cam, "PRO_PILOT", "PROPILOT_DRIVE_TORQUE"),
      seen_brake=self._seen(cp_cam, "PROPILOT_BRAKE", "BRAKE_PRESSURE"),
      seen_status=self._seen(cp_cam, "PROPILOT_LONG_STATUS", "STATUS_BYTE0"),
      seen_pedal=self._seen(cp, "PROPILOT_PEDAL_REQUEST", "PEDAL_REQUEST"),
      seen_lead=self._seen(cp_adas, "PROPILOT_CAMERA_LEAD", "LEAD_BYTE0"),
    )

  @staticmethod
  def get_can_parsers(CP):
    parsers = {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1 if CP.carFingerprint == CAR.NISSAN_ALTIMA else 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0 if CP.carFingerprint == CAR.NISSAN_ALTIMA else 1),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
    # NaN rate: these RX messages must not change canValid or bus timeout.
    if CP.carFingerprint == CAR.NISSAN_ROGUE:
      debug = (
        (parsers[Bus.cam], ("PROPILOT_BRAKE", "PROPILOT_LONG_STATUS")),
        (parsers[Bus.pt], ("PROPILOT_PEDAL_REQUEST",)),
        (parsers[Bus.adas], ("PROPILOT_CAMERA_LEAD",)),
      )
      for parser, names in debug:
        for name in names:
          parser._add_message(name, float("nan"))
    return parsers
