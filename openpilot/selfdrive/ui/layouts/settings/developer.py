from collections.abc import Callable

from openpilot.common.params import Params, UnknownKeyName
from openpilot.starpilot.longitudinal.enhanced_stop_hold import KEY as HOLD_KEY, LABEL as HOLD_LABEL, DESCRIPTION as HOLD_DESCRIPTION
from openpilot.starpilot.car.hyundai.aol import ioniq6_alpha_long_available
from openpilot.starpilot.car.nissan.icbm_settings import ICBM_DESCRIPTION, ICBM_KEY, ICBM_TITLE, read_mode as read_icbm_mode
from openpilot.starpilot.car.ford.longitudinal_preferences import release_long_available as ford_release_long_available
from opendbc.car.volkswagen.meb_modes import MEB_CARS, settings_capable as meb_settings_capable
from openpilot.starpilot.car.volkswagen.preferences import stock_long_requested
from openpilot.selfdrive.ui.widgets.ssh_key import ssh_key_item
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import button_item, multiple_button_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult

# Description constants
DESCRIPTIONS = {
  'enable_adb': tr_noop(
    "ADB (Android Debug Bridge) allows connecting to your device over USB or over the network. " +
    "See https://docs.comma.ai/how-to/connect-to-comma for more info."
  ),
  'ssh_key': tr_noop(
    "Warning: This grants SSH access to all public keys in your GitHub settings. Never enter a GitHub username " +
    "other than your own. A comma employee will NEVER ask you to add their GitHub username."
  ),
  'alpha_longitudinal': tr_noop(
    "<b>WARNING: openpilot longitudinal control is in alpha for this car and may disable Automatic Emergency Braking (AEB).</b><br><br>" +
    "On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. " +
    "This saves a request for openpilot longitudinal control; an exact vehicle check and verified takeover are required at startup. " +
    "Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha. " +
    "Changing this setting will restart openpilot if the car is powered on."
  ),
  'cellular_disabled': tr_noop(
    "Hang up cellular data and don't redial. The modem stays registered and the SIM is untouched. " +
    "Remote access over cellular data is unavailable while this is enabled."
  ),
}


class DeveloperLayout(Widget):
  def __init__(self, on_open_arabesque: Callable | None = None, arabesque_available: Callable[[], bool] = lambda: False):
    super().__init__()
    self._params = Params()
    self._is_release = self._params.get_bool("IsReleaseBranch")

    # Build items and keep references for callbacks/state updates
    self._adb_toggle = toggle_item(
      lambda: tr("Enable ADB"),
      description=lambda: tr(DESCRIPTIONS["enable_adb"]),
      initial_state=self._params.get_bool("AdbEnabled"),
      callback=self._on_enable_adb,
      enabled=ui_state.is_offroad,
    )

    # SSH enable toggle + SSH key management
    self._ssh_toggle = toggle_item(
      lambda: tr("Enable SSH"),
      description="",
      initial_state=self._params.get_bool("SshEnabled"),
      callback=self._on_enable_ssh,
    )
    self._ssh_keys = ssh_key_item(lambda: tr("SSH Keys"), description=lambda: tr(DESCRIPTIONS["ssh_key"]))

    self._joystick_toggle = toggle_item(
      lambda: tr("Joystick Debug Mode"),
      description="",
      initial_state=self._params.get_bool("JoystickDebugMode"),
      callback=self._on_joystick_debug_mode,
      enabled=ui_state.is_offroad,
    )

    self._long_maneuver_toggle = toggle_item(
      lambda: tr("Longitudinal Maneuver Mode"),
      description="",
      initial_state=self._params.get_bool("LongitudinalManeuverMode"),
      callback=self._on_long_maneuver_mode,
    )

    self._lat_maneuver_toggle = toggle_item(
      lambda: tr("Lateral Maneuver Mode"),
      description="",
      initial_state=self._params.get_bool("LateralManeuverMode"),
      callback=self._on_lat_maneuver_mode,
    )

    self._alpha_long_toggle = toggle_item(
      lambda: tr("openpilot Longitudinal Control (Alpha)"),
      description=lambda: tr(DESCRIPTIONS["alpha_longitudinal"]),
      initial_state=self._params.get_bool("AlphaLongitudinalEnabled"),
      callback=self._on_alpha_long_enabled,
      enabled=lambda: ui_state.is_offroad(),
    )

    self._always_hold_toggle = toggle_item(
      lambda: tr(HOLD_LABEL), description=lambda: tr(HOLD_DESCRIPTION),
      initial_state=self._params.get_bool(HOLD_KEY), callback=self._on_always_hold, enabled=ui_state.is_offroad,
    )

    self._ui_debug_toggle = toggle_item(
      lambda: tr("UI Debug Mode"),
      description="",
      initial_state=self._params.get_bool("ShowDebugInfo"),
      callback=self._on_enable_ui_debug,
    )
    self._propilot_long_toggle = toggle_item(
      lambda: tr("ProPILOT Longitudinal Debug"),
      description=lambda: tr(
        "On a 2019 Rogue, show stock ProPILOT drive request, brake pressure, and engaged/armed state. "
        "Display only. openpilot does not send, change, or block any CAN message."
      ),
      initial_state=self._propilot_long_enabled(),
      callback=self._on_propilot_long_debug,
    )
    self._icbm_selector = multiple_button_item(
      lambda: tr(ICBM_TITLE),
      lambda: tr(ICBM_DESCRIPTION),
      [lambda: tr("Off"), lambda: tr("Watch Only"), lambda: tr("Active")],
      self._icbm_mode(), callback=self._on_icbm_mode,
    )
    self._icbm_selector.action_item.set_enabled(ui_state.is_offroad)
    self._on_enable_ui_debug(self._params.get_bool("ShowDebugInfo"))

    self._cellular_disabled_toggle = toggle_item(
      lambda: tr("Disable Cellular"),
      description=lambda: tr(DESCRIPTIONS["cellular_disabled"]),
      initial_state=self._params.get_bool("CellularDisabled"),
      callback=self._on_cellular_disabled,
    )

    self._scroller = Scroller([
      button_item("Arabesque", lambda: tr("OPEN"), callback=on_open_arabesque,
                  enabled=lambda: on_open_arabesque is not None and arabesque_available()),
      self._adb_toggle,
      self._ssh_toggle,
      self._ssh_keys,
      self._joystick_toggle,
      self._long_maneuver_toggle,
      self._lat_maneuver_toggle,
      self._alpha_long_toggle,
      self._always_hold_toggle,
      self._ui_debug_toggle,
      self._propilot_long_toggle,
      self._icbm_selector,
      self._cellular_disabled_toggle,
    ], line_separator=True, spacing=0)

    # Toggles should be not available to change in onroad state
    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # Hide non-release toggles on release builds
    # TODO: we can do an onroad cycle, but alpha long toggle requires a deinit function to re-enable radar and not fault
    for item in (self._joystick_toggle, self._long_maneuver_toggle, self._lat_maneuver_toggle, self._alpha_long_toggle):
      item.set_visible(not self._is_release)

    # CP gating
    if ui_state.CP is not None:
      alpha_avail = (meb_settings_capable(ui_state.CP) if ui_state.CP.brand == 'volkswagen' and ui_state.CP.carFingerprint in MEB_CARS
                     else ui_state.CP.alphaLongitudinalAvailable or ioniq6_alpha_long_available(ui_state.CP))
      if not alpha_avail or (self._is_release and not (meb_settings_capable(ui_state.CP) or ford_release_long_available(ui_state.CP))):
        self._alpha_long_toggle.set_visible(False)
      else:
        self._alpha_long_toggle.set_visible(True)

      long_man_enabled = ui_state.has_longitudinal_control and ui_state.is_offroad()
      self._long_maneuver_toggle.action_item.set_enabled(long_man_enabled)
      self._lat_maneuver_toggle.action_item.set_enabled(ui_state.is_offroad())
    else:
      self._long_maneuver_toggle.action_item.set_enabled(False)
      self._lat_maneuver_toggle.action_item.set_enabled(False)
      self._alpha_long_toggle.set_visible(False)

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for key, item in (
      ("AdbEnabled", self._adb_toggle),
      ("SshEnabled", self._ssh_toggle),
      ("JoystickDebugMode", self._joystick_toggle),
      ("LongitudinalManeuverMode", self._long_maneuver_toggle),
      ("LateralManeuverMode", self._lat_maneuver_toggle),
      ("AlphaLongitudinalEnabled", self._alpha_long_toggle),
      (HOLD_KEY, self._always_hold_toggle),
      ("ShowDebugInfo", self._ui_debug_toggle),
      ("NissanProPilotLongDebug", self._propilot_long_toggle),
      ("CellularDisabled", self._cellular_disabled_toggle),
    ):
      try:
        raw = self._params.get_bool(key)
      except UnknownKeyName:
        if key != "NissanProPilotLongDebug":
          raise
        raw = False
      value = (not stock_long_requested(self._params) if key == "AlphaLongitudinalEnabled" and meb_settings_capable(ui_state.CP)
               else raw)
      item.action_item.set_state(value)

    icbm_selector = getattr(self, "_icbm_selector", None)
    if icbm_selector is not None:
      icbm_selector.action_item.set_selected_button(self._icbm_mode())

  def _icbm_mode(self) -> int:
    return read_icbm_mode(self._params)

  def _on_icbm_mode(self, index: int):
    if not ui_state.is_offroad() or type(index) is not int or not 0 <= index <= 2:
      self._icbm_selector.action_item.set_selected_button(self._icbm_mode())
      return
    try:
      self._params.put(ICBM_KEY, index, block=True)
    except UnknownKeyName:
      pass
    self._icbm_selector.action_item.set_selected_button(self._icbm_mode())

  def _on_always_hold(self, state: bool):
    if ui_state.is_offroad():
      self._params.put_bool(HOLD_KEY, state, block=True)
    self._always_hold_toggle.action_item.set_state(self._params.get_bool(HOLD_KEY))

  def _propilot_long_enabled(self) -> bool:
    try:
      return self._params.get_bool("NissanProPilotLongDebug")
    except UnknownKeyName:
      return False

  def _on_propilot_long_debug(self, state: bool):
    try:
      self._params.put_bool("NissanProPilotLongDebug", state, block=True)
    except UnknownKeyName:
      self._propilot_long_toggle.action_item.set_state(False)

  def _on_enable_ui_debug(self, state: bool):
    self._params.put_bool("ShowDebugInfo", state, block=True)
    gui_app.set_show_touches(state)
    gui_app.set_show_fps(state)

  def _on_cellular_disabled(self, state: bool):
    self._params.put_bool("CellularDisabled", state, block=True)

  def _on_enable_adb(self, state: bool):
    self._params.put_bool("AdbEnabled", state, block=True)

  def _on_enable_ssh(self, state: bool):
    self._params.put_bool("SshEnabled", state, block=True)

  def _on_joystick_debug_mode(self, state: bool):
    self._params.put_bool("JoystickDebugMode", state, block=True)
    self._params.put_bool("LongitudinalManeuverMode", False, block=True)
    self._long_maneuver_toggle.action_item.set_state(False)
    self._params.put_bool("LateralManeuverMode", False, block=True)
    self._lat_maneuver_toggle.action_item.set_state(False)

  def _on_long_maneuver_mode(self, state: bool):
    self._params.put_bool("LongitudinalManeuverMode", state, block=True)
    self._params.put_bool("JoystickDebugMode", False, block=True)
    self._joystick_toggle.action_item.set_state(False)
    self._params.put_bool("LateralManeuverMode", False, block=True)
    self._lat_maneuver_toggle.action_item.set_state(False)

  def _on_lat_maneuver_mode(self, state: bool):
    self._params.put_bool("LateralManeuverMode", state, block=True)
    self._params.put_bool("ExperimentalMode", False, block=True)
    self._params.put_bool("JoystickDebugMode", False, block=True)
    self._joystick_toggle.action_item.set_state(False)
    self._params.put_bool("LongitudinalManeuverMode", False, block=True)
    self._long_maneuver_toggle.action_item.set_state(False)

  def _on_alpha_long_enabled(self, state: bool):
    if state:
      def confirm_callback(result: DialogResult):
        if result == DialogResult.CONFIRM:
          if (not ui_state.is_offroad() or ui_state.CP is None or
              not (meb_settings_capable(ui_state.CP) if ui_state.CP.brand == 'volkswagen' and ui_state.CP.carFingerprint in MEB_CARS
                     else ui_state.CP.alphaLongitudinalAvailable or ioniq6_alpha_long_available(ui_state.CP)) or
              self._is_release and not (meb_settings_capable(ui_state.CP) or ford_release_long_available(ui_state.CP))):
            self._alpha_long_toggle.action_item.set_state(False)
            return
          self._params.put_bool("AlphaLongitudinalEnabled", True, block=True)
          self._params.put_bool("OnroadCycleRequested", True, block=True)
          self._update_toggles()
        else:
          self._alpha_long_toggle.action_item.set_state(False)

      # show confirmation dialog
      content = (f"<h1>{self._alpha_long_toggle.title}</h1><br>" +
                 f"<p>{self._alpha_long_toggle.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)

    else:
      if not ui_state.is_offroad():
        self._alpha_long_toggle.action_item.set_state(True)
        return
      self._params.put_bool("AlphaLongitudinalEnabled", False, block=True)
      self._params.put_bool("OnroadCycleRequested", True, block=True)
      self._update_toggles()
