"""sim3d/app.py — Panda3D 3D world. Driven by QTimer via .step()"""
import queue, math, os, sys

# Must set config BEFORE ShowBase import
from panda3d.core import loadPrcFileData
loadPrcFileData('', 'window-title UAV 3D Simulation | C=Camera | WASD=Move | Space=Up | R=Reset')
loadPrcFileData('', 'win-size 1024 720')
loadPrcFileData('', 'show-frame-rate-meter 1')
loadPrcFileData('', 'sync-video 0')          # unlocked framerate
loadPrcFileData('', 'basic-shaders-only 1')  # safe for Intel HD
loadPrcFileData('', 'shadow-depth-bits 0')   # disable shadow maps

from direct.showbase.ShowBase import ShowBase
from panda3d.core import Vec3, TextNode, NodePath, KeyboardButton
from direct.gui.OnscreenText import OnscreenText

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from sim3d.environment import setup_environment
from sim3d.drone       import DroneModel
from sim3d.camera      import CameraController
from sim3d.particles   import VFXSystem

class UAVSim3D(ShowBase):
    """
    Panda3D real-world simulation window.
    Driven externally via .step() called by a PyQt6 QTimer.
    Physics state arrives via state_q (Queue) from the PyQt6 dashboard.
    """
    def __init__(self, state_q: queue.Queue, cmd_q: queue.Queue):
        ShowBase.__init__(self)
        self.state_q = state_q
        self.cmd_q   = cmd_q

        # ── State ────────────────────────────────────────────────────────────
        self._px = 0.0; self._py = 0.0; self._pz = 0.0
        self._roll = 0.0; self._pitch = 0.0; self._yaw = 0.0
        self._thrust_ratio = 0.0
        self._crashed  = False
        self._running  = False
        self._sim_time = 0.0

        # ── Build world ───────────────────────────────────────────────────────
        setup_environment(self)
        self.drone_model  = DroneModel(self)
        self.camera_ctrl  = CameraController(self, self.drone_model.root)
        self.vfx          = VFXSystem(self, self.drone_model.root)

        # ── HUD text ──────────────────────────────────────────────────────────
        self._hud_alt  = self._make_hud("ALT: --",   (-1.55, 0.95), scale=0.05)
        self._hud_pwr  = self._make_hud("PWR: --",   (-1.55, 0.88), scale=0.05)
        self._hud_batt = self._make_hud("BATT: --",  (-1.55, 0.81), scale=0.05)
        self._hud_mode = self._make_hud("[FOLLOW]",  ( 0.0,  0.95), scale=0.05, align="center")
        self._hud_tip  = self._make_hud(
            "WASD: Move  Space: Up  Ctrl: Down  C: Camera  R: Reset",
            (0.0, -0.93), scale=0.038, align="center")

        # ── Key bindings ──────────────────────────────────────────────────────
        self.accept('c',      self._cycle_camera)
        self.accept('escape', self.userExit)
        self.accept('r',      self._reset_drone)

        # Register key events to forward to PyQt6
        flight_keys = [
            'w', 'a', 's', 'd', 'space', 'control', 'lcontrol', 'rcontrol',
            'arrow_up', 'arrow_down', 'arrow_left', 'arrow_right'
        ]
        for k in flight_keys:
            self.accept(k, self._send_key, [k, True])
            self.accept(k + '-up', self._send_key, [k, False])

        # ── Physics-driven movement state ─────────────────────────────────────
        self._z_target = 0.0
        self._iz_pid   = 0.0

        # ── Task: MUST BE ADDED IN __init__ ──────────────────────────────────
        self.taskMgr.add(self._update_task, 'sim_update')

    def _send_key(self, key, is_down):
        norm_key = key
        if key in ('lcontrol', 'rcontrol'):
            norm_key = 'control'
        try:
            self.cmd_q.put_nowait({'cmd': 'key', 'k': norm_key, 'down': is_down})
        except queue.Full:
            pass

    def _make_hud(self, text, pos, scale=0.05, align="left"):
        a = TextNode.ALeft if align == "left" else TextNode.ACenter
        return OnscreenText(
            text=text, pos=pos, scale=scale,
            fg=(0.2, 1.0, 0.5, 1), shadow=(0, 0, 0, 0.7),
            align=a, mayChange=True)

    def step(self):
        """Called by PyQt6 QTimer at ~60fps. Drives the Panda3D task manager."""
        self.taskMgr.step()

    def _update_task(self, task):
        dt = globalClock.getDt()
        self._sim_time += dt

        # Pull physics state from PyQt6 dashboard (non-blocking)
        try:
            while not self.state_q.empty():
                state = self.state_q.get_nowait()
                self._px         = state.get('x', self._px)
                self._py         = state.get('y', self._py)
                self._pz         = state.get('z', self._pz)
                self._roll       = state.get('roll',  0.0)
                self._pitch      = state.get('pitch', 0.0)
                self._yaw        = state.get('yaw',   0.0)
                self._thrust_ratio = state.get('thrust_ratio', 0.0)
                batt_pct         = state.get('batt_pct', 100.0)
                power            = state.get('power', 0.0)
                crashed          = state.get('crashed', False)
                self._running    = state.get('running', False)

                # Crash trigger
                if crashed and not self._crashed:
                    self.vfx.trigger_crash()
                self._crashed = crashed

                # HUD update
                self._hud_alt.setText(f"ALT: {self._pz:.1f} m")
                self._hud_pwr.setText(f"PWR: {power:.0f} W")
                col = (0.2,1.0,0.3,1) if batt_pct > 50 else (1.0,0.7,0.1,1) if batt_pct > 20 else (1.0,0.2,0.2,1)
                self._hud_batt.setText(f"BATT: {batt_pct:.0f}%")
                self._hud_batt['fg'] = col
        except queue.Empty:
            pass

        # Update 3D drone
        self.drone_model.update(
            self._px, self._py, self._pz,
            self._roll, self._pitch, self._yaw,
            self._thrust_ratio, self._sim_time)

        # Update camera
        self.camera_ctrl.update(dt)

        # Update VFX
        self.vfx.update(self._thrust_ratio, self._pz, self._sim_time)

        return task.cont

    def _cycle_camera(self):
        self.camera_ctrl.cycle_mode()
        modes = ["[FOLLOW]", "[FPV]", "[ORBIT - drag mouse]"]
        self._hud_mode.setText(modes[self.camera_ctrl._mode])

    def _reset_drone(self):
        """Tell PyQt6 dashboard to reset simulation."""
        try:
            self.cmd_q.put_nowait({'cmd': 'reset'})
        except queue.Full:
            pass
