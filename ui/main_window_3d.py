"""
ui/main_window_3d.py — PyQt6 Dashboard that feeds physics state to Panda3D.
Extends the existing 5-panel layout and adds a QTimer that pushes drone
state into state_q at 20Hz so Panda3D can update the 3D view.
"""
import queue, math
from PyQt6.QtCore import QTimer, Qt
from ui.main_window import MainWindow

class MainWindow3D(MainWindow):
    """
    Subclass of the existing dashboard.
    Adds: physics→Panda3D push, cmd_q listener, 6-DOF roll/pitch calculation.
    """
    def __init__(self, state_q: queue.Queue, cmd_q: queue.Queue, parent=None):
        super().__init__()
        self._state_q = state_q
        self._cmd_q   = cmd_q

        # Push physics to Panda3D at 20Hz (separate from Panda3D's 60Hz render)
        self._push_timer = QTimer(self)
        self._push_timer.setInterval(50)   # 20Hz
        self._push_timer.timeout.connect(self._push_state)
        self._push_timer.start()

        # Poll command queue (keyboard events from Panda3D)
        self._cmd_timer = QTimer(self)
        self._cmd_timer.setInterval(20)
        self._cmd_timer.timeout.connect(self._poll_cmds)
        self._cmd_timer.start()

    def _push_state(self):
        """Collect current sim state and push to Panda3D via queue."""
        s = self._panel_sim
        if not hasattr(s, '_t'):
            return

        # 6-DOF angles from velocity and thrust
        vx = s._vx if hasattr(s, '_vx') else 0.0
        vy = s._vy if hasattr(s, '_vy') else 0.0
        vz = s._vz
        thrust_ratio = s._thrust / max(getattr(s, '_T_hover', s._thrust + 0.001), 0.001) \
            if s._thrust > 0 else 0.0

        # Roll from lateral velocity, pitch from forward velocity
        roll_deg  = math.degrees(math.atan2(vx * 0.15, 1.0))
        pitch_deg = math.degrees(math.atan2(vy * 0.15, 1.0))

        batt_pct = (s._E / max(s._E0, 0.001)) * 100 if hasattr(s, '_E0') else 100.0

        state = {
            'x':            s._x,
            'y':            s._y,
            'z':            s._z,
            'roll':         roll_deg,
            'pitch':        pitch_deg,
            'yaw':          0.0,
            'thrust_ratio': thrust_ratio,
            'power':        s._power,
            'batt_pct':     batt_pct,
            'crashed':      False,
            'running':      s._running,
        }
        # Non-blocking: drop if full
        try:
            try:
                self._state_q.get_nowait()
            except queue.Empty:
                pass
            self._state_q.put_nowait(state)
        except queue.Full:
            pass

    def _poll_cmds(self):
        try:
            while not self._cmd_q.empty():
                cmd = self._cmd_q.get_nowait()
                if cmd.get('cmd') == 'reset':
                    self._panel_sim._stop()
                    self._panel_sim._z = 0.0
                    self._panel_sim._vz = 0.0
                    self._panel_sim._x = 0.0
                    self._panel_sim._y = 0.0
                    self._panel_sim._vx = 0.0
                    self._panel_sim._vy = 0.0
                    self._panel_sim._z_target = 0.0
                    self._panel_sim._iz_pid = 0.0
                    print("[Dashboard] Drone reset via R key")
                elif cmd.get('cmd') == 'key':
                    k_str = cmd.get('k')
                    is_down = cmd.get('down')
                    mapping = {
                        'w': Qt.Key.Key_W, 'a': Qt.Key.Key_A, 's': Qt.Key.Key_S, 'd': Qt.Key.Key_D,
                        'space': Qt.Key.Key_Space, 'control': Qt.Key.Key_Control,
                        'arrow_up': Qt.Key.Key_Up, 'arrow_down': Qt.Key.Key_Down,
                        'arrow_left': Qt.Key.Key_Left, 'arrow_right': Qt.Key.Key_Right
                    }
                    if k_str in mapping:
                        qkey = mapping[k_str]
                        if is_down:
                            self._panel_sim._keys.add(qkey)
                        else:
                            self._panel_sim._keys.discard(qkey)
        except queue.Empty:
            pass
