"""
main.py — UAV Endurance Simulation
Launches TWO windows in ONE Python process:
  1. Panda3D  — real-world 3D environment (C++ renderer, very fast)
  2. PyQt6    — engineering dashboard (sliders, plots, PDF export)

They share state via queue.Queue (non-blocking, no threading needed):
  state_q : PyQt6 physics → Panda3D render  (position, orientation, telemetry)
  cmd_q   : Panda3D keyboard → PyQt6 logic  (reset, mode change)

Panda3D is driven by a QTimer calling panda_app.step() every 16ms.
This keeps BOTH windows smooth at 60fps without any threading.
"""
import sys, os, queue
os.environ["QT_LOGGING_RULES"] = "*.debug=false"

# ── 1. Panda3D config MUST be set before any panda3d import ──────────────────
from panda3d.core import loadPrcFileData
loadPrcFileData('', 'win-origin 50 50')     # Panda3D window: left side

# ── 2. Create shared queues ───────────────────────────────────────────────────
state_q = queue.Queue(maxsize=2)   # physics state → Panda3D
cmd_q   = queue.Queue(maxsize=10)  # commands → physics (reset etc.)

# ── 3. Create Panda3D window (must happen before QApplication on some systems) ─
from sim3d.app import UAVSim3D
panda_app = UAVSim3D(state_q, cmd_q)

# ── 4. Now create PyQt6 application ──────────────────────────────────────────
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore    import QTimer
from PyQt6.QtGui     import QFont

app = QApplication(sys.argv)
app.setApplicationName("UAV Endurance Simulation")

# Load stylesheet
qss = os.path.join(os.path.dirname(__file__), "assets", "styles.qss")
if os.path.exists(qss):
    with open(qss) as f: app.setStyleSheet(f.read())
app.setFont(QFont("Segoe UI", 11))

# ── 5. Create dashboard (passes queues in) ────────────────────────────────────
from ui.main_window_3d import MainWindow3D
win = MainWindow3D(state_q, cmd_q)
win.move(1100, 50)   # Dashboard: right side of screen
win.show()

# ── 6. QTimer drives Panda3D at ~60fps — SAME THREAD, no threading needed ────
panda_timer = QTimer()
panda_timer.setInterval(16)         # 16ms ≈ 60fps
panda_timer.timeout.connect(panda_app.step)
panda_timer.start()

# ── 7. Qt event loop runs everything ─────────────────────────────────────────
print("=" * 60)
print("  UAV SIMULATION RUNNING")
print("  Panda3D 3D window: LEFT   | PyQt6 dashboard: RIGHT")
print("  Press C in 3D window to cycle cameras")
print("  Press R in 3D window to reset drone")
print("=" * 60)
sys.exit(app.exec())
