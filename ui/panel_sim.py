"""ui/panel_sim.py — 3D Simulation Panel (persistent, never destroyed)"""
import numpy as np
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QFrame, QPushButton, QSizePolicy, QSplitter)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QKeyEvent

import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import warnings; warnings.filterwarnings("ignore")

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from physics import simulate_flight, simulate_mission, compute_physics

# ── HUD metric widget ────────────────────────────────────────────────────────
class HudBox(QFrame):
    def __init__(self, label, unit, color="#3fb950", parent=None):
        super().__init__(parent)
        self.setObjectName("hud_box")
        self.setMinimumHeight(70)
        v = QVBoxLayout(self); v.setContentsMargins(10,8,10,8); v.setSpacing(2)
        lname = QLabel(label.upper()); lname.setObjectName("unit_lbl"); v.addWidget(lname)
        self.val = QLabel("--")
        self.val.setStyleSheet(f"color:{color};font-size:18px;font-weight:bold;font-family:Consolas;")
        self.val.setAlignment(Qt.AlignmentFlag.AlignRight); v.addWidget(self.val)
        lu = QLabel(unit); lu.setObjectName("unit_lbl")
        lu.setAlignment(Qt.AlignmentFlag.AlignRight); v.addWidget(lu)

    def set_value(self, v, fmt=".2f"):
        self.val.setText(format(v, fmt))


# ── Battery bar ──────────────────────────────────────────────────────────────
class BatteryBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(20)
        self._pct = 100.0

    def set_pct(self, p):
        self._pct = max(0.0, min(100.0, p))
        self.update()

    def paintEvent(self, e):
        from PyQt6.QtGui import QPainter, QColor, QFont
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#0d1117"))
        w = int(self.width() * self._pct / 100)
        c = QColor("#3fb950") if self._pct>50 else QColor("#d29922") if self._pct>20 else QColor("#f85149")
        p.fillRect(0, 0, w, self.height(), c)
        p.setPen(QColor("#e6edf3"))
        f = QFont("Consolas", 8, QFont.Weight.Bold)
        p.setFont(f)
        p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, f"{self._pct:.0f}%")


# ── Live scrolling mini chart ────────────────────────────────────────────────
class LiveChart(FigureCanvas):
    def __init__(self, ylabel, color, maxpts=200, parent=None):
        self.fig, self.ax = plt.subplots(figsize=(3.5, 1.4))
        super().__init__(self.fig)
        self.fig.patch.set_facecolor("#0d1117")
        self.ax.set_facecolor("#161b22")
        self.ax.tick_params(colors="#6e7681", labelsize=6)
        for sp in self.ax.spines.values(): sp.set_color("#21262d")
        self.ax.set_ylabel(ylabel, color="#6e7681", fontsize=7)
        self.ax.grid(True, color="#21262d", lw=0.5)
        self._color = color; self._maxpts = maxpts
        self._xs = []; self._ys = []
        self._line, = self.ax.plot([], [], color=color, lw=1.5)
        self.fig.tight_layout(pad=0.4)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setFixedHeight(110)

    def push(self, x, y):
        self._xs.append(x); self._ys.append(y)
        if len(self._xs) > self._maxpts:
            self._xs.pop(0); self._ys.pop(0)
        self._line.set_data(self._xs, self._ys)
        self.ax.relim(); self.ax.autoscale_view()
        self.draw_idle()

    def reset(self):
        self._xs=[]; self._ys=[]
        self._line.set_data([],[])
        self.ax.relim(); self.draw_idle()


# ── 3D drone canvas ──────────────────────────────────────────────────────────
class DroneCanvas3D(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = plt.figure(figsize=(6, 5))
        super().__init__(self.fig)
        self.fig.patch.set_facecolor("#0d1117")
        self.ax = self.fig.add_subplot(111, projection="3d")
        self._rotor_angle = 0.0
        self._draw_idle_state()
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def _style_ax(self):
        ax = self.ax; ax.set_facecolor("#0d1117")
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False; pane.set_edgecolor("#21262d")
        ax.tick_params(colors="#444466", labelsize=6)
        ax.set_xlabel("X (m)", color="#555577", fontsize=7)
        ax.set_ylabel("Y (m)", color="#555577", fontsize=7)
        ax.set_zlabel("Alt (m)", color="#58a6ff", fontsize=8)

    def _draw_idle_state(self):
        self.ax.cla(); self._style_ax()
        self.ax.set_xlim(-0.6,0.6); self.ax.set_ylim(-0.6,0.6); self.ax.set_zlim(0,15)
        self.ax.set_title("3D UAV  |  PID Flight Controller", color="#58a6ff", fontsize=10, pad=6)
        self._draw_drone(0.0, 0.0, 0.0)
        self.draw()

    def update_drone(self, x, y, z, rotor_angle, thrust_ratio, wind_ms=0):
        self.ax.cla(); self._style_ax()
        self.ax.set_xlim(-0.6,0.6); self.ax.set_ylim(-0.6,0.6); self.ax.set_zlim(0,15)
        title = f"Alt: {z:.1f}m   Thrust Ratio: {thrust_ratio:.2f}   Wind: {wind_ms:.1f}m/s"
        self.ax.set_title(title, color="#58a6ff", fontsize=9, pad=4)
        # ground grid
        for gx in np.linspace(-0.6,0.6,8):
            self.ax.plot([gx,gx],[-0.6,0.6],[0,0], color="#1a1a3a", lw=0.4, alpha=0.5)
        for gy in np.linspace(-0.6,0.6,8):
            self.ax.plot([-0.6,0.6],[gy,gy],[0,0], color="#1a1a3a", lw=0.4, alpha=0.5)
        # target altitude line
        self.ax.plot([0,0],[0,0],[0,14], color="#1f6feb", lw=0.8, linestyle="--", alpha=0.4)
        # wind arrow
        if wind_ms > 0.5:
            self.ax.quiver(0.4, 0, z, -wind_ms/10, 0, 0, color="#d29922",
                          linewidth=1.5, arrow_length_ratio=0.3)
        self._draw_drone(z, rotor_angle, thrust_ratio)
        self.draw_idle()

    def _draw_drone(self, z, rotor_angle, thrust_ratio):
        ax = self.ax
        # Hub
        u2,v2 = np.mgrid[0:2*np.pi:16j, 0:np.pi:8j]
        xh = 0.08*np.cos(u2)*np.sin(v2); yh = 0.08*np.sin(u2)*np.sin(v2)
        zh = 0.04*np.cos(v2)+z
        ax.plot_surface(xh,yh,zh, color="#1f6feb", alpha=0.85, linewidth=0)

        arm_L = 0.28
        arm_ends = [(arm_L,arm_L),(-arm_L,arm_L),(arm_L,-arm_L),(-arm_L,-arm_L)]
        colors = ["#3fb950","#f85149","#3fb950","#f85149"]
        r_prop = 0.11

        for i,(ex,ey) in enumerate(arm_ends):
            ax.plot([0,ex],[0,ey],[z,z], color="#c9d1d9", lw=3)
            R_g,Th_g = np.meshgrid(np.linspace(0,r_prop,2), np.linspace(0,2*np.pi,20))
            Xd = ex+R_g*np.cos(Th_g); Yd = ey+R_g*np.sin(Th_g)
            Zd = np.full_like(Xd, z+0.02)
            alpha_d = 0.22+0.35*min(thrust_ratio,1.0)
            ax.plot_surface(Xd,Yd,Zd, color=colors[i], alpha=alpha_d, linewidth=0)
            for b in range(2):
                ang = rotor_angle + b*np.pi
                ax.plot([ex,ex+r_prop*np.cos(ang)],[ey,ey+r_prop*np.sin(ang)],
                        [z+0.025,z+0.025], color=colors[i], lw=2, alpha=0.9)

        # Downwash
        if thrust_ratio > 0.15:
            for ex,ey in arm_ends:
                tz = np.linspace(z, z-0.25*thrust_ratio, 5)
                ax.scatter([ex]*5,[ey]*5,tz, color="#58a6ff",
                           s=np.linspace(40,5,5), alpha=0.06, depthshade=False)


# ── Main Simulation Panel ────────────────────────────────────────────────────
class SimPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._params = None
        self._mode = "interactive"   # interactive | autonomous | stress
        self._running = False
        self._rotor_angle = 0.0
        # State
        self._x=0.0; self._y=0.0; self._z=0.0
        self._vx=0.0; self._vy=0.0; self._vz=0.0
        self._ix=0.0; self._iy=0.0; self._iz=0.0
        self._thrust=0.0; self._power=0.0
        self._E0=1.0; self._E=1.0
        self._t=0.0; self._dt=0.05
        self._keys = set()
        self._mission_data = None   # (t,x,y,z,T,P,E) arrays
        self._mission_idx = 0
        self._wind_ms = 0.0
        self._z_target = 10.0
        self._iz_pid = 0.0

        self._build_ui()

        # Persistent QTimer — NEVER stops when switching panels
        self._timer = QTimer(self)
        self._timer.setInterval(50)  # 20 fps
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    def _build_ui(self):
        root = QHBoxLayout(self); root.setContentsMargins(0,0,0,0); root.setSpacing(0)

        # ── Left: 3D + live charts ─────────────────────────────
        left = QWidget(); lv = QVBoxLayout(left)
        lv.setContentsMargins(12,12,6,12); lv.setSpacing(8)

        # Mode buttons
        mb = QHBoxLayout()
        self._btn_interactive = QPushButton("Interactive (WASD)"); self._btn_interactive.setObjectName("mode_btn")
        self._btn_autonomous  = QPushButton("Autonomous Mission"); self._btn_autonomous.setObjectName("mode_btn")
        self._btn_stress      = QPushButton("Stress Test");        self._btn_stress.setObjectName("mode_btn")
        for b in [self._btn_interactive,self._btn_autonomous,self._btn_stress]:
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            mb.addWidget(b)
        self._btn_interactive.clicked.connect(lambda: self._set_mode("interactive"))
        self._btn_autonomous.clicked.connect(lambda: self._set_mode("autonomous"))
        self._btn_stress.clicked.connect(lambda: self._set_mode("stress"))
        lv.addLayout(mb)

        # 3D Canvas
        self._canvas3d = DroneCanvas3D()
        lv.addWidget(self._canvas3d, 3)

        # Live mini-charts
        chart_row = QHBoxLayout()
        self._chart_alt  = LiveChart("Alt (m)",  "#58a6ff")
        self._chart_pwr  = LiveChart("Pwr (W)",  "#f85149")
        self._chart_batt = LiveChart("Batt (%)", "#3fb950")
        for c in [self._chart_alt, self._chart_pwr, self._chart_batt]:
            chart_row.addWidget(c)
        lv.addLayout(chart_row)

        # Controls hint
        self._hint_lbl = QLabel("WASD / Arrow keys: move  |  Space: up  |  Ctrl: down  |  R: reset")
        self._hint_lbl.setObjectName("unit_lbl")
        self._hint_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lv.addWidget(self._hint_lbl)

        root.addWidget(left, 3)

        # ── Right: HUD ────────────────────────────────────────
        right = QWidget(); rv = QVBoxLayout(right)
        right.setFixedWidth(230); rv.setContentsMargins(6,12,12,12); rv.setSpacing(8)

        hud_title = QLabel("FLIGHT TELEMETRY"); hud_title.setObjectName("section_title")
        rv.addWidget(hud_title)

        self._hud_alt  = HudBox("Altitude",  "m",   "#58a6ff")
        self._hud_vel  = HudBox("Velocity",  "m/s", "#d29922")
        self._hud_thr  = HudBox("Thrust",    "N",   "#f85149")
        self._hud_pwr  = HudBox("Power",     "W",   "#bc8cff")
        self._hud_batt = HudBox("Battery",   "%",   "#3fb950")
        self._hud_time = HudBox("Time",      "s",   "#8b949e")
        for h in [self._hud_alt,self._hud_vel,self._hud_thr,
                  self._hud_pwr,self._hud_batt,self._hud_time]:
            rv.addWidget(h)

        f = QFrame(); f.setFrameShape(QFrame.Shape.HLine)
        f.setStyleSheet("color:#21262d;"); rv.addWidget(f)

        rv.addWidget(QLabel("BATTERY LEVEL").setObjectName("unit_lbl") or QLabel("BATTERY LEVEL"))
        self._batt_bar = BatteryBar(); rv.addWidget(self._batt_bar)

        f2 = QFrame(); f2.setFrameShape(QFrame.Shape.HLine)
        f2.setStyleSheet("color:#21262d;"); rv.addWidget(f2)

        self._btn_launch = QPushButton("LAUNCH / RESET")
        self._btn_launch.setObjectName("primary_btn")
        self._btn_launch.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._btn_launch.clicked.connect(self._launch)
        rv.addWidget(self._btn_launch)

        self._btn_stop = QPushButton("STOP")
        self._btn_stop.setObjectName("danger_btn")
        self._btn_stop.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._btn_stop.clicked.connect(self._stop)
        rv.addWidget(self._btn_stop)

        rv.addStretch()
        self._status_lbl = QLabel("Ready. Set parameters in Home panel.")
        self._status_lbl.setObjectName("unit_lbl")
        self._status_lbl.setWordWrap(True)
        rv.addWidget(self._status_lbl)
        root.addWidget(right)

        self._set_mode("interactive")
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def _set_mode(self, mode):
        self._mode = mode
        for btn, m in [(self._btn_interactive,"interactive"),
                       (self._btn_autonomous,"autonomous"),
                       (self._btn_stress,"stress")]:
            btn.setProperty("active", str(m == mode).lower())
            btn.style().unpolish(btn); btn.style().polish(btn)
        hints = {
            "interactive": "WASD / Arrow keys: move laterally  |  Space: UP  |  Ctrl: DOWN  |  R: reset",
            "autonomous":  "Fly mission from Mission Planner. Set waypoints then click Launch.",
            "stress":      "Stress test: payload / wind will push beyond limits. Watch battery drain.",
        }
        self._hint_lbl.setText(hints[mode])

    def set_params(self, params: dict):
        """Called from MainWindow whenever Home panel sliders change."""
        self._params = params
        self._wind_ms = params.get("wind_ms", 0.0)

    def set_mission(self, waypoints):
        """Called from Mission panel when a mission is ready."""
        self._waypoints = waypoints

    def _launch(self):
        if self._params is None:
            self._status_lbl.setText("Set parameters in Home panel first.")
            return
        from physics import compute_physics, simulate_flight, simulate_mission
        p = self._params
        ph = compute_physics(p["m_payload"],p["m_battery"],p["batt_den"],
                             p["r_prop"],p["eta"],p["p_elec"],
                             p["altitude_m"],p["temp_c"],p["wind_ms"],
                             p["motor_wear"],p["batt_wear"])
        ph["_eta_eff"] = ph["eta_eff"]; ph["_rho"] = ph["rho"]
        self._ph = ph
        self._E0 = ph["E_batt_Wh"] * 3600 * 0.80
        self._E  = self._E0
        self._T_hover = ph["thrust_hover"]
        self._x=0.0; self._y=0.0; self._z=0.0
        self._vx=0.0; self._vy=0.0; self._vz=0.0
        self._iz_pid=0.0; self._t=0.0
        self._rotor_angle=0.0
        for c in [self._chart_alt,self._chart_pwr,self._chart_batt]: c.reset()
        if self._mode == "interactive":
            self._z_target = 4.0   # Auto-lift off to 4m hover!
        if self._mode == "autonomous":
            wps = getattr(self,"_waypoints",[])
            if not wps:
                self._status_lbl.setText("No waypoints set. Go to Mission Planner.")
                return
            td,tx,ty,tz,tT,tP,tE = simulate_mission(ph, wps)
            self._mission_data = (td,tx,ty,tz,tT,tP,tE)
            self._mission_idx = 0
        self._running = True
        self._status_lbl.setText("Simulation running...")

    def _stop(self):
        self._running = False
        self._status_lbl.setText("Stopped.")

    def _tick(self):
        """Runs at 20 fps regardless of active panel. Physics only updated when running."""
        if not self._running or self._params is None:
            return
        if self._E <= 0:
            self._running = False
            self._status_lbl.setText("Battery depleted! Mission ended.")
            return

        dt = self._dt
        self._t += dt
        ph = self._ph

        if self._mode == "autonomous" and self._mission_data is not None:
            idx = self._mission_idx
            td,tx,ty,tz,tT,tP,tE = self._mission_data
            if idx >= len(td):
                self._running=False; self._status_lbl.setText("Mission complete!"); return
            self._x=tx[idx]; self._y=ty[idx]; self._z=tz[idx]
            self._thrust=tT[idx]; self._power=tP[idx]; self._E=tE[idx]
            self._mission_idx += 1

        elif self._mode == "interactive":
            # Altitude PID
            Kp,Ki,Kd = 6.0, 0.2, 4.5
            if Qt.Key.Key_Space in self._keys: self._z_target += 0.25
            if Qt.Key.Key_Control in self._keys: self._z_target = max(0, self._z_target - 0.25)
            if Qt.Key.Key_R in self._keys: self._z_target=0.0; self._x=0;self._y=0;self._z=0
            self._z_target = max(0.0, self._z_target)
            e=self._z_target-self._z; self._iz_pid+=e*dt; d=-self._vz
            self._iz_pid = float(np.clip(self._iz_pid, -10.0, 10.0))
            a_req=Kp*e+Ki*self._iz_pid+Kd*d
            T_req=ph["m_total"]*(a_req+9.81)
            self._thrust=float(np.clip(T_req,0,ph["m_total"]*9.81*2.8))
            az=self._thrust/ph["m_total"]-9.81
            self._vz=self._vz+az*dt
            self._z=max(0.0,self._z+self._vz*dt)
            # Lateral with velocity for 6-DOF banking
            ax_req = 0.0; ay_req = 0.0
            accel = 8.0; drag = 1.8
            if Qt.Key.Key_W in self._keys or Qt.Key.Key_Up in self._keys: ay_req += accel
            if Qt.Key.Key_S in self._keys or Qt.Key.Key_Down in self._keys: ay_req -= accel
            if Qt.Key.Key_A in self._keys or Qt.Key.Key_Left in self._keys: ax_req -= accel
            if Qt.Key.Key_D in self._keys or Qt.Key.Key_Right in self._keys: ax_req += accel
            
            # Apply drag to simulate air resistance and level out
            self._vx += (ax_req - self._vx * drag) * dt
            self._vy += (ay_req - self._vy * drag) * dt
            
            self._x += self._vx * dt
            self._y += self._vy * dt
            # Bounding box of 200m
            self._x = float(np.clip(self._x, -100, 100))
            self._y = float(np.clip(self._y, -100, 100))
            # Power
            rho=ph["rho"]; A=ph["A_disk"]; eta=ph["eta_eff"]
            Pi=(self._thrust**1.5/np.sqrt(2*rho*A))/eta + ph["P_elec"]
            self._power=Pi; self._E=max(0.0,self._E-Pi*dt)

        else:  # stress test — same as interactive but with live slider changes
            rho=ph["rho"]; A=ph["A_disk"]; eta=ph["eta_eff"]
            T=ph["thrust_hover"]; Pi=(T**1.5/np.sqrt(2*rho*A))/eta+ph["P_elec"]
            self._thrust=T; self._power=Pi
            self._E=max(0.0,self._E-Pi*dt)
            Kp,Kd=5.0,4.0
            e=10.0-self._z; d=-self._vz
            a_req=Kp*e+Kd*d; T_req=ph["m_total"]*(a_req+9.81)
            self._thrust=float(np.clip(T_req,0,ph["m_total"]*9.81*2.8))
            az=self._thrust/ph["m_total"]-9.81
            self._vz+=az*dt; self._z=max(0.0,self._z+self._vz*dt)

        # Rotor spin speed proportional to thrust
        T_h = self._T_hover if hasattr(self,"_T_hover") else max(self._thrust,0.1)
        thr_ratio = self._thrust / max(T_h,0.1)
        self._rotor_angle += (2.0+thr_ratio*9.0)*dt

        # Update HUD
        batt_pct = (self._E/self._E0)*100
        speed = np.sqrt(self._vx**2+self._vy**2+self._vz**2)
        self._hud_alt.set_value(self._z)
        self._hud_vel.set_value(speed)
        self._hud_thr.set_value(self._thrust,".1f")
        self._hud_pwr.set_value(self._power,".1f")
        self._hud_batt.set_value(batt_pct,".1f")
        self._hud_time.set_value(self._t,".1f")
        self._batt_bar.set_pct(batt_pct)

        # Live charts
        self._chart_alt.push(self._t, self._z)
        self._chart_pwr.push(self._t, self._power)
        self._chart_batt.push(self._t, batt_pct)

        # Only redraw 3D if this panel is visible — keeps sim alive when on other panels
        if self.isVisible():
            self._canvas3d.update_drone(self._x, self._y, self._z,
                                        self._rotor_angle, thr_ratio, self._wind_ms)

    def keyPressEvent(self, e: QKeyEvent):
        self._keys.add(e.key()); e.accept()

    def keyReleaseEvent(self, e: QKeyEvent):
        self._keys.discard(e.key()); e.accept()
