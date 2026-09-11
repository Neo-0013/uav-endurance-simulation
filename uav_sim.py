"""
============================================================
  UAV QUADCOPTER ENDURANCE SIMULATION
  Full 3D Physics Simulation + GUI Dashboard

  Requirements: numpy and matplotlib (already installed)
  tkinter is built-in to Python.

  Run:  python3 uav_sim.py
============================================================
"""

import numpy as np
import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.gridspec as gridspec
import warnings
warnings.filterwarnings("ignore")

# ─────────────────────────────────────────────
#  CONSTANTS
# ─────────────────────────────────────────────
G       = 9.81    # m/s^2
RHO     = 1.225   # kg/m^3  (sea-level air density)
N_PROPS = 4       # quadcopter


# ─────────────────────────────────────────────
#  PHYSICS ENGINE
# ─────────────────────────────────────────────
def compute_physics(m_payload, m_battery, batt_density, r_prop, eta, p_elec):
    """
    Momentum Theory hover model.
    Returns dict of all key computed values.
    """
    m_empty   = 1.2                          # fixed airframe (kg)
    m_total   = m_empty + m_battery + m_payload
    weight    = m_total * G
    A_disk    = N_PROPS * np.pi * r_prop**2  # total actuator disk area
    T         = weight                        # hover: thrust = weight
    P_induced = (T ** 1.5) / np.sqrt(2 * RHO * A_disk)
    P_total   = (P_induced / eta) + p_elec
    E_batt_Wh = m_battery * batt_density
    endurance_m = (0.80 * E_batt_Wh) / P_total * 60   # 80 % usable battery
    return dict(m_total=m_total, weight=weight, thrust_hover=T,
                P_induced=P_induced, P_elec=p_elec, P_total=P_total,
                E_batt_Wh=E_batt_Wh, endurance_m=endurance_m, A_disk=A_disk)


def sweep_payload(m_battery, batt_density, r_prop, eta, p_elec,
                  payload_range=None):
    if payload_range is None:
        payload_range = np.linspace(0, 3.0, 100)
    rows = []
    for mp in payload_range:
        ph = compute_physics(mp, m_battery, batt_density, r_prop, eta, p_elec)
        rows.append((mp, ph["P_total"], ph["endurance_m"]))
    return np.array(rows)   # cols: payload, power, endurance_min


def simulate_flight(params, z_target=12.0, dt=0.05, t_end=18.0):
    """
    PID-controlled vertical take-off and hover.
    Returns arrays: t, z, vz, thrust, power, battery_energy_J
    """
    m_total  = params["m_total"]
    A_disk   = params["A_disk"]
    eta      = params.get("_eta", 0.55)
    p_elec   = params["P_elec"]
    E_batt_J = params["E_batt_Wh"] * 3600 * 0.80

    Kp, Ki, Kd = 4.0, 0.15, 3.5
    T_max    = m_total * G * 2.5
    steps    = int(t_end / dt) + 1

    t_arr  = np.zeros(steps);  z_arr  = np.zeros(steps)
    vz_arr = np.zeros(steps);  T_arr  = np.zeros(steps)
    P_arr  = np.zeros(steps);  E_arr  = np.zeros(steps)
    E_arr[0] = E_batt_J
    integral = 0.0

    for i in range(steps - 1):
        t_arr[i + 1] = t_arr[i] + dt
        err          = z_target - z_arr[i]
        integral    += err * dt
        derivative   = -vz_arr[i]
        a_req        = Kp * err + Ki * integral + Kd * derivative
        T_req        = m_total * (a_req + G)
        T_arr[i]     = float(np.clip(T_req, 0.0, T_max))

        az            = (T_arr[i] / m_total) - G
        vz_arr[i + 1] = vz_arr[i] + az * dt
        z_arr[i + 1]  = max(0.0, z_arr[i] + vz_arr[i + 1] * dt)
        if z_arr[i + 1] == 0.0:
            vz_arr[i + 1] = 0.0

        P_i        = ((T_arr[i] ** 1.5) / np.sqrt(2 * RHO * A_disk)) / eta + p_elec
        P_arr[i]   = P_i
        E_arr[i+1] = max(0.0, E_arr[i] - P_i * dt)

    T_arr[-1] = T_arr[-2];  P_arr[-1] = P_arr[-2]
    return t_arr, z_arr, vz_arr, T_arr, P_arr, E_arr


# ─────────────────────────────────────────────
#  MAIN APPLICATION
# ─────────────────────────────────────────────
class UAVSimApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("UAV Quadcopter  —  Endurance Simulation Dashboard")
        self.geometry("1300x820")
        self.configure(bg="#1a1a2e")
        self.resizable(True, True)

        self._anim        = None
        self._rotor_angle = 0.0

        self._build_styles()
        self._build_header()
        self._build_notebook()
        self._build_tab1()
        self._build_tab2()
        self._build_tab3()

    # ── Styles ──────────────────────────────────────────────────────────────
    def _build_styles(self):
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure("Dark.TNotebook",      background="#1a1a2e", borderwidth=0)
        s.configure("Dark.TNotebook.Tab",  background="#16213e", foreground="#e0e0e0",
                    font=("Helvetica", 11, "bold"), padding=[20, 8])
        s.map("Dark.TNotebook.Tab",
              background=[("selected", "#0f3460")],
              foreground=[("selected", "#00d4ff")])
        s.configure("Card.TFrame", background="#16213e")

    # ── Header ──────────────────────────────────────────────────────────────
    def _build_header(self):
        hdr = tk.Frame(self, bg="#0f3460", height=55)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="  UAV QUADCOPTER ENDURANCE SIMULATION DASHBOARD",
                 bg="#0f3460", fg="#00d4ff",
                 font=("Helvetica", 15, "bold")).pack(side=tk.LEFT, padx=20, pady=12)
        tk.Label(hdr,
                 text="Physics: Momentum Theory  |  Controller: PID  |  Model: Quadcopter",
                 bg="#0f3460", fg="#aaaaaa",
                 font=("Helvetica", 9)).pack(side=tk.RIGHT, padx=20)

    # ── Notebook ─────────────────────────────────────────────────────────────
    def _build_notebook(self):
        self.nb = ttk.Notebook(self, style="Dark.TNotebook")
        self.nb.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.tab1 = ttk.Frame(self.nb, style="Card.TFrame")
        self.tab2 = ttk.Frame(self.nb, style="Card.TFrame")
        self.tab3 = ttk.Frame(self.nb, style="Card.TFrame")
        self.nb.add(self.tab1, text="  Control Panel")
        self.nb.add(self.tab2, text="  3D Live Simulation")
        self.nb.add(self.tab3, text="  Endurance Analysis")

    # ════════════════════════════════════════════════════════════════════════
    #  TAB 1  —  Control Panel
    # ════════════════════════════════════════════════════════════════════════
    def _build_tab1(self):
        root = self.tab1

        # ── Left: Sliders ─────────────────────────────────────
        lf = tk.Frame(root, bg="#16213e")
        lf.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=16, pady=16)

        tk.Label(lf, text="UAV PARAMETER CONFIGURATION",
                 bg="#16213e", fg="#00d4ff",
                 font=("Helvetica", 13, "bold")).pack(anchor="w", pady=(0, 12))

        cfg = [
            ("Payload Mass",            "m_payload", 0.0, 3.0,  0.5,  "kg"),
            ("Battery Mass",            "m_battery", 0.2, 2.0,  0.8,  "kg"),
            ("Battery Energy Density",  "batt_den",  80,  250,  160,  "Wh/kg"),
            ("Propeller Radius",        "r_prop",    0.06,0.25, 0.12, "m"),
            ("Motor Efficiency",        "eta",       0.3, 0.9,  0.55, ""),
            ("Electronics Power",       "p_elec",    5,   40,   15,   "W"),
        ]
        self._slider_vars   = {}
        self._value_labels  = {}
        for label, key, lo, hi, default, unit in cfg:
            row = tk.Frame(lf, bg="#16213e")
            row.pack(fill=tk.X, pady=6)
            tk.Label(row, text=label, bg="#16213e", fg="#cccccc",
                     width=26, anchor="w",
                     font=("Helvetica", 10)).pack(side=tk.LEFT)
            var = tk.DoubleVar(value=default)
            self._slider_vars[key] = var
            ttk.Scale(row, from_=lo, to=hi, variable=var,
                      orient=tk.HORIZONTAL, length=280,
                      command=lambda _, k=key: self._on_slider(k)).pack(side=tk.LEFT, padx=8)
            fmt = f"{default:.0f}" if key in ("batt_den", "p_elec") else f"{default:.2f}"
            val_lbl = tk.Label(row, text=f"{fmt} {unit}",
                               bg="#0a0a1a", fg="#00ff88",
                               width=12, anchor="center",
                               font=("Consolas", 10, "bold"))
            val_lbl.pack(side=tk.LEFT, padx=4)
            self._value_labels[key] = (val_lbl, unit)

        # ── Right: Live Computed Panel ─────────────────────────
        rf = tk.Frame(root, bg="#0f3460", width=340)
        rf.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 16), pady=16)
        rf.pack_propagate(False)

        tk.Label(rf, text="LIVE COMPUTED VALUES", bg="#0f3460", fg="#00d4ff",
                 font=("Helvetica", 12, "bold")).pack(pady=(16, 12))

        metrics = [
            ("Total Mass",     "_lbl_mass",   "kg"),
            ("Total Weight",   "_lbl_weight", "N"),
            ("Hover Thrust",   "_lbl_thrust", "N"),
            ("Hover Power",    "_lbl_power",  "W"),
            ("Battery Energy", "_lbl_energy", "Wh"),
            ("Endurance",      "_lbl_endur",  "min"),
            ("Power / kg",     "_lbl_pw_kg",  "W/kg"),
        ]
        for name, attr, unit in metrics:
            row = tk.Frame(rf, bg="#0f3460")
            row.pack(fill=tk.X, padx=16, pady=5)
            tk.Label(row, text=name, bg="#0f3460", fg="#aaaaaa",
                     font=("Helvetica", 10), anchor="w",
                     width=18).pack(side=tk.LEFT)
            lbl = tk.Label(row, text="--", bg="#0a0a1a", fg="#00ff88",
                           font=("Consolas", 11, "bold"),
                           width=10, anchor="e")
            lbl.pack(side=tk.RIGHT)
            tk.Label(row, text=unit, bg="#0f3460", fg="#666666",
                     font=("Helvetica", 9), width=5).pack(side=tk.RIGHT)
            setattr(self, attr, lbl)

        tk.Frame(rf, bg="#00d4ff", height=1).pack(fill=tk.X, padx=16, pady=12)

        btn_frame = tk.Frame(rf, bg="#0f3460")
        btn_frame.pack(fill=tk.X, padx=16, pady=4)

        tk.Button(btn_frame, text="LAUNCH SIMULATION",
                  bg="#00d4ff", fg="#000000",
                  font=("Helvetica", 10, "bold"), relief="flat",
                  cursor="hand2", pady=8,
                  command=self._launch_sim).pack(fill=tk.X, pady=4)

        tk.Button(btn_frame, text="RUN ENDURANCE ANALYSIS",
                  bg="#00ff88", fg="#000000",
                  font=("Helvetica", 10, "bold"), relief="flat",
                  cursor="hand2", pady=8,
                  command=self._run_analysis).pack(fill=tk.X, pady=4)

        self._status_var = tk.StringVar(
            value="Ready. Adjust parameters, then click a button above.")
        tk.Label(rf, textvariable=self._status_var,
                 bg="#0f3460", fg="#ffaa00",
                 wraplength=300, justify=tk.LEFT,
                 font=("Helvetica", 9)).pack(padx=16, pady=12, anchor="w")

        self._update_computed()

    def _get_params(self):
        return {k: v.get() for k, v in self._slider_vars.items()}

    def _on_slider(self, key):
        var            = self._slider_vars[key]
        lbl, unit      = self._value_labels[key]
        val            = var.get()
        fmt = f"{val:.0f}" if key in ("batt_den", "p_elec") else f"{val:.2f}"
        lbl.config(text=f"{fmt} {unit}")
        self._update_computed()

    def _update_computed(self):
        p  = self._get_params()
        ph = compute_physics(p["m_payload"], p["m_battery"], p["batt_den"],
                             p["r_prop"], p["eta"], p["p_elec"])
        self._lbl_mass.config(text=f"{ph['m_total']:.3f}")
        self._lbl_weight.config(text=f"{ph['weight']:.2f}")
        self._lbl_thrust.config(text=f"{ph['thrust_hover']:.2f}")
        self._lbl_power.config(text=f"{ph['P_total']:.1f}")
        self._lbl_energy.config(text=f"{ph['E_batt_Wh']:.1f}")
        self._lbl_endur.config(text=f"{ph['endurance_m']:.1f}")
        self._lbl_pw_kg.config(text=f"{ph['P_total'] / ph['m_total']:.1f}")

    # ════════════════════════════════════════════════════════════════════════
    #  TAB 2  —  3D Live Simulation
    # ════════════════════════════════════════════════════════════════════════
    def _build_tab2(self):
        left = tk.Frame(self.tab2, bg="#0a0a1a")
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._fig_sim = plt.Figure(figsize=(7, 6), facecolor="#0a0a1a")
        self._ax3d    = self._fig_sim.add_subplot(111, projection="3d")
        self._ax3d.set_facecolor("#0a0a1a")
        self._canvas_sim = FigureCanvasTkAgg(self._fig_sim, master=left)
        self._canvas_sim.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        right = tk.Frame(self.tab2, bg="#0f3460", width=280)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)

        tk.Label(right, text="FLIGHT TELEMETRY HUD",
                 bg="#0f3460", fg="#00d4ff",
                 font=("Helvetica", 11, "bold")).pack(pady=(16, 8))

        hud = [
            ("ALTITUDE",  "_hud_alt",  "m",   "#00ff88"),
            ("VELOCITY",  "_hud_vel",  "m/s", "#ffaa00"),
            ("THRUST",    "_hud_thr",  "N",   "#ff6666"),
            ("POWER",     "_hud_pwr",  "W",   "#ff44aa"),
            ("BATTERY",   "_hud_batt", "%",   "#44ddff"),
            ("TIME",      "_hud_time", "s",   "#aaaaaa"),
        ]
        for name, attr, unit, color in hud:
            box = tk.Frame(right, bg="#0a0a1a")
            box.pack(fill=tk.X, padx=14, pady=3)
            tk.Label(box, text=name, bg="#0a0a1a", fg="#444466",
                     font=("Helvetica", 8, "bold")).pack(anchor="w", padx=8, pady=(4, 0))
            lbl = tk.Label(box, text="--", bg="#0a0a1a", fg=color,
                           font=("Consolas", 18, "bold"), anchor="e")
            lbl.pack(anchor="e", padx=10)
            tk.Label(box, text=unit, bg="#0a0a1a", fg="#333355",
                     font=("Helvetica", 8)).pack(anchor="e", padx=10, pady=(0, 4))
            setattr(self, attr, lbl)

        tk.Frame(right, bg="#00d4ff", height=1).pack(fill=tk.X, padx=14, pady=8)
        tk.Label(right, text="BATTERY LEVEL", bg="#0f3460", fg="#444466",
                 font=("Helvetica", 8, "bold")).pack(anchor="w", padx=14)
        self._batt_canvas = tk.Canvas(right, height=22, bg="#0a0a1a",
                                       highlightthickness=0)
        self._batt_canvas.pack(fill=tk.X, padx=14, pady=4)

        self._sim_status = tk.StringVar(
            value="Ready. Go to Control Panel and click 'Launch Simulation'.")
        tk.Label(right, textvariable=self._sim_status,
                 bg="#0f3460", fg="#ffaa00",
                 wraplength=250, justify=tk.LEFT,
                 font=("Helvetica", 8)).pack(padx=14, pady=8, anchor="w")

        self._draw_drone(z=0.0, rotor_angle=0.0, thrust_ratio=0.0)

    def _draw_drone(self, z=0.0, rotor_angle=0.0, thrust_ratio=0.0):
        ax = self._ax3d
        ax.cla()
        ax.set_facecolor("#0a0a1a")
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor("#222244")
        ax.set_xlim(-0.6, 0.6);  ax.set_ylim(-0.6, 0.6);  ax.set_zlim(0, 15)
        ax.set_xlabel("X (m)", color="#555577", fontsize=7)
        ax.set_ylabel("Y (m)", color="#555577", fontsize=7)
        ax.set_zlabel("Altitude (m)", color="#00d4ff", fontsize=8)
        ax.tick_params(colors="#444466", labelsize=6)
        ax.set_title("3D Quadcopter  |  PID-Controlled Flight",
                     color="#00d4ff", fontsize=10)

        # Ground grid
        for x in np.linspace(-0.6, 0.6, 8):
            ax.plot([x, x], [-0.6, 0.6], [0, 0], color="#1a1a3a", lw=0.5, alpha=0.5)
        for y in np.linspace(-0.6, 0.6, 8):
            ax.plot([-0.6, 0.6], [y, y], [0, 0], color="#1a1a3a", lw=0.5, alpha=0.5)
        ax.plot([0, 0], [0, 0], [0, 12], color="#1a2a44", lw=1,
                linestyle="--", alpha=0.5)

        # Central Hub
        u2, v2 = np.mgrid[0:2*np.pi:16j, 0:np.pi:8j]
        xh = 0.08 * np.cos(u2) * np.sin(v2)
        yh = 0.08 * np.sin(u2) * np.sin(v2)
        zh = 0.04 * np.cos(v2) + z
        ax.plot_surface(xh, yh, zh, color="#3355ff", alpha=0.85, linewidth=0)

        # Arms + rotors
        arm_L = 0.28
        arm_ends = [(arm_L, arm_L), (-arm_L, arm_L),
                    (arm_L, -arm_L), (-arm_L, -arm_L)]
        r_prop = 0.11
        colors = ["#00ff88", "#ff4444", "#00ff88", "#ff4444"]

        for i, (ex, ey) in enumerate(arm_ends):
            ax.plot([0, ex], [0, ey], [z, z], color="#aaaaaa", lw=3)

            R_g, Th_g = np.meshgrid(np.linspace(0, r_prop, 2),
                                     np.linspace(0, 2 * np.pi, 20))
            Xd = ex + R_g * np.cos(Th_g)
            Yd = ey + R_g * np.sin(Th_g)
            Zd = np.full_like(Xd, z + 0.02)
            alpha_d = 0.25 + 0.35 * min(thrust_ratio, 1.0)
            ax.plot_surface(Xd, Yd, Zd, color=colors[i],
                            alpha=alpha_d, linewidth=0)

            for b in range(2):
                ang = rotor_angle + b * np.pi
                ax.plot([ex, ex + r_prop * np.cos(ang)],
                        [ey, ey + r_prop * np.sin(ang)],
                        [z + 0.025, z + 0.025],
                        color=colors[i], lw=2, alpha=0.9)

        # Downwash glow
        if thrust_ratio > 0.15:
            for ex, ey in arm_ends:
                tz = np.linspace(z, z - 0.3 * thrust_ratio, 6)
                ax.scatter([ex] * 6, [ey] * 6, tz,
                           color="#0088ff",
                           s=np.linspace(60, 5, 6),
                           alpha=0.07, depthshade=False)

        self._canvas_sim.draw_idle()

    def _launch_sim(self):
        if self._anim is not None:
            try:
                self._anim.event_source.stop()
            except Exception:
                pass
            self._anim = None

        self.nb.select(1)
        self._sim_status.set("Simulation running...")
        self.update()

        p  = self._get_params()
        ph = compute_physics(p["m_payload"], p["m_battery"], p["batt_den"],
                             p["r_prop"], p["eta"], p["p_elec"])
        ph["_eta"] = p["eta"]

        t, z, vz, thr, pwr, ebatt = simulate_flight(ph, z_target=12.0)
        E0  = ph["E_batt_Wh"] * 3600 * 0.80
        T_h = ph["thrust_hover"]
        self._rotor_angle = 0.0
        SKIP = 2

        def update_frame(frame):
            i = frame * SKIP
            if i >= len(t):
                return
            z_now   = z[i];   thr_now = thr[i]
            pwr_now = pwr[i]; e_now   = ebatt[i]
            t_now   = t[i];   vz_now  = vz[i]

            thr_ratio = min(thr_now / T_h, 1.5)
            self._rotor_angle += (2.0 + thr_ratio * 8.0) * 0.1
            self._draw_drone(z=z_now, rotor_angle=self._rotor_angle,
                             thrust_ratio=thr_ratio)

            batt_pct = (e_now / E0) * 100
            self._hud_alt.config(text=f"{z_now:.2f}")
            self._hud_vel.config(text=f"{vz_now:.2f}")
            self._hud_thr.config(text=f"{thr_now:.1f}")
            self._hud_pwr.config(text=f"{pwr_now:.1f}")
            self._hud_batt.config(text=f"{batt_pct:.1f}")
            self._hud_time.config(text=f"{t_now:.1f}")

            bc = self._batt_canvas
            bc.delete("all")
            W  = bc.winfo_width() or 230
            fw = int(W * batt_pct / 100)
            col = ("#00ff88" if batt_pct > 50 else
                   "#ffaa00" if batt_pct > 20 else "#ff4444")
            bc.create_rectangle(0, 0, W, 22, fill="#111133", outline="")
            bc.create_rectangle(0, 0, fw, 22, fill=col, outline="")
            bc.create_text(W // 2, 11, text=f"{batt_pct:.0f}%",
                           fill="white", font=("Consolas", 8, "bold"))

        total_frames = len(t) // SKIP
        self._anim = animation.FuncAnimation(
            self._fig_sim, update_frame,
            frames=total_frames, interval=50,
            repeat=False, blit=False)
        self._canvas_sim.draw()
        self.after(int(len(t) * 50),
                   lambda: self._sim_status.set("Simulation complete!"))

    # ════════════════════════════════════════════════════════════════════════
    #  TAB 3  —  Endurance Analysis
    # ════════════════════════════════════════════════════════════════════════
    def _build_tab3(self):
        self._fig_an = plt.Figure(figsize=(11, 7), facecolor="#0a0a1a")
        self._canvas_an = FigureCanvasTkAgg(self._fig_an, master=self.tab3)
        self._canvas_an.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        ax = self._fig_an.add_subplot(111)
        ax.set_facecolor("#0a0a1a")
        ax.text(0.5, 0.5,
                "Click  'RUN ENDURANCE ANALYSIS'  in the Control Panel",
                ha="center", va="center",
                color="#555577", fontsize=13, transform=ax.transAxes)
        ax.axis("off")
        self._canvas_an.draw()

    def _run_analysis(self):
        self.nb.select(2)
        p  = self._get_params()
        ph = compute_physics(p["m_payload"], p["m_battery"], p["batt_den"],
                             p["r_prop"], p["eta"], p["p_elec"])

        payload_range = np.linspace(0, 3.0, 100)
        data      = sweep_payload(p["m_battery"], p["batt_den"],
                                  p["r_prop"], p["eta"], p["p_elec"],
                                  payload_range)
        payloads  = data[:, 0]
        powers    = data[:, 1]
        endurs    = data[:, 2]

        self._fig_an.clf()
        self._fig_an.patch.set_facecolor("#0a0a1a")

        gs = gridspec.GridSpec(2, 3, figure=self._fig_an,
                               hspace=0.45, wspace=0.38,
                               left=0.07, right=0.97,
                               top=0.90, bottom=0.10)
        ax1 = self._fig_an.add_subplot(gs[0, :2])
        ax2 = self._fig_an.add_subplot(gs[1, :2])
        ax3 = self._fig_an.add_subplot(gs[:, 2])

        PANEL  = "#0f1a30"; CYAN  = "#00d4ff"; GREEN  = "#00ff88"
        ORANGE = "#ffaa00"; RED   = "#ff6666"; GREY   = "#444466"
        cur_e  = ph["endurance_m"]; cur_mp = p["m_payload"]

        def sax(ax, title, xlabel, ylabel):
            ax.set_facecolor(PANEL)
            ax.set_title(title, color=CYAN, fontsize=9, fontweight="bold", pad=6)
            ax.set_xlabel(xlabel, color="#aaaaaa", fontsize=8)
            ax.set_ylabel(ylabel, color="#aaaaaa", fontsize=8)
            ax.tick_params(colors=GREY, labelsize=7)
            ax.spines[:].set_color("#223355")
            ax.grid(True, color="#1a2a44", lw=0.6, linestyle="--")

        # ── Plot 1: Endurance vs Payload ───────────────────────────────────
        sax(ax1, "FLIGHT ENDURANCE  vs  PAYLOAD MASS",
            "Payload Mass (kg)", "Endurance (minutes)")
        ax1.plot(payloads, endurs, color=CYAN, lw=2.2, zorder=3)
        ax1.fill_between(payloads, endurs, alpha=0.12, color=CYAN)
        ax1.plot(cur_mp, cur_e, "o", color=GREEN, ms=9, zorder=5,
                 label=f"Current: {cur_mp:.2f} kg  ->  {cur_e:.1f} min")

        target = 15.0
        good   = np.where(endurs >= target)[0]
        sp_load = sp_end = sp_pwr = 0.0
        si_sweet = -1
        if len(good) > 0:
            si_sweet = good[-1]
            sp_load  = payloads[si_sweet]
            sp_end   = endurs[si_sweet]
            sp_pwr   = powers[si_sweet]
            ax1.axvline(sp_load, color=ORANGE, lw=1.2, linestyle="--", alpha=0.7)
            ax1.axhline(target,  color=ORANGE, lw=1.0, linestyle=":",  alpha=0.6)
            ax1.plot(sp_load, sp_end, "*", color=ORANGE, ms=14, zorder=6,
                     label=f"Sweet Spot: {sp_load:.2f} kg @ {sp_end:.1f} min")
        ax1.legend(fontsize=7.5, facecolor="#112233", edgecolor=GREY,
                   labelcolor="white", loc="upper right")
        ax1.set_xlim(0, 3.0); ax1.set_ylim(bottom=0)

        # ── Plot 2: Endurance vs Power ─────────────────────────────────────
        sax(ax2, "FLIGHT ENDURANCE  vs  POWER CONSUMPTION",
            "Total Power Consumption (W)", "Endurance (minutes)")
        si2 = np.argsort(powers)
        ax2.plot(powers[si2], endurs[si2], color=RED, lw=2.2, zorder=3)
        ax2.fill_between(powers[si2], endurs[si2], alpha=0.10, color=RED)
        ax2.plot(ph["P_total"], cur_e, "o", color=GREEN, ms=9, zorder=5,
                 label=f"Current: {ph['P_total']:.1f} W  ->  {cur_e:.1f} min")
        ax2.legend(fontsize=7.5, facecolor="#112233", edgecolor=GREY,
                   labelcolor="white")

        # ── Plot 3: Pie chart + Summary text ───────────────────────────────
        ax3.axis("off"); ax3.set_facecolor("#0a0a1a")
        pos = ax3.get_position()
        pie_ax = self._fig_an.add_axes(
            [pos.x0, pos.y0 + pos.height * 0.45,
             pos.width, pos.height * 0.55])
        pie_ax.set_facecolor("#0a0a1a")
        sizes  = [ph["P_induced"], ph["P_elec"]]
        labels = [f"Propulsion\n{ph['P_induced']:.1f} W",
                  f"Electronics\n{ph['P_elec']:.1f} W"]
        _, texts, autotexts = pie_ax.pie(
            sizes, labels=labels, colors=[CYAN, ORANGE],
            explode=[0.05, 0.05], autopct="%1.1f%%", startangle=90,
            textprops={"color": "white", "fontsize": 7.5},
            wedgeprops={"linewidth": 2, "edgecolor": "#0a0a1a"})
        for at in autotexts:
            at.set_color("#0a0a1a"); at.set_fontweight("bold"); at.set_fontsize(8)
        pie_ax.set_title("POWER BREAKDOWN", color=CYAN, fontsize=9,
                         fontweight="bold", pad=6)

        if si_sweet >= 0:
            opt_lines = [
                f"  Payload  : {sp_load:.2f} kg",
                f"  Endurance: {sp_end:.1f} min",
                f"  Power    : {sp_pwr:.1f} W",
            ]
        else:
            opt_lines = ["  No solution >= 15 min found"]

        summary = "\n".join([
            "=== OPTIMAL CONDITION ===",
            *opt_lines,
            "",
            "=== YOUR CURRENT CONFIG ===",
            f"  Payload  : {cur_mp:.2f} kg",
            f"  Tot Mass : {ph['m_total']:.2f} kg",
            f"  Endurance: {cur_e:.1f} min",
            f"  Hover Pwr: {ph['P_total']:.1f} W",
            f"  Battery  : {ph['E_batt_Wh']:.0f} Wh",
        ])
        ax3.text(0.05, 0.40, summary,
                 transform=ax3.transAxes,
                 color="white", fontsize=7.5,
                 fontfamily="monospace", verticalalignment="top",
                 bbox=dict(boxstyle="round,pad=0.6", facecolor="#0f1a30",
                           edgecolor=CYAN, linewidth=1.2))

        self._fig_an.suptitle(
            f"UAV ENDURANCE ANALYSIS  |  r_prop={p['r_prop']:.2f}m  "
            f"eta={p['eta']:.2f}  Battery={p['m_battery']:.1f}kg x {p['batt_den']:.0f}Wh/kg",
            color=CYAN, fontsize=9, y=0.97)
        self._canvas_an.draw()


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 55)
    print("  UAV QUADCOPTER ENDURANCE SIMULATION DASHBOARD")
    print("  Launching window...")
    print("=" * 55)
    app = UAVSimApp()
    app.mainloop()
