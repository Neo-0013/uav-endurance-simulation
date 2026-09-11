# 📚 UAV Quadcopter Endurance Modeling — Comprehensive Academic Report Guide

> **Notice to Students & Contributors:**  
> Use this document as your master blueprint when writing your formal academic project report, laboratory dissertation, or seminar presentation. It details every equation, design decision, software framework, and theoretical principle implemented in this simulation, structured from fundamental concepts to advanced engineering implementations.

---

## Table of Contents
1. [Report Outline Template](#1-report-outline-template)
2. [Chapter 1: Introduction & Literature Review](#chapter-1-introduction--literature-review)
3. [Chapter 2: Software Technology Stack & Architecture](#chapter-2-software-technology-stack--architecture)
4. [Chapter 3: Aerodynamic & Mathematical Modeling](#chapter-3-aerodynamic--mathematical-modeling)
5. [Chapter 4: Control Systems & 6-DOF Dynamics](#chapter-4-control-systems--6-dof-dynamics)
6. [Chapter 5: Detailed Software Module Implementation](#chapter-5-detailed-software-module-implementation)
7. [Chapter 6: Experimental Analysis & Discussion (The Graphs)](#chapter-6-experimental-analysis--discussion-the-graphs)
8. [Chapter 7: Conclusion & Future Scope (ROS2 Integration)](#chapter-7-conclusion--future-scope-ros2-integration)
9. [Appendix: Viva & Oral Defense Q&A Preparation](#appendix-viva--oral-defense-qa-preparation)

---

## 1. Report Outline Template

If your university requires standard IEEE, Springer, or university report formatting, use the following structure:

* **Cover Page:** Project Title, Student Name(s), Roll Number(s), Department, Institution, Date
* **Abstract:** 200–250 words summarizing the problem, software methodology, and key results.
* **Keywords:** UAV, Multirotor, Momentum Theory, Flight Endurance, Aerodynamics, PyQt6, Panda3D, PID Controller.
* **Chapter 1:** Introduction & Motivation
* **Chapter 2:** Software Technology Stack & Architecture
* **Chapter 3:** Mathematical & Aerodynamic Formulation
* **Chapter 4:** Flight Control & Dynamic Kinematics
* **Chapter 5:** Software Implementation & Data Pipeline
* **Chapter 6:** Parametric Experiments & Results
* **Chapter 7:** Discussion & The Optimal Operating Condition
* **Chapter 8:** Conclusions & Future Roadmap
* **References:** Academic citations (Leishman, Seddon, Newman, etc.)

---

## Chapter 1: Introduction & Literature Review

### 1.1 The Fundamental UAV Problem
Unmanned Aerial Vehicles (UAVs)—specifically multirotors such as quadcopters—have become ubiquitous across industrial inspection, precision agriculture, search-and-rescue, and aerial photography.

However, unlike fixed-wing aircraft which generate lift passively through aerodynamic wing profiles as forward velocity increases (high Lift-to-Drag ratio $L/D \approx 10-20$), a multirotor relies purely on **rotary wing propulsion** to fight gravity ($L/D \approx 1$). Every second the vehicle remains airborne, its motors must continuously push air downward. Consequently, multirotor flight endurance is typically restricted to **15 to 30 minutes**, making endurance optimization the single most critical constraint in UAV mission planning.

### 1.2 The "Battery Mass vs. Payload" Paradox
A naive assumption in drone design is that flight time can be arbitrarily extended by adding larger batteries. In practice, batteries possess finite specific energy density ($\sim 130 - 200 \text{ Wh/kg}$ for Lithium-Polymer chemistry). 

Adding battery mass increases total aircraft weight, which exponentially increases the thrust required to maintain hover. At a certain threshold—the **Point of Diminishing Returns**—the additional energy stored in the battery is entirely consumed by lifting the battery's own weight, resulting in declining endurance.

---

## Chapter 2: Software Technology Stack & Architecture

### 2.1 Software Stack Selection & Justification

| Technology | Role in Project | Why This Was Chosen Over Alternatives |
|---|---|---|
| **Python 3.10+** | Core Programming Language | High numerical productivity, native bindings to aerospace & GUI libraries, rapid prototyping. |
| **PyQt6 (Qt 6)** | Ground Control Station (GCS) GUI | Professional-grade C++ Qt backend. Chosen over Tkinter (too rudimentary) and Electron (excessive 300MB+ RAM overhead). |
| **Panda3D (v1.10+)** | 3D Simulation Engine | Disney and Carnegie Mellon's open-source C++ game engine. Chosen over Unity/Unreal (too heavy, requires separate language) and Pygame (too slow for 3D). Runs at 60 FPS on low-end Intel HD graphics. |
| **NumPy** | High-Performance Math | Vectorized numerical arrays for payload sweeps, PID integrations, and atmospheric matrices. |
| **Matplotlib** | Scientific Data Visualization | Embedded directly into PyQt6 using `FigureCanvasQTAgg` to render high-resolution 2D engineering plots. |
| **ReportLab** | Automated PDF Generation | Native PDF rendering engine generating printable, vector-quality flight reports with flowables and tables. |

---

### 2.2 Decoupled Multi-Rate Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                 PyQt6 Ground Control Station                │
│  • 20 Hz Aerodynamic Physics & Numerical Integration        │
│  • Active PID Altitude Controller & 6-DOF Attitude Coupling │
│  • 5 Dashboard Panels (Config, Telemetry, Plots, Mission)   │
│  • Automated PDF Report Generator (ReportLab)               │
└──────────────────────────────┬──────────────────────────────┘
                               │  Thread-Safe Python Queues
                               │  (Telemetry state_q / Commands cmd_q)
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                 Panda3D 3D Visualizer                       │
│  • 60 FPS Hardware-Accelerated C++ Rendering Pipeline       │
│  • 100% Procedural Drone & Terrain Meshes (Zero asset lag)  │
│  • Real-time VFX (Downwash dust particles, smoke trail)     │
│  • Tri-Mode Dynamic Camera (Follow, FPV, Free Orbit)        │
└─────────────────────────────────────────────────────────────┘
```

#### Why Decoupled Multi-Rate?
* **Physics Loop (20 Hz / 50ms):** Aerodynamic calculations, battery integration, and controller signals do not need 60Hz evaluation. Running physics at 20 Hz conserves CPU cycles.
* **Rendering Loop (60 FPS / 16ms):** Human visual perception demands 60 FPS for smooth motion and camera control. Driven via `QTimer` calling `panda_app.step()`.
* **Zero GIL Contention:** By passing data through non-blocking Python `queue.Queue` objects (`state_q` and `cmd_q`), the UI event loop and graphics engine communicate with microsecond latency without locking or freezing the desktop window.

---

## Chapter 3: Aerodynamic & Mathematical Modeling

### 3.1 Actuator Disk / Rankine-Froude Momentum Theory
The simulation models rotor aerodynamics using **Momentum Theory**, which treats each propeller as an infinitesimally thin circular disk across which a uniform pressure jump occurs without rotational swirl losses.

#### 3.1.1 Conservation of Mass
The mass flow rate $\dot{m}$ of air passing through the rotor disk of area $A$ with induced velocity $v_i$ is:
$$\dot{m} = \rho A v_i$$
where $\rho$ is the ambient air density ($\text{kg/m}^3$) and $A = 4 \times (\pi r_{\text{prop}}^2)$ for a 4-rotor quadcopter.

#### 3.1.2 Conservation of Momentum
Assuming far upstream velocity is zero ($v_\infty = 0$) in steady hover, the velocity in the far slipstream (vena contracta) reaches $w = 2 v_i$. The total upward thrust $T$ generated equals the rate of change of momentum:
$$T = \dot{m} w = (\rho A v_i) (2 v_i) = 2 \rho A v_i^2$$

Solving for the **induced velocity** ($v_i$):
$$v_i = \sqrt{\frac{T}{2 \rho A}}$$

#### 3.1.3 Induced Hover Power ($P_{\text{induced}}$)
The ideal power required to impart kinetic energy to this air column is the product of thrust and induced velocity:
$$P_{\text{induced}} = T \cdot v_i = T \sqrt{\frac{T}{2 \rho A}} = \frac{T^{3/2}}{\sqrt{2 \rho A}}$$

> **Key Equation to Highlight in Report:**  
> **$P_{\text{induced}} \propto T^{1.5}$**  
> Power scales with thrust to the power of 1.5. Doubling the aircraft weight requires $2^{1.5} \approx 2.83$ times more power!

### 3.2 Total Electrical Power Consumption
Real-world quadcopters incur electrical, transmission, and environmental drag penalties:
$$P_{\text{total}} = \frac{P_{\text{induced}}}{\eta_{\text{motor}}} + P_{\text{electronics}} + P_{\text{wind}}$$

* **$\eta_{\text{motor}}$ (Combined Efficiency):** Typically $0.50 - 0.70$ (50%–70%), representing brushless DC motor (BLDC) copper losses, iron losses, and Electronic Speed Controller (ESC) switching resistance.
* **$P_{\text{electronics}}$:** Continuous baseline overhead for flight computer, GPS, radios, and telemetry ($\sim 10 - 20\text{ W}$).
* **$P_{\text{wind}}$ (Parasitic Drag Power):**
  $$P_{\text{wind}} = \frac{1}{2} \rho V_{\text{wind}}^3 C_D A_{\text{front}}$$
  Parasitic drag increases with the cube of wind velocity ($V^3$).

### 3.3 International Standard Atmosphere (ISA) Model
Air density $\rho$ is not constant; it diminishes with altitude and higher temperatures:
$$\rho(h, T) = \frac{p(h)}{R_{\text{specific}} \cdot T_{\text{Kelvin}}}$$
$$p(h) = p_0 \cdot \left(1 - \frac{L \cdot h}{T_0}\right)^{\frac{g \cdot M}{R_0 \cdot L}}$$
Because $\rho$ is in the denominator ($\sqrt{2 \rho A}$), high-altitude or high-temperature environments degrade hover efficiency.

### 3.4 Battery Discharge & Endurance Formulation
The flight endurance $E_{\text{time}}$ in minutes is formulated using an 80% Depth-of-Discharge (DoD) safe limit:
$$E_{\text{usable}} = m_{\text{battery}} \times \text{SpecificEnergy} (\text{Wh/kg}) \times 0.80$$
$$\text{Endurance (min)} = \frac{E_{\text{usable}} (\text{Wh}) \times 60}{P_{\text{total}} (\text{W})}$$

---

## Chapter 4: Control Systems & 6-DOF Dynamics

### 4.1 PID Altitude Controller
The vertical axis is governed by a closed-loop Proportional-Integral-Derivative (PID) controller:
$$e(t) = z_{\text{target}} - z(t)$$
$$a_{\text{req}}(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$
$$T_{\text{command}} = m_{\text{total}} \cdot (g + a_{\text{req}})$$

* **Anti-Windup:** The integral accumulator is clamped to $[-10, 10]$ to prevent integrator windup during sustained climbs.
* **Thrust Limiter:** Thrust is bounded by the motor ceiling: $0 \le T \le 2.8 \times m_{\text{total}} g$.

### 4.2 6-DOF Attitude Kinematics
When horizontal velocity changes, the vehicle visual model tilts proportionally to acceleration:
$$\text{Roll Angle } \phi = \arctan\left(\frac{v_x \cdot k}{g}\right)$$
$$\text{Pitch Angle } \theta = \arctan\left(\frac{v_y \cdot k}{g}\right)$$

---

## Chapter 5: Detailed Software Module Implementation

In this chapter, detail how the code is organized into modular subsystems:

### 5.1 Core Subsystems Overview

```
uav-endurance-simulation/
├── main.py                 # System launcher & frame synchronization loop
├── physics.py              # Pure math & aerodynamic modeling engine
├── sim3d/                  # 3D Visualizer Package (Panda3D)
│   ├── app.py              # Window management, HUD, & task manager step
│   ├── drone.py            # Procedural 3D drone mesh, rotor spin, 6-DOF tilt
│   ├── environment.py      # Procedural terrain plane, lighting, & fog
│   ├── camera.py           # Camera controller (Follow, FPV, Orbit)
│   ├── particles.py        # CardMaker particle system for downwash & smoke
│   └── geometry_utils.py   # Procedural vertex buffer primitives
├── ui/                     # Ground Control Station Package (PyQt6)
│   ├── main_window.py      # Base dashboard with sidebar & persistent QStackedWidget
│   ├── main_window_3d.py   # Telemetry bridge pushing state at 20Hz & polling keys
│   ├── panel_home.py       # Configuration sliders with real-time parameter card
│   ├── panel_sim.py        # Flight telemetry HUD, live strip-charts, 3 flight modes
│   ├── panel_analysis.py   # 4 Matplotlib endurance plots & optimal condition solver
│   ├── panel_mission.py    # 2D click-to-place GPS waypoint planner & path solver
│   └── panel_report.py     # PDF export preview panel
└── export/
    └── report.py           # ReportLab automated PDF generator
```

### 5.2 Key Software Design Decisions

1. **Persistent `QStackedWidget` Navigation:**
   * Standard tab widgets often recreate child widgets when navigating away.
   * `QStackedWidget` hides non-active panels while keeping their internal timers and state variables completely alive. Switching to the Mission Planner does not pause or kill the running flight simulation.
2. **Procedural Geometry (Zero-Asset Architecture):**
   * Instead of loading high-polygon `.obj` or `.glb` files from disk (which fail if files are missing or paths break), `geometry_utils.py` generates boxes, cylinders, and disks directly in memory using Panda3D's `GeomVertexWriter` and `GeomTriangles`.
   * Result: Instant startup, zero asset path errors, and total memory usage under 100 MB.
3. **Focus Isolation (`Qt.FocusPolicy.NoFocus`):**
   * All GUI buttons on the dashboard have `NoFocus` assigned. This prevents the OS spacebar key (which climbs the drone) from inadvertently triggering a focused button's `clicked()` signal.

---

## Chapter 6: Experimental Analysis & Discussion (The Graphs)

When presenting your results chapter, detail these four experimental graphs:

### 6.1 Flight Endurance vs. Payload Mass
* **Observation:** Curve exhibits a steep initial decline, tapering as payload approaches maximum allowable limit.
* **Theoretical Reason:** Direct consequence of $P \propto T^{1.5}$. At $0\text{ kg}$ payload, endurance reaches $\sim 24\text{ min}$. At $1.5\text{ kg}$, endurance drops below $10\text{ min}$.
* **The Sweet Spot Definition:** The intersection with the 15-minute mission threshold line ($\approx 0.81\text{ kg}$ for standard configuration).

### 6.2 Flight Endurance vs. Power Consumption
* **Observation:** Hyperbolic decay curve ($y \propto 1/x$).
* **Theoretical Reason:** Fixed energy pool divided by varying power draw ($t = E/P$).

### 6.3 Power Breakdown Analysis
* **Observation:** Propulsion accounts for $92\%-96\%$ of total power.
* **Theoretical Reason:** Avionics consume fixed baseline power ($\sim 15\text{ W}$), whereas lifting a $2.5\text{ kg}$ mass against Earth's gravity demands hundreds of watts.

### 6.4 Wind Speed Sensitivity
* **Observation:** Minor impact under $5\text{ m/s}$, catastrophic drop past $12\text{ m/s}$.
* **Theoretical Reason:** Parasitic aerodynamic drag scales with the cube of airspeed ($P_{\text{wind}} \propto V^3$).

---

## Chapter 7: Conclusion & Future Scope (ROS2 Integration)

### 7.1 Summary of Contributions
* Developed a multi-variable aerodynamic UAV endurance model.
* Implemented a dual-window Ground Control Station and 60 FPS 3D visualizer.
* Solved for optimal operating conditions under varying payload and atmospheric conditions.
* Implemented automated PDF engineering report generation.

### 7.2 Future Roadmap: ROS2 Migration
The decoupled architecture of this simulation was deliberately structured to map 1:1 to **ROS2 (Robot Operating System)** nodes:
* `/uav_physics_node`: Publishes `sensor_msgs/BatteryState` and `nav_msgs/Odometry`.
* `/uav_render_node`: Subscribes to odometry for Panda3D/RViz visualization.
* `/uav_gcs_node`: Publishes `geometry_msgs/Twist` via `/cmd_vel` from PyQt6.

---

## Appendix: Viva & Oral Defense Q&A Preparation

| Question | Recommended Answer |
|---|---|
| **Why Momentum Theory instead of Blade Element Theory (BEMT)?** | *"Momentum Theory provides an exact analytical solution for global induced hover power with minimal computational overhead, enabling 20 Hz real-time simulation without needing rotor blade airfoil polar tables."* |
| **Why does power scale with $T^{1.5}$ instead of linearly?** | *"Because thrust increases both the mass flow rate and the induced velocity simultaneously ($T = 2\rho A v_i^2$). Since Power is Thrust $\times$ Velocity ($P = T \cdot v_i$), substituting $v_i \propto \sqrt{T}$ yields $P \propto T^{1.5}$."* |
| **Why limit battery discharge to 80%?** | *"Lithium-Polymer cell voltages collapse precipitously past 80% Depth-of-Discharge (DoD). Discharging beyond this threshold causes permanent internal dendrite growth, cell swelling, and in-flight voltage sag failures."* |
| **Why does the drone use Panda3D instead of Pygame?** | *"Panda3D is a native C++ engine with built-in scene graphs, lighting, fog, and particle systems. It delivers steady 60 FPS on low-end Intel HD graphics where Pygame 3D CPU math would severely lag."* |
| **How do PyQt6 and Panda3D talk to each other?** | *"They run in a single process communicating via two thread-safe FIFO queues (`queue.Queue`). PyQt pushes telemetry states at 20 Hz, and Panda3D pushes keyboard events back to Qt at 50 Hz with zero GUI blocking."* |
