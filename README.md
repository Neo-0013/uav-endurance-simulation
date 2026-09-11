# 🛸 Autonomous UAV Quadcopter Flight & Endurance Simulator

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![Framework](https://img.shields.io/badge/GUI-PyQt6-green.svg)](https://riverbankcomputing.com/software/pyqt/)
[![3D Engine](https://img.shields.io/badge/3D%20Engine-Panda3D-orange.svg)](https://www.panda3d.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

An advanced, dual-window Unmanned Aerial Vehicle (UAV) Ground Control Station (GCS) and real-time 3D flight simulation. Built to model **Rankine-Froude Momentum Theory**, **International Standard Atmosphere (ISA)** dynamics, **6-DOF flight kinematics**, and **Lithium-Polymer battery discharge curves** under variable payloads and environmental stressors.

---

## 📸 System Architecture

The simulation employs a **decoupled, multi-rate architecture** in a single Python process, mirroring industrial avionics stacks (e.g., ArduPilot + RViz + QGroundControl):

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

---

## ✨ Features

- ⚡ **Non-Linear Aerodynamic Modeling:** Uses Momentum Theory ($P_{\text{induced}} \propto T^{1.5}$) to calculate power demand. Accurately simulates the heavy endurance penalty of adding payload.
- 🌍 **Atmospheric & Environmental Scaling:** Incorporates the International Standard Atmosphere (ISA) to compute air density ($\rho$) dynamically across altitude ($0 - 4000\text{ m}$) and ambient temperature ($-10^\circ\text{C to } 45^\circ\text{C}$), plus parasitic wind drag.
- 🔋 **LiPo Battery Model:** Implements dynamic Coulomb counting with an 80% Depth-of-Discharge (DoD) safety threshold to prevent battery over-discharge.
- 🎯 **Sweet Spot Optimization:** Automatically solves for the **Optimal Operating Condition**—the maximum payload mass that still meets a 15-minute mission endurance requirement.
- 🎮 **3 Selectable Flight Modes:**
  - **Interactive (WASD):** Manual flight with active PID altitude control and responsive banking.
  - **Autonomous Mission:** Automated waypoint navigation with route feasibility checks.
  - **Stress Test:** Full-throttle load simulation to analyze extreme battery drain.
- 📄 **1-Click PDF Report Export:** Generates an engineering report complete with configuration parameters, computed physics, high-resolution Matplotlib graphs, and optimization analysis.
- 💻 **Optimized for Low-End Hardware:** Procedurally generated meshes and decoupled loops ensure smooth 60 FPS performance on low-end laptops and integrated graphics (Intel HD).

---

## 📦 Installation & Setup

### 1. Prerequisites
- **Python 3.10, 3.11, or 3.12**
- Git

### 2. Clone the Repository
```bash
git clone https://github.com/<your-username>/uav-endurance-simulation.git
cd uav-endurance-simulation
```

### 3. Create a Virtual Environment (Recommended)
```bash
# Linux / macOS
python3 -m venv venv
source venv/bin/activate

# Windows (Command Prompt)
python -m venv venv
venv\Scripts\activate
```

### 4. Install Dependencies
```bash
pip install -r requirements.txt
```

> **Note for Debian/Ubuntu (PEP 668):**
> If you get an `externally-managed-environment` error and wish to install globally without a virtual environment, run:
> ```bash
> pip install -r requirements.txt --break-system-packages
> ```

---

## 🚀 How to Run

Launch the complete application with:

```bash
python3 main.py
```
*(On Debian/Ubuntu systems with multiple Python versions, you can also run `/usr/bin/python3 main.py`)*

Two windows will open side by side:
1. **Left Window:** Panda3D Real-time 3D Flight Visualizer.
2. **Right Window:** PyQt6 Ground Control Station Dashboard.

---

## 🕹️ Flight Controls

| Key | Action |
|---|---|
| **Space** | Climb / Increase altitude |
| **Ctrl** | Descend / Lower altitude |
| **W / Up Arrow** | Pitch forward (fly forward) |
| **S / Down Arrow** | Pitch backward (fly backward) |
| **A / Left Arrow** | Roll left (strafe left) |
| **D / Right Arrow** | Roll right (strafe right) |
| **C** | Cycle Camera Mode (`FOLLOW` → `FPV` → `ORBIT`) |
| **R** | Emergency Reset / Land drone |
| **Left Click + Drag** | Rotate camera (in `ORBIT` mode) |
| **Mouse Scroll** | Zoom in / Zoom out (in `ORBIT` mode) |

---

## 📐 Mathematical Formulation

### 1. Hover Thrust & Induced Velocity
To maintain steady hover, total thrust must balance total aircraft weight:
$$T = W = m_{\text{total}} \cdot g = (m_{\text{frame}} + m_{\text{payload}} + m_{\text{battery}}) \cdot g$$

According to Rankine-Froude Momentum Theory, the induced velocity $v_i$ through the rotor disks is:
$$v_i = \sqrt{\frac{T}{2 \rho A_{\text{disk}}}}$$
where $A_{\text{disk}} = 4 \times (\pi r_{\text{prop}}^2)$ is the total actuator disk area.

### 2. Power Consumption
Total electrical power draw accounts for aerodynamic induced power, electrical/mechanical motor losses ($\eta$), baseline avionics power, and wind drag:
$$P_{\text{induced}} = T \cdot v_i = \frac{T^{3/2}}{\sqrt{2 \rho A_{\text{disk}}}}$$
$$P_{\text{total}} = \frac{P_{\text{induced}}}{\eta_{\text{motor}}} + P_{\text{electronics}} + P_{\text{wind\_drag}}$$

### 3. Flight Endurance
Using an 80% Depth-of-Discharge (DoD) safety buffer:
$$\text{Endurance (minutes)} = \frac{E_{\text{battery}} \times 0.80}{P_{\text{total}}} \times 60$$

---

## 📁 Repository Structure

```
uav-endurance-simulation/
├── main.py                 # Main entry point (launches dual windows)
├── physics.py              # Aerodynamics, Momentum Theory, ISA & PID engine
├── requirements.txt        # Python package dependencies
├── README.md               # Project documentation & user guide
├── REPORT_GUIDE.md         # Comprehensive academic report writing guide
├── assets/
│   └── styles.qss          # GCS Dark Navy/Teal Qt stylesheet
├── sim3d/                  # Panda3D 3D Graphics Engine
│   ├── app.py              # Panda3D ShowBase application loop
│   ├── drone.py            # Procedural 4-arm quadcopter model & kinematics
│   ├── environment.py      # Procedural terrain, sky, lighting, and trees
│   ├── camera.py           # 3-mode camera controller (Follow/FPV/Orbit)
│   ├── particles.py        # VFX Particle system (downwash dust, smoke, crash)
│   └── geometry_utils.py   # Procedural vertex buffer geometry primitives
├── ui/                     # PyQt6 Ground Control Station
│   ├── main_window.py      # Base dashboard window & sidebar navigation
│   ├── main_window_3d.py   # Synchronized window with queue telemetry bridge
│   ├── panel_home.py       # Configuration sliders & live values
│   ├── panel_sim.py        # Telemetry HUD, live mini-charts & flight controller
│   ├── panel_analysis.py   # 4 Matplotlib endurance plots & sweet spot card
│   ├── panel_mission.py    # 2D click-to-place waypoint mission planner
│   └── panel_report.py     # PDF export preview panel
└── export/
    └── report.py           # ReportLab PDF report generation engine
```

---

## 📄 Academic Report Guide

Need to prepare an academic project report or defense presentation for your university course?
See the included **[REPORT_GUIDE.md](REPORT_GUIDE.md)** for a complete, section-by-section breakdown containing:
- Theoretical derivations
- Experimental methodology
- Analysis of graphs and sweet spots
- Viva / Oral defense questions & answers

---

## 📜 License
This project is open-source under the [MIT License](LICENSE). Feel free to use and modify it for academic and research purposes!
