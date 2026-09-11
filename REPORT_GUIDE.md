# 📚 UAV Quadcopter Endurance Modeling — Comprehensive Academic Report Guide

> **Notice to Students & Contributors:**  
> Use this document as your blueprint when writing your formal academic project report, laboratory dissertation, or seminar presentation. It details every equation, design decision, and theoretical principle implemented in this simulation, structured from fundamental concepts to advanced aerodynamic modeling.

---

## Table of Contents
1. [Report Outline Template](#1-report-outline-template)
2. [Chapter 1: Introduction & Literature Review](#chapter-1-introduction--literature-review)
3. [Chapter 2: Aerodynamic & Mathematical Modeling](#chapter-2-aerodynamic--mathematical-modeling)
4. [Chapter 3: Control Systems & 6-DOF Dynamics](#chapter-3-control-systems--6-dof-dynamics)
5. [Chapter 4: Software Architecture & Decoupled Engineering](#chapter-4-software-architecture--decoupled-engineering)
6. [Chapter 5: Experimental Analysis & Discussion (The Graphs)](#chapter-5-experimental-analysis--discussion-the-graphs)
7. [Chapter 6: Conclusion & Future Scope (ROS2 Integration)](#chapter-6-conclusion--future-scope-ros2-integration)
8. [Appendix: Viva & Oral Defense Q&A Preparation](#appendix-viva--oral-defense-qa-preparation)

---

## 1. Report Outline Template

If your university requires standard IEEE, Springer, or university report formatting, use the following structure:

* **Cover Page:** Project Title, Student Name(s), Roll Number(s), Department, Institution, Date
* **Abstract:** 200–250 words summarizing the problem, methodology, and key results.
* **Keywords:** UAV, Multirotor, Momentum Theory, Flight Endurance, Aerodynamics, PID Controller, LiPo Battery.
* **Chapter 1:** Introduction & Motivation
* **Chapter 2:** Mathematical & Aerodynamic Formulation
* **Chapter 3:** Simulation System Design & Implementation
* **Chapter 4:** Parametric Experiments & Results
* **Chapter 5:** Discussion & The Optimal Operating Condition
* **Chapter 6:** Conclusions & Future Roadmap
* **References:** Academic citations (Leishman, Seddon, Newman, etc.)

---

## Chapter 1: Introduction & Literature Review

### 1.1 The Fundamental UAV Problem
Unmanned Aerial Vehicles (UAVs)—specifically multirotors such as quadcopters—have become ubiquitous across industrial inspection, precision agriculture, search-and-rescue, and aerial photography.

However, unlike fixed-wing aircraft which generate lift passively through aerodynamic wing profiles as forward velocity increases (high Lift-to-Drag ratio $L/D \approx 10-20$), a multirotor relies purely on **rotary wing propulsion** to fight gravity ($L/D \approx 1$). Every second the vehicle remains airborne, its motors must continuously push mass downward. Consequently, multirotor flight endurance is typically restricted to **15 to 30 minutes**, making endurance optimization the single most critical constraint in UAV mission planning.

### 1.2 The "Battery Mass vs. Payload" Paradox
A naive assumption in drone design is that flight time can be arbitrarily extended by adding larger batteries. In practice, batteries possess finite specific energy density ($\sim 130 - 200 \text{ Wh/kg}$ for Lithium-Polymer chemistry). 

Adding battery mass increases total aircraft weight, which exponentially increases the thrust required to maintain hover. At a certain threshold—the **Point of Diminishing Returns**—the additional energy stored in the battery is entirely consumed by lifting the battery's own weight, resulting in declining endurance.

---

## Chapter 2: Aerodynamic & Mathematical Modeling

### 2.1 Actuator Disk / Rankine-Froude Momentum Theory
The simulation models rotor aerodynamics using **Momentum Theory**, which treats each propeller as an infinitesimally thin circular disk across which a uniform pressure jump occurs without rotational swirl losses.

#### 2.1.1 Conservation of Mass
The mass flow rate $\dot{m}$ of air passing through the rotor disk of area $A$ with induced velocity $v_i$ is:
$$\dot{m} = \rho A v_i$$
where $\rho$ is the ambient air density ($\text{kg/m}^3$) and $A = 4 \times (\pi r_{\text{prop}}^2)$ for a 4-rotor quadcopter.

#### 2.1.2 Conservation of Momentum
Assuming far upstream velocity is zero ($v_\infty = 0$) in steady hover, the velocity in the far slipstream (vena contracta) reaches $w = 2 v_i$. The total upward thrust $T$ generated equals the rate of change of momentum:
$$T = \dot{m} w = (\rho A v_i) (2 v_i) = 2 \rho A v_i^2$$

Solving for the **induced velocity** ($v_i$):
$$v_i = \sqrt{\frac{T}{2 \rho A}}$$

#### 2.1.3 Induced Hover Power ($P_{\text{induced}}$)
The ideal power required to impart kinetic energy to this air column is the product of thrust and induced velocity:
$$P_{\text{induced}} = T \cdot v_i = T \sqrt{\frac{T}{2 \rho A}} = \frac{T^{3/2}}{\sqrt{2 \rho A}}$$

> **Key Equation to Highlight in Report:**  
> **$P_{\text{induced}} \propto T^{1.5}$**  
> Power scales with thrust to the power of 1.5. Doubling the aircraft weight requires $2^{1.5} \approx 2.83$ times more power!

### 2.2 Total Electrical Power Consumption
Real-world quadcopters incur electrical, transmission, and environmental drag penalties:
$$P_{\text{total}} = \frac{P_{\text{induced}}}{\eta_{\text{motor}}} + P_{\text{electronics}} + P_{\text{wind}}$$

* **$\eta_{\text{motor}}$ (Combined Efficiency):** Typically $0.50 - 0.70$ (50%–70%), representing brushless DC motor (BLDC) copper losses, iron losses, and Electronic Speed Controller (ESC) switching resistance.
* **$P_{\text{electronics}}$:** Continuous baseline overhead for flight computer, GPS, radios, and telemetry ($\sim 10 - 20\text{ W}$).
* **$P_{\text{wind}}$ (Parasitic Drag Power):**
  $$P_{\text{wind}} = \frac{1}{2} \rho V_{\text{wind}}^3 C_D A_{\text{front}}$$
  Parasitic drag increases with the cube of wind velocity ($V^3$).

### 2.3 International Standard Atmosphere (ISA) Model
Air density $\rho$ is not constant; it diminishes with altitude and higher temperatures:
$$\rho(h, T) = \frac{p(h)}{R_{\text{specific}} \cdot T_{\text{Kelvin}}}$$
$$p(h) = p_0 \cdot \left(1 - \frac{L \cdot h}{T_0}\right)^{\frac{g \cdot M}{R_0 \cdot L}}$$
Because $\rho$ is in the denominator ($\sqrt{2 \rho A}$), high-altitude or high-temperature environments degrade hover efficiency.

### 2.4 Battery Discharge & Endurance Formulation
The flight endurance $E_{\text{time}}$ in minutes is formulated using an 80% Depth-of-Discharge (DoD) safe limit:
$$E_{\text{usable}} = m_{\text{battery}} \times \text{SpecificEnergy} (\text{Wh/kg}) \times 0.80$$
$$\text{Endurance (min)} = \left( \frac{E_{\text{usable}} (\text{Wh}) \times 3600 \text{ s/Wh}}{P_{\text{total}} (\text{W})} \right) / 60 = \frac{E_{\text{usable}} \times 60}{P_{\text{total}}}$$

---

## Chapter 3: Control Systems & 6-DOF Dynamics

### 3.1 PID Altitude Controller
The vertical axis is governed by a closed-loop Proportional-Integral-Derivative (PID) controller:
$$e(t) = z_{\text{target}} - z(t)$$
$$a_{\text{req}}(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$
$$T_{\text{command}} = m_{\text{total}} \cdot (g + a_{\text{req}})$$

* **Anti-Windup:** The integral term is clamped to $[-10, 10]$ to prevent integrator windup during sustained climbs.
* **Thrust Limiter:** Thrust is bounded by the motor ceiling: $0 \le T \le 2.8 \times m_{\text{total}} g$.

### 3.2 6-DOF Attitude Coupling
When horizontal velocity changes, the vehicle visual model tilts proportionally to acceleration:
$$\text{Roll Angle } \phi = \arctan\left(\frac{v_x \cdot k}{g}\right)$$
$$\text{Pitch Angle } \theta = \arctan\left(\frac{v_y \cdot k}{g}\right)$$

---

## Chapter 4: Software Architecture & Decoupled Engineering

### 4.1 The Dual-Process Paradigm
Standard Python scripts attempting to run Matplotlib plots alongside OpenGL 3D graphics in a single loop suffer from severe GIL (Global Interpreter Lock) contention and sub-15 FPS framerates.

This project implements an industrial decoupled architecture:
1. **PyQt6 Ground Station (Worker at 20 Hz):** Runs physics integration, collects telemetry, updates mini-charts, and handles user UI input.
2. **Panda3D Visualizer (Master at 60 FPS):** C++ accelerated game engine that receives kinematics via a non-blocking `queue.Queue`.
3. **Procedural Geometry Pipeline:** The drone airframe, rotors, landing gear, and terrain are generated dynamically via mathematical vertex buffers (`GeomVertexData`), eliminating external 3D asset dependencies.

---

## Chapter 5: Experimental Analysis & Discussion (The Graphs)

When presenting your results chapter, detail these four experimental graphs:

### 5.1 Flight Endurance vs. Payload Mass
* **Observation:** Curve exhibits a steep initial decline, tapering as payload approaches maximum allowable limit.
* **Theoretical Reason:** Direct consequence of $P \propto T^{1.5}$. At $0\text{ kg}$ payload, endurance reaches $\sim 24\text{ min}$. At $1.5\text{ kg}$, endurance drops below $10\text{ min}$.
* **The Sweet Spot Definition:** The intersection with the 15-minute mission threshold line ($\approx 0.81\text{ kg}$ for standard configuration).

### 5.2 Flight Endurance vs. Power Consumption
* **Observation:** Hyperbolic decay curve ($y \propto 1/x$).
* **Theoretical Reason:** Fixed energy pool divided by varying power draw ($t = E/P$).

### 5.3 Power Breakdown Analysis
* **Observation:** Propulsion accounts for $92\%-96\%$ of total power.
* **Theoretical Reason:** Avionics consume fixed baseline power ($\sim 15\text{ W}$), whereas lifting a $2.5\text{ kg}$ mass against Earth's gravity demands hundreds of watts.

### 5.4 Wind Speed Sensitivity
* **Observation:** Minor impact under $5\text{ m/s}$, catastrophic drop past $12\text{ m/s}$.
* **Theoretical Reason:** Parasitic aerodynamic drag scales with the cube of airspeed ($P_{\text{wind}} \propto V^3$).

---

## Chapter 6: Conclusion & Future Scope (ROS2 Integration)

### 6.1 Summary of Contributions
* Developed a multi-variable aerodynamic UAV endurance model.
* Implemented a dual-window Ground Control Station and 60 FPS 3D visualizer.
* Solved for optimal operating conditions under varying payload and atmospheric conditions.
* Implemented automated PDF engineering report generation.

### 6.2 Future Roadmap: ROS2 Migration
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
